#include "mcode.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "motor.h"
#include "usart.h"
#include "homing.h" /* [修改] 引入 homing_request / Homing_IsActive / Home_Start */
#include "system.h" /* [新增] 引入 current_config / System_ApplyNoteConfig（音数自动切换） */
#include "notes.h" /* [新增] 自检回中用 Notes_ConvertMMToPulses */
#include "fileio.h" /* [修复] FileIO_NotifyLineDoneOnly（M80 组合行只置 line_done 不回 OK） */
#include "handle.h" /* [修复] handle_enable：手柄模式与 homing 互斥 */

/* 回调指针（由用户注册） */
static mcb_move_t cb_move = NULL;
static mcb_feed_t cb_feed = NULL;
static mcb_punch_t cb_punch = NULL;
static mcb_s_t cb_s = NULL;
static mcb_evt_t cb_batch_start = NULL;
static mcb_evt_t cb_batch_end = NULL;
static send_ok_t cb_send_ok = NULL;

/* pending 命令存放 */
parsed_cmd_t pending_cmd;
static volatile uint8_t have_pending = 0;

/* 执行状态标志（pending 命令内部） */
static uint8_t startedM = 0;
static uint8_t startedY = 0;
static uint8_t startedP = 0;
static uint8_t appliedS = 0;

/* busy 标志（仅当真实启动异步动作时置 1） */
volatile uint8_t move_busy = 0;
volatile uint8_t feed_busy = 0;
volatile uint8_t punch_busy = 0;

/* mcode 会话标志 */
static volatile uint8_t in_mcode = 0;

/* 如果某条行包含 M80 与其它字段，则延后退出 mcode，直到该 pending 完成 */
static uint8_t pending_end_on_complete = 0;

/* 该 pending 在完成时是否需要发送 OK（基于当时 in_mcode 与是否有动作） */
static uint8_t pending_send_ok = 0;

/* 如果该 pending 是由含 M80 的行触发（即需要延后退出 mcode），则在完成时不要发送 OK */
static uint8_t pending_no_ok_on_complete = 0;

/* ---------------- OK 重发机制（仅在 mcode 模式生效） ---------------- */
static uint8_t ok_repeat_active = 0; /* 正在等待主机响应并重复发送 OK */
static uint32_t last_ok_ms = 0; /* 上次发送 OK 的时间（ms） */
static const uint32_t OK_REPEAT_INTERVAL_MS = 1000; /* 重发间隔：1000 ms（1 s） */

/* [新增] 回零完成后是否回中（M80/累计满N孔触发的回零用；自检/标定的回零有各自的回中流程） */
static volatile uint8_t homing_recenter = 0;
/* [新增] 累计打孔数阈值回零：当前行 OK 延迟到 homing 完成后再回（避免 homing 期间上位机发下一行打断回零） */
static volatile uint8_t pending_homing_ok = 0;

/* ---------------- helper functions ---------------- */

/* Helper: 注册/初始化保持原有接口 */
void MCodeSimple_Init(mcb_move_t cbM, mcb_feed_t cbY, mcb_punch_t cbP,
		mcb_s_t cbS, mcb_evt_t cb_beg, mcb_evt_t cb_end) {
	cb_move = cbM;
	cb_feed = cbY;
	cb_punch = cbP;
	cb_s = cbS;
	cb_batch_start = cb_beg;
	cb_batch_end = cb_end;
}

void MCodeSimple_RegisterSendOK(send_ok_t send_ok_cb) {
	cb_send_ok = send_ok_cb;
}

/* 新的解析函数 - 直接解析字母和后面的数字，不处理空格干扰 */
static void parse_statement(const char *start, const char *end,
		parsed_cmd_t *cmd) {
	/* 清零结构体，且把 S_val_present 明确初始化为 0（表示没有 '=value'） */
	memset(cmd, 0, sizeof(parsed_cmd_t));
	cmd->S_val_present = 0; /* 重要：0 = 没有 '=value'，1 = 有 '=value' */

	const char *p = start;

	while (p < end) {
		/* 跳过空白 */
		while (p < end && isspace((unsigned char )*p))
			p++;
		if (p >= end)
			break;

		/* 检查字母 */
		if (isalpha((unsigned char )*p)) {
			char letter = toupper((unsigned char )*p);
			p++; /* 跳过字母 */

			/* 跳过可能的空格 */
			while (p < end && isspace((unsigned char )*p))
				p++;

			/* 解析数字（可带 +/-） */
			int sign = 1;
			if (p < end && *p == '-') {
				sign = -1;
				p++;
			} else if (p < end && *p == '+') {
				p++;
			}

			int num = 0;
			int has_digit = 0;
			while (p < end && isdigit((unsigned char )*p)) {
				num = num * 10 + (*p - '0');
				p++;
				has_digit = 1;
			}

			if (!has_digit) {
				/* 没有数字的话跳过这个字母（比如单独字母的情况） */
				continue;
			}

			num = num * sign;

			/* 设置对应的命令 */
			switch (letter) {
			case 'M':
				cmd->hasM = 1;
				cmd->M = num;
				break;

			case 'Y':
				cmd->hasY = 1;
				cmd->Y = num;
				break;

			case 'P':
				cmd->hasP = 1;
				cmd->P = num;
				break;

			case 'S':
				cmd->hasS = 1;
				cmd->S_id = num;

				/* 检查是否有 '=value'（允许空格） */
				while (p < end && isspace((unsigned char )*p))
					p++;

				if (p < end && *p == '=') {
					p++; /* 跳过 '=' */
					while (p < end && isspace((unsigned char )*p))
						p++;

					int sval_sign = 1;
					if (p < end && *p == '-') {
						sval_sign = -1;
						p++;
					} else if (p < end && *p == '+') {
						p++;
					}

					int sval = 0;
					int has_sval_digit = 0;
					while (p < end && isdigit((unsigned char )*p)) {
						sval = sval * 10 + (*p - '0');
						p++;
						has_sval_digit = 1;
					}

					if (has_sval_digit) {
						cmd->S_val_present = 1;
						cmd->S_val = sval * sval_sign;
					} else {
						/* 遇到 S= 但没数字，视为没有值（保持 S_val_present = 0） */
					}
				} else {
					/* 没有 '=' 就保持 S_val_present == 0（只代表设置 id，不带 value） */
				}
				break;
			default:
				/* 未知字母忽略 */
				break;
			}
		} else {
			p++;
		}
	}
}

/* 判断 parsed_cmd_t 是否包含“真正的动作”：
 - M：只有 M != 90 && M != 80 才算
 - Y：只有 Y != 0 才算
 - P：只有 P != 0 才算
 S 不计入动作（只是参数/设置） */
static int pending_has_action(const parsed_cmd_t *c) {
	if (!c)
		return 0;
	if (c->hasM && (c->M != 90) && (c->M != 80))
		return 1;
	if (c->hasY && c->Y != 0)
		return 1;
	if (c->hasP && c->P != 0)
		return 1;
	return 0;
}

/* [新增] M 命令编号 → 音数 + 音序号（1-based）
   1~30 → 30音(音序号1~30)；101~115 → 15音；201~220 → 20音；
   401~440 → 40音；501~550 → 50音
   返回音序号，*count 输出音数，无效返回 0 */
static uint16_t mcode_map_note(int m, uint16_t *count) {
	if (m >= 1 && m <= 30)
		{ *count = 30; return (uint16_t) m; }
	if (m >= 101 && m <= 115)
		{ *count = 15; return (uint16_t) (m - 100); }
	if (m >= 201 && m <= 220)
		{ *count = 20; return (uint16_t) (m - 200); }
	if (m >= 401 && m <= 440)
		{ *count = 40; return (uint16_t) (m - 400); }
	if (m >= 501 && m <= 550)
		{ *count = 50; return (uint16_t) (m - 500); }
	*count = 0;
	return 0;
}

/* send OK：优先 cb_send_ok（fileio 注册：置 fileio_line_done + 置 deferred_ok 由主循环发 OK），否则兜底直接发 */
static void sendOK_internal(void) {
	if (cb_send_ok) {
		cb_send_ok();
	} else {
		USART_printf(&huart1, "OK\n");
	}
}

/* ---------------- main API: ProcessBuffer / Service / Notify ---------------- */

/* ProcessBuffer: 解析 buf（可能含多行），只保留最后一条有效命令作为 pending；
 并处理 M90/M80 的立即或延后行为（详见文件头说明）
 注意：每次收到主机新数据（ProcessBuffer 被调用）时都会停止 OK 重发（认为主机已活跃） */
void MCodeSimple_ProcessBuffer(const char *buf) {
	if (!buf)
		return;

	/* 主机活跃 —— 停止任何正在进行的 OK 重发 */
	ok_repeat_active = 0;
	last_ok_ms = 0;

	const char *p = buf;
	const char *line_start = p;
	parsed_cmd_t last_valid;
	uint8_t have_last = 0;

	/* 在解析过程中，pending_end_on_complete 可能被设置（M80 + 其它字段） */
	/* 但最终是否保留该标志，取决于是否把命令变成 pending（即是否有真实动作） */

	while (*p) {
		if (*p == '\r' || *p == '\n') {
			const char *line_end = p;
			const char *q = line_start;
			while (q < line_end && isspace((unsigned char )*q))
				q++;
			if (q < line_end) {
				parse_statement(line_start, line_end, &last_valid);

				int line_has_extra = (last_valid.hasY || last_valid.hasP
						|| last_valid.hasS
						|| (last_valid.hasM
								&& (last_valid.M != 90 && last_valid.M != 80)));
				if (last_valid.hasM && last_valid.M == 90) {
					if (line_has_extra) {
						/* 含 M90 且同一行有其它字段：立即进入 mcode 并作为 pending */
						in_mcode = 1;
						if (cb_batch_start)
							cb_batch_start();
						have_last = 1;
					} else {
						/* 纯 M90：立即进入 mcode（不作为 pending） */
						in_mcode = 1;
						if (cb_batch_start)
							cb_batch_start();
						if (cb_send_ok)
							cb_send_ok(); /* [修复] FileIO 模式置 fileio_line_done，否则离线执行卡死 */
					}
				} else if (last_valid.hasM && last_valid.M == 80) {
					if (line_has_extra) {
						/* 含 M80 且同一行有其它字段：把退出延后到 pending 完成 */
						pending_end_on_complete = 1;
						have_last = 1;
					} else {
						/* 纯 M80：立即退出 mcode */
						in_mcode = 0;
						if (cb_batch_end)
							cb_batch_end();
						if (cb_send_ok)
							cb_send_ok(); /* [修复] FileIO 模式置 fileio_line_done，否则离线执行卡死 */
					}
				} else {
					/* 普通行当作 pending */
					have_last = 1;
				}
			}
			while (*p == '\r' || *p == '\n')
				p++;
			line_start = p;
		} else {
			p++;
		}
	}

	/* 处理最后一行（可能没有换行结尾） */
	if (p > line_start) {
		parse_statement(line_start, p, &last_valid);
		int line_has_extra = (last_valid.hasY || last_valid.hasP
				|| last_valid.hasS
				|| (last_valid.hasM
						&& (last_valid.M != 90 && last_valid.M != 80)));
		if (last_valid.hasM && last_valid.M == 90) {
			if (line_has_extra) {
				in_mcode = 1;
				if (cb_batch_start)
					cb_batch_start();
				have_last = 1;
			} else {
				in_mcode = 1;
				if (cb_batch_start)
					cb_batch_start();
				if (cb_send_ok)
					cb_send_ok(); /* [修复] FileIO 模式置 fileio_line_done */
			}
		} else if (last_valid.hasM && last_valid.M == 80) {
			if (line_has_extra) {
				pending_end_on_complete = 1;
				have_last = 1;
			} else {
				in_mcode = 0;
				if (cb_batch_end)
					cb_batch_end();
				if (cb_send_ok)
					cb_send_ok(); /* [修复] FileIO 模式置 fileio_line_done */
			}
		} else {
			have_last = 1;
		}
	}

	/* 覆盖 pending（如果有） */
	if (have_last) {

		/* === 无任何真实动作（S-only 或仅无动作字段）时立即应用 S，但不回 OK === */
		if (!pending_has_action(&last_valid)) {
			/* 只要包含 S 就同步应用（保证参数立即生效），但**不发送 OK** */
			if (last_valid.hasS && cb_s) {
				if (last_valid.S_val_present) {
					/* 带 =value */
					cb_s(last_valid.S_id, 1, last_valid.S_val);
				} else {
					/* 只有 S id，没有 value */
					cb_s(last_valid.S_id, 0, -1);
				}
			}

			/* [修复] M80 + 无动作字段（M80 S0 / M80 Y0 / M80 P0）：也要退出 mcode，
			   否则批模式不退、后续命令仍按批模式回 OK，协议状态错乱 */
			uint8_t is_m80 = last_valid.hasM && (last_valid.M == 80);
			if (is_m80) {
				in_mcode = 0;
				if (cb_batch_end)
					cb_batch_end();
			}

			if (in_mcode || is_m80) {
				sendOK_internal(); // 立刻回应（mcode 中或 M80 退出都回 OK，与纯 M80 行为一致）
			}

			/* 不进入 pending，不影响状态机（注意清理 M80 延后退出的标志以免误作用） */
			pending_end_on_complete = 0;
			pending_send_ok = 0;
			pending_no_ok_on_complete = 0;
			return;
		}

		/* === 有真实动作，才进入 pending === */
		memcpy(&pending_cmd, &last_valid, sizeof(parsed_cmd_t));
		have_pending = 1;
		startedM = startedY = startedP = appliedS = 0;

		/* [修复] 非 M80 的新 pending：清 pending_end_on_complete，防止上一轮「M80 组合行未完成」
		   残留的粘性标志泄漏到新命令，导致新 pending 完成时错误退出 mcode。
		   只在「建立新 pending」时清，不影响「旧 pending 还在跑、S-only 命令不覆盖」的情况 */
		if (!(pending_cmd.hasM && pending_cmd.M == 80)) {
			pending_end_on_complete = 0;
		}

		/* 如果这个 pending 来源于 earlier 设置的 pending_end_on_complete（M80+其他字段），
		 则标记该 pending 在完成时**不要**发送 OK（但仍退出 mcode）。 */
		pending_no_ok_on_complete =
				(pending_end_on_complete && pending_cmd.hasM && pending_cmd.M == 80) ? 1 : 0;

		/* 只有在 mcode 中 + 有动作，完成后才 OK（但后续还会检查 pending_no_ok_on_complete 抑制） */
		pending_send_ok = in_mcode ? 1 : 0;
	}
}

/* Service: 在主循环或允许调用回调的上下文频繁调用，推进状态机 */
void MCodeSimple_Service(void) {
	/* [新增] 出厂标定 + 开机自检序列推进（在 homing_request 执行点之前） */
	FactoryCalib_Task();
	SelfTest_Task();

	/* [修改] 回零请求执行点：所有动作空闲、无待执行命令时，才真正回零（避免打断正在进行的动作） */
	if (homing_request && !Homing_IsActive() && move_busy == 0 && feed_busy == 0
			&& punch_busy == 0 && !have_pending && !handle_enable) {
		homing_request = 0;
		homing_recenter = homing_no_recenter ? 0 : 1; /* [修改] 打孔中途阈值回零不回中(回待打位置)，其他场景(M80/标定)回中 */
		homing_no_recenter = 0; /* 消费标志 */
		Home_Start();
	}

	/* [修复] 回零失败也清标志，避免残留导致下次成功误触发回中 */
	if (homing_recenter && Home_IsFailed()) {
		homing_recenter = 0;
	}

	/* [修改] 累计打孔数阈值回零：homing 失败也回延迟的 OK，避免上位机卡住等 ACK */
	if (pending_homing_ok && Home_IsFailed()) {
		pending_homing_ok = 0;
		sendOK_internal();
		if (in_mcode) {
			ok_repeat_active = 1;
			last_ok_ms = HAL_GetTick();
		}
	}

	/* [新增] 回零完成后回中：homing_recenter 置位时，Home_IsDone() 后移动到 software_max/2 工作区中间 */
	if (homing_recenter && Home_IsDone()) {
		homing_recenter = 0;
		float mid_mm = (float) current_config.software_max / 100.0f / 2.0f;
		int32_t mid_pulses = Notes_ConvertMMToPulses(mid_mm);
		move_target_position = -mid_pulses;
		move_busy = 1;
	}

	/* [修改] 累计打孔数阈值回零：homing 完成后回延迟的 OK（当前行真正完成，上位机再发下一行） */
	if (pending_homing_ok && Home_IsDone()) {
		pending_homing_ok = 0;
		sendOK_internal();
		if (in_mcode) {
			ok_repeat_active = 1;
			last_ok_ms = HAL_GetTick();
		}
	}

	/* 先处理 OK 重发逻辑（如果激活并仍在 mcode） */
	if (ok_repeat_active) {
		/* 如果已经离开 mcode，则停止重发 */
		if (!in_mcode) {
			ok_repeat_active = 0;
		} else {
			/* 达到间隔则重发 OK */
			uint32_t now = HAL_GetTick();
			if ((now - last_ok_ms) >= OK_REPEAT_INTERVAL_MS) {
				sendOK_internal();
				last_ok_ms = now;
			}
		}
	}

	if (!have_pending)
		return;

	/* [修改] homing / 打孔 / 标定进行中时，命令排队等动作完成（不启动动作，避免打断）；
	   但急停 S0 例外：立即生效，可随时打断 */
	if ((Homing_IsActive() || punch_busy || Calibration_IsActive()
			|| SelfTest_IsActive() || FactoryCalib_IsActive())
			&& !(pending_cmd.hasS && pending_cmd.S_id == 0)) {
		return;
	}

	/* 1) 应用 S（立即应用，不计为异步动作） */
	if (pending_cmd.hasS && !appliedS) {
		if (cb_s) {
			if (pending_cmd.S_val_present)
				cb_s(pending_cmd.S_id, 1, pending_cmd.S_val);
			else
				cb_s(pending_cmd.S_id, 0, -1);
		}
		appliedS = 1;
		/* [修复] S0 急停会调 MCodeSimple_Abort 清 have_pending，但不清 pending_cmd；
		   若不在此返回，step2/3/4 会因 startedM/Y/P 已被清 0 而重新启动 M/Y/P，急停被反悔 */
		if (!have_pending)
			return;
	}

	/* 2) 启动 M（只有当 M 非 90/80 且 cb_move 存在时才真正发起异步动作） */
	if (pending_cmd.hasM && !startedM) {
		if ((pending_cmd.M != 90) && (pending_cmd.M != 80) && cb_move) {
			uint16_t cnt, note_idx = mcode_map_note(pending_cmd.M, &cnt);
			if (note_idx >= 1) {
				/* [新增] 音数自动切换：M101→15音 / M201→20音 / M401→40音 / M501→50音 / M1~M30→30音 */
				if (cnt != current_config.note_count) {
					current_config.note_count = cnt;
					save_config_request = 1; /* 掉电保存自动切换的音数 */
					System_ApplyNoteConfig();
					/* [修复] 音数切换是低频事件（每 bank 边界一次），此处同步重算定位表，
					   否则 cb_move 立刻用旧表/零值表（切更大 bank 时 notes_pos[] 未初始化=0，电机冲向 home） */
					notes_recalc_request = 0;
					Notes_Init((float) current_config.home_bias / 100.0f);
				}
				move_busy = 1;
				cb_move(note_idx);
			}
		}
		/* 无论是否真启动，都把 startedM 置 1，空操作视为已完成 */
		startedM = 1;
	}

	/* 3) 启动 Y（只有当 Y 非 0 且 cb_feed 存在时才真正发起异步动作） */
	if (pending_cmd.hasY && !startedY) {
		if (pending_cmd.Y != 0 && cb_feed) {
			feed_busy = 1;
			cb_feed(pending_cmd.Y);
		}
		startedY = 1;
	}

	/* 4) 判断是否可以启动 P：需要 M 与 Y 完成（或不存在） */
	uint8_t m_done = (!pending_cmd.hasM)
			|| (pending_cmd.hasM && startedM && move_busy == 0);
	uint8_t y_done = (!pending_cmd.hasY)
			|| (pending_cmd.hasY && startedY && feed_busy == 0);
	if (m_done && y_done) {
		if (pending_cmd.hasP && !startedP) {
			if (pending_cmd.P != 0 && cb_punch) {
				punch_busy = 1;
				cb_punch(pending_cmd.P);
			}
			/* 无论是否真启动，都把 startedP 置 1，防止阻塞 */
			startedP = 1;
		}
	}

	/* 5) 完成判定： */
	if (pending_cmd.hasP && pending_cmd.P != 0) {
		if (startedP && punch_busy == 0) {
			/* [修复] 解耦「置 fileio_line_done」与「回 OK」：M80 组合行（pending_no_ok_on_complete）
			   只置 line_done（FileIO 推进）不回 OK，普通命令才 sendOK 回 ACK */
			if (pending_send_ok) {
				if (pending_no_ok_on_complete) {
					if (cb_send_ok)
						FileIO_NotifyLineDoneOnly();
				} else if (homing_no_recenter) {
					/* [修改] 累计打孔数阈值回零：OK 延迟到 homing 完成后再回 */
					pending_homing_ok = 1;
				} else {
					sendOK_internal();
					/* 如果仍在 mcode，启用 OK 重发，直到主机下一次发数据 */
					if (in_mcode) {
						ok_repeat_active = 1;
						last_ok_ms = HAL_GetTick();
					}
				}
			}
			if (pending_end_on_complete) {
				in_mcode = 0;
				if (cb_batch_end)
					cb_batch_end();
				/* 退出 mcode 后关闭 OK 重发 */
				ok_repeat_active = 0;
			}
			/* 清理 pending */
			have_pending = 0;
			pending_end_on_complete = 0;
			pending_send_ok = 0;
			pending_no_ok_on_complete = 0;
		}
	} else {
		uint8_t m_done2 = (!pending_cmd.hasM)
				|| (pending_cmd.hasM && startedM && move_busy == 0);
		uint8_t y_done2 = (!pending_cmd.hasY)
				|| (pending_cmd.hasY && startedY && feed_busy == 0);
		if (m_done2 && y_done2) {
			/* [修复] 解耦「置 fileio_line_done」与「回 OK」：M80 组合行只置 line_done 不回 OK */
			if (pending_send_ok) {
				if (pending_no_ok_on_complete) {
					if (cb_send_ok)
						FileIO_NotifyLineDoneOnly();
				} else if (homing_no_recenter) {
					/* [修改] 累计打孔数阈值回零：OK 延迟到 homing 完成后再回 */
					pending_homing_ok = 1;
				} else {
					sendOK_internal();
					if (in_mcode) {
						ok_repeat_active = 1;
						last_ok_ms = HAL_GetTick();
					}
				}
			}
			if (pending_end_on_complete) {
				in_mcode = 0;
				if (cb_batch_end)
					cb_batch_end();
				ok_repeat_active = 0;
			}
			have_pending = 0;
			pending_end_on_complete = 0;
			pending_send_ok = 0;
			pending_no_ok_on_complete = 0;
		}
	}
}

/* Notify 函数：动作层在动作完成时必须调用它们（可在 ISR 或主循环） */
void MCodeSimple_NotifyMoveDone(void) {
	move_busy = 0;
}
void MCodeSimple_NotifyFeedDone(void) {
	feed_busy = 0;
}
void MCodeSimple_NotifyPunchDone(void) {
	punch_busy = 0;
}

/* [新增] 紧急停止：清 pending 与 busy 状态，命令状态机复位 */
void MCodeSimple_Abort(void) {
	have_pending = 0;
	startedM = 0;
	startedY = 0;
	startedP = 0;
	appliedS = 0;
	move_busy = 0;
	feed_busy = 0;
	punch_busy = 0;
	pending_end_on_complete = 0;
	pending_send_ok = 0;
	pending_no_ok_on_complete = 0;
	ok_repeat_active = 0;
	in_mcode = 0; /* [修复] 急停同时退出 mcode 批模式，否则残留 in_mcode=1 导致后续命令仍按批模式回 OK/重发 */
	homing_recenter = 0; /* [修复] 急停清回中标志，否则残留导致下次任意回零成功后意外回中 */
}

/* [新增] 置 mcode 批模式标志：断电恢复从断点继续（跳过 M90 行）时由 FileIO_Resume 调用，
   否则 in_mcode 保持 0 → pending_send_ok=0 → 动作完成不发 OK → fileio_line_done 不置位 → 卡死 */
void MCodeSimple_SetInMcode(uint8_t v) {
	in_mcode = v;
}

/* ============================================================
 * [新增] 出厂标定序列（首次烧录，calibrated=0 时执行）
 * homing → 标主轴死区（move_base）→ 标软件限位 → 回中 → 标进纸死区
 * 注：先标 move_base 再标 software_max，标软件限位时才能用真实死区起步
 * ============================================================ */
typedef enum {
	FC_IDLE = 0,
	FC_HOMING,
	FC_MOVE_BASE,
	FC_SW_MAX,
	FC_RECENTER1,
	FC_PAPER_BASE,
	FC_DONE,
} factory_calib_state_t;

static volatile factory_calib_state_t fc_state = FC_IDLE;
static uint32_t fc_wait_ms = 0; /* [新增] 等待 move 到位的超时计数 */

/* [新增] 等待 move 到位，带超时（5000ms）。返回 1=到位或超时（应推进状态），0=继续等 */
#define MOVE_WAIT_TIMEOUT_MS 5000
static uint8_t WaitMoveDone(uint32_t *wait_ms) {
	if (move_busy == 0) {
		*wait_ms = 0;
		return 1;
	}
	if (++(*wait_ms) >= MOVE_WAIT_TIMEOUT_MS) {
		*wait_ms = 0;
		/* [修复] 超时停电机 + 清 move_busy，否则闭环持续朝不可达目标驱动电机堵转，
		   且 move_busy 永久为 1 导致后续所有 M 命令卡死 */
		set_move_pwm_duty(0);
		MCodeSimple_NotifyMoveDone();
		return 1; /* 超时强制推进，避免 ±1 抖动卡死 */
	}
	return 0;
}

uint8_t FactoryCalib_IsActive(void) {
	return (fc_state != FC_IDLE && fc_state != FC_DONE);
}

void FactoryCalib_Start(void) {
	calib_is_factory = 1; /* [修复] 标记出厂序列标定，Calib_* 不置 calibrated，避免中途急停过早落盘 */
	deferred_print_text = L("[FactoryCalib] Start: homing...", "[FactoryCalib] Start: homing...");
	fc_state = FC_HOMING;
	Home_Start();
}

static void FactoryCalib_Recenter(void) {
	float mid_mm = (float) current_config.software_max / 100.0f / 2.0f;
	int32_t mid_pulses = Notes_ConvertMMToPulses(mid_mm);
	/* [修改] 工作区在 home 负方向（远离 home = 位置减小），回中目标取负 */
	move_target_position = -mid_pulses;
	move_busy = 1;
}

void FactoryCalib_Task(void) {
	switch (fc_state) {
	case FC_IDLE:
	case FC_DONE:
		return;

	case FC_HOMING:
		if (Home_IsDone()) {
			deferred_print_text = L("[FactoryCalib] Homing 完成，标定主轴基准...",
					"[FactoryCalib] Homing done, calibrating move base...");
			fc_state = FC_MOVE_BASE;
			Calibration_Start(CALIB_MOVE_BASE);
		} else if (Home_IsFailed()) {
			deferred_print_text = L("[FactoryCalib] Homing 失败，中止",
					"[FactoryCalib] Homing failed, abort");
			fc_state = FC_IDLE;
		}
		break;

	case FC_MOVE_BASE:
		if (!Calibration_IsActive()) {
			deferred_print_text = L("[FactoryCalib] 主轴基准完成，标定软件限位...",
					"[FactoryCalib] Move base done, calibrating software limit...");
			homing_request = 0;
			fc_state = FC_SW_MAX;
			Calibration_Start(CALIB_SW_MAX);
		}
		break;

	case FC_SW_MAX:
		if (!Calibration_IsActive()) {
			deferred_print_text = L("[FactoryCalib] 软件限位完成，回中...",
					"[FactoryCalib] Software limit done, recenter...");
			homing_request = 0;
			fc_state = FC_RECENTER1;
			FactoryCalib_Recenter();
		}
		break;

	case FC_RECENTER1:
		if (WaitMoveDone(&fc_wait_ms)) {
			deferred_print_text = L("[FactoryCalib] 回中完成，标定进纸基准...",
					"[FactoryCalib] Recenter done, calibrating paper base...");
			fc_state = FC_PAPER_BASE;
			Calibration_Start(CALIB_PAPER_BASE);
		}
		break;

	case FC_PAPER_BASE:
		if (!Calibration_IsActive()) {
			deferred_print_text = L("[FactoryCalib] 标定全部完成！",
					"[FactoryCalib] All calibration done!");
			fc_state = FC_DONE;
			/* [修复] 出厂序列全部步骤完成才置已标定，避免中途急停导致下次开机跳过剩余标定 */
			current_config.calibrated = 1;
			calib_is_factory = 0;
			save_config_request = 1;
		}
		break;
	}
}

void FactoryCalib_Stop(void) {
	fc_state = FC_IDLE;
	calib_is_factory = 0; /* [修复] 急停清出厂序列标志 */
}

/* ============================================================
 * [新增] 开机自检序列（每次开机执行）
 * homing → 到软件限位 → 回中
 * ============================================================ */
typedef enum {
	ST_IDLE = 0,
	ST_HOMING,
	ST_TO_LIMIT,
	ST_RECENTER,
	ST_DONE,
} selftest_state_t;

static volatile selftest_state_t st_state = ST_IDLE;
static uint32_t st_wait_ms = 0; /* [新增] 等待 move 到位的超时计数 */

uint8_t SelfTest_IsActive(void) {
	return (st_state != ST_IDLE && st_state != ST_DONE);
}

void SelfTest_Start(void) {
	deferred_print_text = L("[SelfTest] 开始：homing...", "[SelfTest] Start: homing...");
	st_state = ST_HOMING;
	Home_Start();
}

static void SelfTest_Recenter(void) {
	float mid_mm = (float) current_config.software_max / 100.0f / 2.0f;
	int32_t mid_pulses = Notes_ConvertMMToPulses(mid_mm);
	/* [修改] 工作区在 home 负方向，回中目标取负 */
	move_target_position = -mid_pulses;
	move_busy = 1;
}

void SelfTest_Task(void) {
	switch (st_state) {
	case ST_IDLE:
	case ST_DONE:
		return;

	case ST_HOMING:
		if (Home_IsDone()) {
			deferred_print_text = L("[SelfTest] Homing 完成，移动到限位...",
					"[SelfTest] Homing done, move to limit...");
			st_state = ST_TO_LIMIT;
			/* [修改] 软件限位在 home 负方向，目标取负 */
			move_target_position = -Notes_ConvertMMToPulses(
					(float) current_config.software_max / 100.0f);
			move_busy = 1;
		} else if (Home_IsFailed()) {
			deferred_print_text = L("[SelfTest] Homing 失败，中止", "[SelfTest] Homing failed, abort");
			st_state = ST_IDLE;
		}
		break;

	case ST_TO_LIMIT:
		if (WaitMoveDone(&st_wait_ms)) {
			deferred_print_text = L("[SelfTest] 到达限位，回中...", "[SelfTest] At limit, recenter...");
			st_state = ST_RECENTER;
			SelfTest_Recenter();
		}
		break;

	case ST_RECENTER:
		if (WaitMoveDone(&st_wait_ms)) {
			deferred_print_text = L("[SelfTest] 完成", "[SelfTest] Done");
			st_state = ST_DONE;
		}
		break;
	}
}

void SelfTest_Stop(void) {
	st_state = ST_IDLE;
}
