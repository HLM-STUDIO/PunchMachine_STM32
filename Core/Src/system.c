#include "system.h"
#include "usart.h"
#include "motor.h"
#include "mcode.h"
#include "notes.h"
#include "handle.h"
#include "homing.h" /* [新增] S0 急停需 Home_Stop / homing_request */
#include "autotune.h" /* [新增] S88 自整定触发 / S0 急停中止 */
#include "flash.h"
#include "storage.h" /* [新增] 双冗余存储 */
#include "fileio.h"  /* [新增] 离线文件交互 */
#include "w25q32.h"  /* [新增] W25Q32 */
#include <string.h>
#include <math.h>  /* [修复] S15 显示主轴位置用 roundf */

/* 全局系统配置 */
SystemConfig_t current_config;

/* [新增] 主循环保存配置请求标志：1=请求写 Flash（在 while(1) 主循环执行，避免 1ms 中断里擦写 Flash 阻塞控制） */
volatile uint8_t save_config_request = 0;
volatile uint8_t notes_recalc_request = 0; /* [新增] 音数/间距重算请求：中断只置标志，主循环执行 Notes_Init */
volatile uint8_t calib_is_factory = 0; /* [新增] 1=出厂序列标定，Calib_* 不置 calibrated；0=单步标定各自置位 */

/* [新增] S 命令长文本延迟打印请求（在主循环打印，避免 1ms 中断阻塞串口） */
volatile print_request_t print_request = PRINT_NONE;
volatile const char *deferred_print_text = NULL; /* [新增] 动态延迟打印文本 */

/* 默认系统配置（注意 magic 使用 0xA5A5，checksum 留 0 由计算得出） */
SystemConfig_t default_config = { .magic = 0xA5A5, .version = CONFIG_VERSION, .move_base =
		360, .paper_base = 360, .punch_delay = 50, .software_max = 7000,
		.handle_thres = 150, .handle_x_mid = 2048, .handle_y_mid = 2048,
		.home_bias = 5950, /* [修改] 59505→5950（59.50mm），与 homing.c 硬编码 Notes_Init(59.5f) 一致 */
		.punch_lock_move = 0, /* [修改] 打孔时锁定 move 编码器计数（默认关闭=闭环跑，实测闭环更稳；S7=1 才锁死） */
		.note_spacing = 2000, /* 30 音间距 2.000mm（单位 mm×1000） */
		.note_spacing_15 = 2000, /* [新增] 15 音间距 */
		.note_spacing_20 = 2000, /* [新增] 20 音间距 */
		.note_spacing_40 = 2000, /* [新增] 40 音间距 */
		.note_spacing_50 = 2000, /* [新增] 50 音间距 */
		.note_count = 30, /* [新增] 默认音数 30 */
		.calibrated = 0, /* [新增] 默认未标定：首次开机自动标定 */
		.language = 1, /* [新增] 默认中文 */
		.tick_mul = 311, .tick_mem = 1, .move_dead_band = 1, .calib_sw_margin = 200, .punch_total = 0,
		.move_total = 0, .checksum = 0, .home_per_points = 500, .encoder_filter = 15, ._pad2 = 0,
	.move_kp = 3000, .move_kd = 100, .move_spd_kp = 150, .move_spd_ki = 30,
	.paper_kp = 4000, .paper_kd = 150, .paper_spd_kp = 120, .paper_spd_ki = 20,
	.paper_dead_band = 2, .homing_coarse = 640, .homing_fine = 60, .homing_backoff = 140 };

typedef enum {
	CMD_OK = 0, CMD_ERR_PARAM, CMD_ERR_UNSUPPORTED, CMD_ERR_INTERNAL
} cmd_status_t;

/* [新增] 应用音数+间距配置到 notes 并重算定位点（音数/间距变化时调用） */
void System_ApplyNoteConfig(void) {
	float spacing;
	switch (current_config.note_count) {
	case 15:
		spacing = (float) current_config.note_spacing_15 / 1000.0f;
		break;
	case 20:
		spacing = (float) current_config.note_spacing_20 / 1000.0f;
		break;
	case 40:
		spacing = (float) current_config.note_spacing_40 / 1000.0f;
		break;
	case 50:
		spacing = (float) current_config.note_spacing_50 / 1000.0f;
		break;
	case 30:
	default:
		spacing = (float) current_config.note_spacing / 1000.0f;
		break;
	}
	Notes_SetCount(current_config.note_count);
	Notes_SetSpacing(spacing);
	/* [修复] 不在中断里直接 Notes_Init（软浮点 50 次 roundf 挤爆 1ms 周期），只置标志由主循环执行 */
	notes_recalc_request = 1;

	/* [新增] 行程检查：总跨度 (note_count-1)*spacing 必须 ≤ 可用行程 min(home_bias, software_max)，
	   否则靠边音轨被钳到 home 原点/软件限位 */
	float span_mm = (current_config.note_count > 0)
			? (float)(current_config.note_count - 1) * spacing : 0.0f;
	float avail_mm = (float) current_config.home_bias / 100.0f;
	float sw_mm = (float) current_config.software_max / 100.0f;
	if (avail_mm > sw_mm)
		avail_mm = sw_mm;
	if (span_mm > avail_mm) {
		print_request = PRINT_TRAVEL_WARN; /* 延迟到主循环打印 */
	}
}

/* [新增] 行程不足警告：主循环调用，重算当前音数间距对比可用行程 */
void PrintTravelWarning(void) {
	float sp;
	switch (current_config.note_count) {
	case 15: sp = (float) current_config.note_spacing_15 / 1000.0f; break;
	case 20: sp = (float) current_config.note_spacing_20 / 1000.0f; break;
	case 40: sp = (float) current_config.note_spacing_40 / 1000.0f; break;
	case 50: sp = (float) current_config.note_spacing_50 / 1000.0f; break;
	case 30:
	default: sp = (float) current_config.note_spacing / 1000.0f; break;
	}
	float span = (current_config.note_count > 0)
			? (float)(current_config.note_count - 1) * sp : 0.0f;
	float avail = (float) current_config.home_bias / 100.0f;
	float sw_mm = (float) current_config.software_max / 100.0f;
	if (avail > sw_mm)
		avail = sw_mm; /* [修复] 可用行程 = min(home_bias, software_max) */
	/* [修复] 不用 %f（newlib-nano 无 printf_float 会打印乱码），拆整数/小数用 %ld.%02ld */
	int32_t sp100 = (int32_t) roundf(sp * 100.0f);
	int32_t span100 = (int32_t) roundf(span * 100.0f);
	int32_t avail100 = (int32_t) roundf(avail * 100.0f);
	USART_printf(&huart1, "[WARN] %s: %u%s x %ld.%02ldmm = %ld.%02ldmm > %s %ld.%02ldmm, %s\n",
			L("行程不足", "Travel insufficient"),
			(unsigned int) current_config.note_count, L("音", " notes"),
			(long) (sp100 / 100), (long) (sp100 % 100),
			(long) (span100 / 100), (long) (span100 % 100),
			L("可用行程", "available"),
			(long) (avail100 / 100), (long) (avail100 % 100),
			L("请减小间距或增大行程限位", "reduce spacing or increase travel limit"));
}

/* [新增] PID 参数保存开关（S63）：0=试验只写 RAM（默认），1=掉电保存。开关本身不持久化，上电回 0 */
uint8_t pid_save_enable = 0;

/* [新增] PID 参数设置（S51-S59 共用）：应用控制器；pid_save_enable=1 时同步写 current_config。
   返回 1=已写 current_config 需保存（由 system_command 末尾统一 need_save 保存） */
static uint8_t set_pid_param(uint8_t which, uint16_t val) {
	Motor_SetPIDParam(which, val);
	if (!pid_save_enable)
		return 0;
	switch (which) {
	case PID_MOVE_KP:      current_config.move_kp = val; break;
	case PID_MOVE_KD:      current_config.move_kd = val; break;
	case PID_MOVE_SPD_KP:  current_config.move_spd_kp = val; break;
	case PID_MOVE_SPD_KI:  current_config.move_spd_ki = val; break;
	case PID_PAPER_KP:     current_config.paper_kp = val; break;
	case PID_PAPER_KD:     current_config.paper_kd = val; break;
	case PID_PAPER_SPD_KP: current_config.paper_spd_kp = val; break;
	case PID_PAPER_SPD_KI: current_config.paper_spd_ki = val; break;
	case PID_PAPER_DEAD_BAND: current_config.paper_dead_band = val; break;
	default: return 0;
	}
	return 1;
}

void system_command(int S_id, uint8_t has_val, int32_t val) {
	cmd_status_t status = CMD_OK;
	uint8_t need_save = 0;

	switch (S_id) {

	case 0: // Global stop (紧急停止)
		/* [新增] 紧急停止：停所有电机 + 停打孔 + 停回零 + 清命令状态 */
		set_move_pwm_duty(0);
		set_paper_pwm_duty(0);
		paper_active = 0;
		/* 同步目标=当前位置，闭环 err=0，电机彻底停住不继续驱动到原目标 */
		move_target_position = read_move_encoder_counter();
		paper_target_position = read_paper_encoder_counter();
		Punch_Stop();
		punch_state = PUNCH_WAIT_LEAVE;
		punch_done_cnt = 0;
		punch_target_cnt = 0;
		Home_Stop();
		homing_request = 0;
		SelfTest_Stop(); /* [新增] 停止自检序列 */
		FactoryCalib_Stop(); /* [新增] 停止出厂标定 */
		Calibration_Stop(); /* [新增] 停止标定 */
		Autotune_Abort(); /* [新增] 停止自整定 */
		MCodeSimple_Abort();
		FileIO_Stop(); /* [新增] 停止离线执行（清进度） */
		break;

	case 1: // Manual mode ，藏着S1=1，有种你能进来！
		set_handle_mode();
		if (has_val && val == 1) {
			TestFlashFunction(); // hidden factory test
		}
		break;

	case 2: // Exit manual mode
		reset_handle_mode();
		break;

	case 3: // Home bias (mm*100)
		/* [修改] 去掉"比第一个点位置还大"的错误拦截（原校验恒为真导致永远设置失败）；
		   只保留基本校验：home_bias 必须为正且不超过 uint16_t 范围 */
		if (!has_val || val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.home_bias = (uint16_t) val;
			need_save = 1;
			System_ApplyNoteConfig(); /* 用新的 home_bias 重算定位点 */
		}
		break;

	case 4: // Re-sample handle midpoint
		current_config.handle_x_mid = Simple_Filter_GetCh1();
		current_config.handle_y_mid = Simple_Filter_GetCh2();
		need_save = 1;
//		need_print = 1;
		break;

	case 5: // Software max (mm*100)
		if (!has_val || val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.software_max = (uint16_t) val;
			Notes_SetSwMaxMM100((uint16_t) val); /* [新增] 同步软件限位脉冲缓存 */
			need_save = 1;
		}
		break;
	case 6: //打多少孔自动homing一下
		if (!has_val || val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.home_per_points = (uint16_t) val;
			need_save = 1;
		}
		break;

	case 7: // [新增] 打孔时锁定 move 编码器计数开关（1=锁死避免震动毛刺，0=不锁）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.punch_lock_move = (uint16_t) (val ? 1 : 0);
			need_save = 1;
		}
		break;

	case 8: // [修改] 30 音间距（mm×1000，2000=2.000mm，1500=1.500mm）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_spacing = (uint16_t) val;
			need_save = 1;
			if (current_config.note_count == 30) {
				System_ApplyNoteConfig(); /* 当前 30 音，立即重算 */
			}
		}
		break;

	case 9: // [新增] 当前音数（15/20/30/40/50）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val != 15 && val != 20 && val != 30 && val != 40 && val != 50) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_count = (uint16_t) val;
			need_save = 1;
			System_ApplyNoteConfig(); /* 音数变了，立即重算 */
		}
		break;

	case 81: // [新增] 15 音间距（mm×1000）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_spacing_15 = (uint16_t) val;
			need_save = 1;
			if (current_config.note_count == 15) {
				System_ApplyNoteConfig();
			}
		}
		break;

	case 82: // [新增] 20 音间距（mm×1000）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_spacing_20 = (uint16_t) val;
			need_save = 1;
			if (current_config.note_count == 20) {
				System_ApplyNoteConfig();
			}
		}
		break;

	case 83: // [新增] 40 音间距（mm×1000）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_spacing_40 = (uint16_t) val;
			need_save = 1;
			if (current_config.note_count == 40) {
				System_ApplyNoteConfig();
			}
		}
		break;

	case 84: // [新增] 移动到位允许误差（脉冲，最小 1；设 0 会退化为 1，避免精确对齐导致极限环卡死）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val < 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.move_dead_band = (uint16_t) val;
			need_save = 1;
		}
		break;

	case 85: // [新增] 50 音间距（mm×1000）
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else if (val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.note_spacing_50 = (uint16_t) val;
			need_save = 1;
			if (current_config.note_count == 50) {
				System_ApplyNoteConfig();
			}
		}
		break;

	case 86: // [新增] 标定余量（mm×100，200=2.00mm，S86 设置）
		if (!has_val || val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.calib_sw_margin = (uint16_t) val;
			need_save = 1;
		}
		break;

	case 87: // [新增] 编码器数字滤波（0-15，S87 设置，默认 15；0=关滤波，15=最强滤波）
		if (!has_val || val < 0 || val > 15) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.encoder_filter = (uint16_t) val;
			need_save = 1;
			Encoder_SetFilter((uint16_t) val); /* 运行时应用（改 CCMR1 的 IC1F/IC2F，立即生效，不用重启） */
		}
		break;

	case 88: // [新增] 移动轴 PID 自整定（继电反馈法，自动测位置环 kp/kd 并应用）
		Autotune_Start();
		break;

	/* [隐藏] S51-S58：PID 参数（×1000 定点）S51 move_kp ... S58 paper_spd_ki，不进帮助 */
	case 51: case 52: case 53: case 54:
	case 55: case 56: case 57: case 58: {
		if (!has_val) { status = CMD_ERR_PARAM; break; }
		uint8_t which = (uint8_t)(S_id - 51);   /* S51→0=PID_MOVE_KP ... S58→7=PID_PAPER_SPD_KI */
		int32_t lo, hi;
		switch (which) {
		case PID_MOVE_KP: case PID_PAPER_KP: lo = 10; hi = 60000; break;
		case PID_MOVE_KD: case PID_PAPER_KD: lo = 0;   hi = 20000; break;
		case PID_MOVE_SPD_KP: case PID_PAPER_SPD_KP: lo = 10; hi = 10000; break;
		default: lo = 0; hi = 2000; break;   /* spd_ki：0 合法（纯 P） */
		}
		if (val < lo || val > hi) { status = CMD_ERR_PARAM; break; }
		need_save |= set_pid_param(which, (uint16_t) val);
		break;
	}

	/* [隐藏] S59：paper 到位容忍（脉冲），最小 1（0 物理不可达卡死） */
	case 59:
		if (!has_val || val < 1 || val > 100) { status = CMD_ERR_PARAM; break; }
		need_save |= set_pid_param(PID_PAPER_DEAD_BAND, (uint16_t) val);
		break;

	/* [隐藏] S60-S62：homing PWM 加值（coarse/fine/backoff），fine 勿<30 堵转 */
	case 60: case 61: case 62: {
		if (!has_val) { status = CMD_ERR_PARAM; break; }
		int32_t lo2 = (S_id == 60) ? 300 : (S_id == 61) ? 30 : 50;
		int32_t hi2 = (S_id == 60) ? 2000 : (S_id == 61) ? 500 : 1000;
		if (val < lo2 || val > hi2) { status = CMD_ERR_PARAM; break; }
		if (S_id == 60) current_config.homing_coarse = (uint16_t) val;
		else if (S_id == 61) current_config.homing_fine = (uint16_t) val;
		else current_config.homing_backoff = (uint16_t) val;
		if (pid_save_enable) need_save = 1;
		break;
	}

	/* [隐藏] S63：PID 保存开关（0=试验只RAM默认，1=掉电保存；开关本身不持久化） */
	case 63:
		pid_save_enable = (has_val && val != 0) ? 1 : 0;
		break;

	/* [隐藏] S64：一键重置 PID 参数到默认值（kp/kd/spd_kp/ki/dead_band + homing 三 margin） */
	case 64: {
		static const uint16_t pid_def[9] = {
			3000, 100, 150, 30,     /* move_kp/kd/spd_kp/spd_ki（×100/×100/×100/×1000） */
			4000, 150, 120, 20,     /* paper_kp/kd/spd_kp/spd_ki */
			2                       /* paper_dead_band */
		};
		for (uint8_t i = 0; i < 9; i++)
			need_save |= set_pid_param(i, pid_def[i]);
		current_config.homing_coarse = 640;
		current_config.homing_fine = 60;
		current_config.homing_backoff = 140;
		if (pid_save_enable)
			need_save = 1;
		break;
	}

	case 10: // Show config
		print_request = PRINT_CONFIG; /* [修改] 延迟到主循环打印，避免 1ms 中断阻塞 */
		break;

	case 15: // [修复] 显示主轴位置（帮助文档有列出但原未实现，落 default 回 ERR）
		{
			float mm = Notes_ConvertPulsesToMM(read_move_encoder_counter());
			int32_t mm100 = (int32_t) roundf(mm * 100.0f);
			int32_t ip = mm100 / 100, fp = mm100 % 100;
			if (fp < 0)
				fp = -fp;
			/* [修复] ip==0 且 mm100<0（如 -0.50mm）整数除法丢负号，补 '-' */
			if (ip == 0 && mm100 < 0)
				USART_printf(&huart1, "%s -%ld.%02ld mm\n", L("主轴位置", "Move position"),
						(long) ip, (long) fp);
			else
				USART_printf(&huart1, "%s %ld.%02ld mm\n", L("主轴位置", "Move position"),
						(long) ip, (long) fp);
		}
		break;

	case 16: // [修复] 复位显示（帮助文档有列出但原未实现）
		USART_printf(&huart1, "\n\n");
		break;

	case 11: // [新增] 启动软件自动标定（software_max 行程范围）
		if (Homing_IsActive() || punch_busy || SelfTest_IsActive()
				|| FactoryCalib_IsActive() || !current_config.calibrated) {
			status = CMD_ERR_PARAM; /* [修改] 加 move_base 已标定前置校验，否则软件限位可能误标成 0 */
		} else {
			Calibration_Start(CALIB_SW_MAX);
		}
		break;

	case 12: // [新增] 标定 move_base（move 死区）
		if (Homing_IsActive() || punch_busy || SelfTest_IsActive()
				|| FactoryCalib_IsActive()) {
			status = CMD_ERR_PARAM;
		} else {
			Calibration_Start(CALIB_MOVE_BASE);
		}
		break;

	case 13: // [新增] 标定 paper_base（paper 死区）
		if (Homing_IsActive() || punch_busy || SelfTest_IsActive()
				|| FactoryCalib_IsActive()) {
			status = CMD_ERR_PARAM;
		} else {
			Calibration_Start(CALIB_PAPER_BASE);
		}
		break;

	case 14: // Handle threshold
		if (!has_val || val < 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.handle_thres = (uint16_t) val;
			need_save = 1;
		}
		break;

	case 90: // Help
		print_request = PRINT_HELP; /* [修改] 延迟到主循环打印，避免 1ms 中断阻塞 */
		break;

	case 95: // Tick memory enable
		if (!has_val) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.tick_mem = (uint16_t) (val ? 1 : 0);
			need_save = 1;
		}
		break;

	case 96: // Tick multiplier
		if (!has_val || val <= 0 || val > 65535) {
			status = CMD_ERR_PARAM;
		} else {
			Paper_Set_RollerDiameter_mm(val);
			current_config.tick_mul = (uint16_t) val;
			need_save = 1;
		}
		break;

	case 1000: // Punch total
		if (!has_val || val < 0) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.punch_total = (uint32_t) val;
			need_save = 1;
		}
		break;

	case 1001: // Move total
		if (!has_val || val < 0) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.move_total = (uint32_t) val;
			need_save = 1;
		}
		break;

	case 30: // [新增] 存储器读写自测（延迟到主循环执行，避免 1ms 中断里擦写 Flash 阻塞）
		print_request = PRINT_W25TEST;
		break;

	case 31: // [新增] 存储器全片擦除（延迟到主循环，擦除约 40s，会清空配置/离线文件）
		print_request = PRINT_W25ERASE;
		break;

	case 17: // [新增] 离线执行文件（S17）
		if (FileIO_Run() != 0) {
			status = CMD_ERR_UNSUPPORTED;
			if (!w25q32_present)
				USART_printf(&huart1, "%s\n", L("未检测到存储器", "Memory not detected"));
			else if (w25q_capacity < 0x400000UL)
				USART_printf(&huart1, "%s\n", L("存储器容量不足(需≥4MB)", "Memory capacity too small (need >=4MB)"));
		}
		break;

	case 18: // [新增] 离线暂停（S18，行级暂停）
		FileIO_Pause();
		break;

	case 19: // [新增] 离线继续（S19，从暂停/断电断点继续）
		if (FileIO_Resume() != 0) {
			status = CMD_ERR_UNSUPPORTED;
			if (!w25q32_present)
				USART_printf(&huart1, "%s\n", L("未检测到存储器", "Memory not detected"));
			else if (w25q_capacity < 0x400000UL)
				USART_printf(&huart1, "%s\n", L("存储器容量不足(需≥4MB)", "Memory capacity too small (need >=4MB)"));
		}
		break;

	case 20: // [新增] 离线回读预览（S20，分包回传）
		if (FileIO_Preview() != 0) {
			status = CMD_ERR_UNSUPPORTED;
			if (!w25q32_present)
				USART_printf(&huart1, "%s\n", L("未检测到存储器", "Memory not detected"));
			else if (w25q_capacity < 0x400000UL)
				USART_printf(&huart1, "%s\n", L("存储器容量不足(需≥4MB)", "Memory capacity too small (need >=4MB)"));
		}
		break;

	case 21: // [新增] 切换语言（S21=0 英文 / 1 中文，持久化）
		if (!has_val || (val != 0 && val != 1)) {
			status = CMD_ERR_PARAM;
		} else {
			current_config.language = (uint16_t) val;
			need_save = 1;
			/* 立即用新语言确认（language 已更新，L 宏即时生效） */
			USART_printf(&huart1, "%s\n", L("语言已切换为中文", "Language set to English"));
		}
		break;

	default:
		status = CMD_ERR_UNSUPPORTED;
		break;
	}

	/* ====== Unified post-processing ====== */

	if (status == CMD_OK) {

		if (need_save) {
			/* [修改] 不在这里写 Flash：system_command 可能被 MCodeSimple_Service 在 1ms 中断里调用，
			   直接擦写 Flash 会阻塞控制中断几十 ms；改置标志由主循环统一写 */
			save_config_request = 1;
		}
	}

	/* ====== Unified response ====== */

	if (status == CMD_OK) {
		if (has_val) {
			USART_printf(&huart1, "S%d=%d:OK\n", S_id, val);
		} else {
			USART_printf(&huart1, "S%d:OK\n", S_id);
		}
	} else {
		USART_printf(&huart1, "S%d:ERR,%d\n", S_id, status);
	}
}

void showHelpInfo(void) {
	USART_printf(&huart1, "\n");
	USART_printf(&huart1, "========================================\n");
	USART_printf(&huart1, "      MusicBox Puncher  Console\n");
	USART_printf(&huart1, "      FW v%u | %s %u | %s %s\n",
			(unsigned int) current_config.version,
			L("音数", "Notes"), (unsigned int) current_config.note_count,
			L("已标定", "Calib"), current_config.calibrated ? L("是", "Yes") : L("否", "No"));
	USART_printf(&huart1, "========================================\n");

	USART_printf(&huart1, "\n%s\n", L("[基础]", "[Basic]"));
	USART_printf(&huart1, "  S0    %s\n", L("紧急停止", "Emergency stop"));
	USART_printf(&huart1, "  S1    %s\n", L("进入手动模式", "Enter manual mode"));
	USART_printf(&huart1, "  S2    %s\n", L("退出手动模式", "Exit manual mode"));
	USART_printf(&huart1, "  S21=0/1 %s / %s\n", "语言(0=英 1=中)", "Language (0=EN 1=CN)"); /* [修改] 语言移入基础并双语显示：任意语言下都能看懂怎么切换 */
	USART_printf(&huart1, "  S90   %s\n", L("显示帮助", "Show help"));

	USART_printf(&huart1, "\n%s\n", L("[参数]", "[Parameters]"));
	USART_printf(&huart1, "  S3=X  %s\n", L("原点偏移(mm*100)", "Origin offset (mm*100)"));
	USART_printf(&huart1, "  S4    %s\n", L("重采手柄中点", "Re-sample handle midpoint"));
	USART_printf(&huart1, "  S5=X  %s\n", L("软件行程限位(mm*100)", "Software travel limit (mm*100)"));
	USART_printf(&huart1, "  S6=X  %s\n", L("自动回零间隔(孔)", "Auto-homing interval (punches)"));
	USART_printf(&huart1, "  S7=X  %s\n", L("打孔锁定移动编码器(0/1)", "Lock move encoder while punching (0/1)"));
	USART_printf(&huart1, "  S8=X  %s\n", L("30音间距(mm*1000)", "30-note spacing (mm*1000)"));
	USART_printf(&huart1, "  S9=X  %s\n", L("音数(15/20/30/40/50)", "Note count (15/20/30/40/50)"));
	USART_printf(&huart1, "  S81=X %s\n", L("15音间距", "15-note spacing"));
	USART_printf(&huart1, "  S82=X %s\n", L("20音间距", "20-note spacing"));
	USART_printf(&huart1, "  S83=X %s\n", L("40音间距", "40-note spacing"));
	USART_printf(&huart1, "  S84=X %s\n", L("移动到位误差(脉冲)", "Move dead band (pulses)"));
	USART_printf(&huart1, "  S85=X %s\n", L("50音间距", "50-note spacing"));
	USART_printf(&huart1, "  S86=X %s\n", L("标定余量(mm*100)", "Calib margin (mm*100)"));
	USART_printf(&huart1, "  S87=X %s\n", L("编码器数字滤波(0-15)", "Encoder digital filter (0-15)"));
	USART_printf(&huart1, "  S14=X %s\n", L("手柄死区阈值", "Handle deadzone threshold"));
	USART_printf(&huart1, "  S95=X %s\n", L("进纸比记忆(0/1)", "Paper feed ratio memory (0/1)"));
	USART_printf(&huart1, "  S96=X %s\n", L("进纸比", "Paper feed ratio"));

	USART_printf(&huart1, "\n%s\n", L("[标定]", "[Calibration]"));
	USART_printf(&huart1, "  S11   %s\n", L("标定软件行程限位", "Calibrate software travel limit"));
	USART_printf(&huart1, "  S12   %s\n", L("标定主轴基准PWM", "Calibrate move base PWM"));
	USART_printf(&huart1, "  S13   %s\n", L("标定进纸基准PWM", "Calibrate paper base PWM"));
	USART_printf(&huart1, "  S30   %s\n", L("存储器读写自测", "Memory read/write self-test"));
	USART_printf(&huart1, "  S31   %s\n", L("存储器全片擦除", "Memory full chip erase"));

	USART_printf(&huart1, "\n%s\n", L("[离线]", "[Offline]"));
	USART_printf(&huart1, "  S17   %s\n", L("执行离线文件", "Run offline file"));
	USART_printf(&huart1, "  S18   %s\n", L("暂停", "Pause"));
	USART_printf(&huart1, "  S19   %s\n", L("继续", "Resume"));
	USART_printf(&huart1, "  S20   %s\n", L("回读预览", "Preview (readback)"));

	USART_printf(&huart1, "\n%s\n", L("[信息]", "[Info]"));
	USART_printf(&huart1, "  S10   %s\n", L("显示系统配置", "Show system config"));
	USART_printf(&huart1, "  S15   %s\n", L("显示主轴位置", "Show move position"));
	USART_printf(&huart1, "  S16   %s\n", L("复位显示", "Reset display"));
	USART_printf(&huart1, "  S1000=X %s\n", L("累计打孔数", "Total punch count"));
	USART_printf(&huart1, "  S1001=X %s\n", L("累计行程(mm*100)", "Total travel (mm*100)"));

	USART_printf(&huart1, "\n========================================\n\n");
}

/* 计算校验和（按 16-bit 单元异或，排除 checksum 字段） */
uint16_t CalculateChecksum(SystemConfig_t *config) {
	if (config == NULL)
		return 0;

	uint16_t *ptr = (uint16_t*) config;
	uint32_t words = sizeof(SystemConfig_t) / sizeof(uint16_t);
	/* 计算 checksum 字段在 16-bit 单元数组中的索引 */
	uint32_t checksum_index = offsetof(SystemConfig_t, checksum)
			/ sizeof(uint16_t);

	uint16_t sum = 0;
	for (uint32_t i = 0; i < words; i++) {
		if (i == checksum_index)
			continue; /* 明确跳过 checksum 字段本身 */
		sum ^= ptr[i];
	}
	return sum;
}

int ValidateConfig(SystemConfig_t *config) {
	if (config == NULL)
		return -1;

	if (config->magic != 0xA5A5)
		return -1;

	/* [修复] 版本校验：跨固件结构体变化时 version 应递增，旧版本结构体布局不兼容，
	   直接判无效回默认（原只查 magic+checksum，version 字段纯摆设） */
	if (config->version != CONFIG_VERSION) {
		return -3;
	}

	uint16_t calc = CalculateChecksum(config);
	if (config->checksum != calc) {
		USART_printf(&huart1, "%s: 0x%04X (%s 0x%04X)\n",
				L("校验和错误", "Checksum error"),
				(unsigned int) config->checksum,
				L("期望", "expected"),
				(unsigned int) calc);
		return -2;
	}
	return 0;
}

/* [新增] 配置迁移（版本分支链）：从旧版本逐步迁移到 CONFIG_VERSION。
   约定：新字段一律追加在结构体末尾（不改变已有字段偏移），字段复用位（padding→实际字段）填默认。
   读旧数据由 Storage_LoadCompat 负责（W25Q32 按旧 len+旧 CRC，内部 Flash 按 magic 弱校验），
   这里只按 version 分支给各版本新增字段填默认值。 */
int MigrateConfig(SystemConfig_t *cfg) {
	uint16_t from = cfg->version;

	/* v107 → v108：_pad2 复用为 calib_sw_margin（标定余量，默认 200=2.00mm） */
	if (from < 108) {
		cfg->calib_sw_margin = 200;
		from = 108;
	}

	/* v108 → v109：末尾追加 encoder_filter（编码器数字滤波，默认 15） */
	if (from < 109) {
		cfg->encoder_filter = 15;
		from = 109;
	}

	/* v109 → v110：末尾追加 12 个 PID 参数字段（S51-S62），默认值 */
	if (from < 110) {
		cfg->move_kp = 3000;
		cfg->move_kd = 100;
		cfg->move_spd_kp = 150;
		cfg->move_spd_ki = 30;
		cfg->paper_kp = 4000;
		cfg->paper_kd = 150;
		cfg->paper_spd_kp = 120;
		cfg->paper_spd_ki = 20;
		cfg->paper_dead_band = 2;
		cfg->homing_coarse = 640;
		cfg->homing_fine = 60;
		cfg->homing_backoff = 140;
		from = 110;
	}

	/* [修复] 关键字段范围兜底：跨更大版本迁移时，旧版本没有的字段可能是 0/垃圾，填默认 */
	if (cfg->note_count != 15 && cfg->note_count != 20 && cfg->note_count != 30
			&& cfg->note_count != 40 && cfg->note_count != 50)
		cfg->note_count = 30;
	if (cfg->calibrated > 1)
		cfg->calibrated = 0;
	if (cfg->note_spacing == 0)
		cfg->note_spacing = 2000;
	if (cfg->note_spacing_15 == 0)
		cfg->note_spacing_15 = 2000;
	if (cfg->note_spacing_20 == 0)
		cfg->note_spacing_20 = 2000;
	if (cfg->note_spacing_40 == 0)
		cfg->note_spacing_40 = 2000;
	if (cfg->note_spacing_50 == 0)
		cfg->note_spacing_50 = 2000;
	if (cfg->encoder_filter > 15)
		cfg->encoder_filter = 15;
	if (cfg->language > 1)
		cfg->language = 1; /* [修复] 语言 0=英 1=中，非法值兜回中文（0 合法保留） */
	if (cfg->home_per_points == 0)
		cfg->home_per_points = 500; /* [修复] 0 会每孔回零+每孔写 Flash，兜回默认 500 */
	if (cfg->punch_lock_move > 1)
		cfg->punch_lock_move = 1; /* [修复] 布尔字段兜底 */
	/* [新增] PID 参数字段兜底：kp 不允许 0（位置环失效卡死），homing_fine 勿<30（堵转），kd/ki 下限 0 合法 */
	if (cfg->move_kp < 10 || cfg->move_kp > 60000)
		cfg->move_kp = 3000;
	if (cfg->move_kd > 20000)
		cfg->move_kd = 100;
	if (cfg->move_spd_kp < 10 || cfg->move_spd_kp > 10000)
		cfg->move_spd_kp = 150;
	if (cfg->move_spd_ki > 2000)
		cfg->move_spd_ki = 30;
	if (cfg->paper_kp < 10 || cfg->paper_kp > 60000)
		cfg->paper_kp = 4000;
	if (cfg->paper_kd > 20000)
		cfg->paper_kd = 150;
	if (cfg->paper_spd_kp < 10 || cfg->paper_spd_kp > 10000)
		cfg->paper_spd_kp = 120;
	if (cfg->paper_spd_ki > 2000)
		cfg->paper_spd_ki = 20;
	if (cfg->paper_dead_band < 1 || cfg->paper_dead_band > 100)
		cfg->paper_dead_band = 2;
	if (cfg->homing_coarse < 300 || cfg->homing_coarse > 2000)
		cfg->homing_coarse = 640;
	if (cfg->homing_fine < 30 || cfg->homing_fine > 500)
		cfg->homing_fine = 60;
	if (cfg->homing_backoff < 50 || cfg->homing_backoff > 1000)
		cfg->homing_backoff = 140;

	cfg->version = CONFIG_VERSION;
	cfg->checksum = CalculateChecksum(cfg);
	if (SaveSystemConfig() != 0)
		return -1;
	return 0;
}

int LoadSystemConfig(void) {
	/* [修改] 双冗余读：W25Q32(权威) → 内部 Flash(回退) */
	int ret = Storage_Load(&current_config);
	if (ret != 0) {
		/* [新增] 版本不匹配时尝试迁移：保留旧字段、新字段填默认，避免升级后用户配置丢失 */
		if (Storage_LoadCompat(&current_config) == 0
				&& MigrateConfig(&current_config) == 0) {
			USART_printf(&huart1, "%s\n",
					L("配置已迁移到新版本", "Config migrated to new version"));
			return 0;
		}
		USART_printf(&huart1, "%s\n",
				L("配置无效或未初始化，使用默认配置", "Config invalid or uninitialized, using default configuration"));
		current_config = default_config;
		current_config.checksum = CalculateChecksum(&current_config);
		if (SaveSystemConfig() != 0) {
			USART_printf(&huart1, "%s\n", L("保存默认配置失败", "Failed to save default configuration."));
			return -1;
		}
		return -1;
	}
	USART_printf(&huart1, "%s\n", L("配置加载成功", "Successfully loaded configuration."));
	return 0;
}

int SaveSystemConfig(void) {
	current_config.checksum = CalculateChecksum(&current_config);

	/* [修改] 双冗余写：内部 Flash + W25Q32 同时写 */
	if (Storage_Save(&current_config) != 0) {
		USART_printf(&huart1, "%s\n", L("保存配置失败", "Failed to save configuration"));
		return -1;
	}
	USART_printf(&huart1, "%s\n", L("配置已保存", "Configuration saved"));
	return 0;
}

void ModifyConfigExample(void) {
	USART_printf(&huart1, "%s\n", L("修改配置示例...", "Modify configuration example..."));
	current_config.handle_thres = 200;
	if (SaveSystemConfig() == 0) {
		USART_printf(&huart1, "%s\n", L("配置修改成功", "Configuration modified successfully"));
		PrintSystemConfig();
	} else {
		USART_printf(&huart1, "%s\n", L("配置修改失败", "Configuration modification failed"));
	}
}

/**
 * @brief  Print system configuration (中英文切换，优化版本)
 */
void PrintSystemConfig(void) {
	USART_printf(&huart1, "%s\n", L("=== 系统配置 ===", "=== System Configuration ==="));

	USART_printf(&huart1, "%s: %u\n", L("主轴基准", "Move Base"),
			(unsigned int) current_config.move_base);

	USART_printf(&huart1, "%s: %u\n", L("进纸基准", "Paper Base"),
			(unsigned int) current_config.paper_base);

	USART_printf(&huart1, "%s: %u ms\n", L("打孔延时", "Punch Delay"),
			(unsigned int) current_config.punch_delay);

	/* software_max: uint16_t，单位 mm*100 */
	USART_printf(&huart1, "%s: %u.%02u mm\n", L("软件限位", "Software Max"),
			(unsigned int) (current_config.software_max / 100),
			(unsigned int) (current_config.software_max % 100));

	USART_printf(&huart1, "%s: %u\n", L("手柄阈值", "Handle Threshold"),
			(unsigned int) current_config.handle_thres);

	USART_printf(&huart1, "%s: %u\n", L("手柄X中点", "Handle X Midpoint"),
			(unsigned int) current_config.handle_x_mid);

	USART_printf(&huart1, "%s: %u\n", L("手柄Y中点", "Handle Y Midpoint"),
			(unsigned int) current_config.handle_y_mid);

	/* home_bias: uint16_t，单位 mm*100 */
	USART_printf(&huart1, "%s: %u.%02u mm\n", L("原点偏移", "Origin offset"),
			(unsigned int) (current_config.home_bias / 100),
			(unsigned int) (current_config.home_bias % 100));

	/* calib_sw_margin: uint16_t，单位 mm*100 */
	USART_printf(&huart1, "%s: %u.%02u mm\n", L("标定余量", "Calib margin"),
			(unsigned int) (current_config.calib_sw_margin / 100),
			(unsigned int) (current_config.calib_sw_margin % 100));

	USART_printf(&huart1, "%s: %u\n", L("打孔锁定移动", "Punch Lock Move"),
			(unsigned int) current_config.punch_lock_move);

	USART_printf(&huart1, "%s: %u\n", L("音数", "Note Count"),
			(unsigned int) current_config.note_count);

	USART_printf(&huart1, "%s: %s\n", L("已标定", "Calibrated"),
			current_config.calibrated ? L("是", "Yes") : L("否", "No"));

	USART_printf(&huart1, "%s: %u.%03u mm\n", L("30音间距", "Note Spacing 30"),
			(unsigned int) (current_config.note_spacing / 1000),
			(unsigned int) (current_config.note_spacing % 1000));
	USART_printf(&huart1, "%s: %u.%03u mm\n", L("15音间距", "Note Spacing 15"),
			(unsigned int) (current_config.note_spacing_15 / 1000),
			(unsigned int) (current_config.note_spacing_15 % 1000));
	USART_printf(&huart1, "%s: %u.%03u mm\n", L("20音间距", "Note Spacing 20"),
			(unsigned int) (current_config.note_spacing_20 / 1000),
			(unsigned int) (current_config.note_spacing_20 % 1000));
	USART_printf(&huart1, "%s: %u.%03u mm\n", L("40音间距", "Note Spacing 40"),
			(unsigned int) (current_config.note_spacing_40 / 1000),
			(unsigned int) (current_config.note_spacing_40 % 1000));
	USART_printf(&huart1, "%s: %u.%03u mm\n", L("50音间距", "Note Spacing 50"),
			(unsigned int) (current_config.note_spacing_50 / 1000),
			(unsigned int) (current_config.note_spacing_50 % 1000));

	USART_printf(&huart1, "%s: %u\n", L("进纸比倍率", "Feed ratio multiplier"),
			(unsigned int) current_config.tick_mul);

	USART_printf(&huart1, "%s: %u\n", L("进纸比记忆", "Feed ratio memory"),
			(unsigned int) current_config.tick_mem);

	USART_printf(&huart1, "%s: %u\n", L("移动到位误差", "Move dead band"),
			(unsigned int) current_config.move_dead_band);

	USART_printf(&huart1, "%s: %u\n", L("编码器数字滤波", "Encoder filter"),
			(unsigned int) current_config.encoder_filter);

	/* [隐藏] PID 参数（kp/kd/spd_kp ×100、spd_ki ×1000；dead_band/margin 原始值），仅 S10 打印给调试者看，不进帮助 */
	USART_printf(&huart1, "move_kp=%u.%02u move_kd=%u.%02u move_spd_kp=%u.%02u move_spd_ki=%u.%03u\n",
			(unsigned int)(current_config.move_kp / 100), (unsigned int)(current_config.move_kp % 100),
			(unsigned int)(current_config.move_kd / 100), (unsigned int)(current_config.move_kd % 100),
			(unsigned int)(current_config.move_spd_kp / 100), (unsigned int)(current_config.move_spd_kp % 100),
			(unsigned int)(current_config.move_spd_ki / 1000), (unsigned int)(current_config.move_spd_ki % 1000));
	USART_printf(&huart1, "paper_kp=%u.%02u paper_kd=%u.%02u paper_spd_kp=%u.%02u paper_spd_ki=%u.%03u dead_band=%u\n",
			(unsigned int)(current_config.paper_kp / 100), (unsigned int)(current_config.paper_kp % 100),
			(unsigned int)(current_config.paper_kd / 100), (unsigned int)(current_config.paper_kd % 100),
			(unsigned int)(current_config.paper_spd_kp / 100), (unsigned int)(current_config.paper_spd_kp % 100),
			(unsigned int)(current_config.paper_spd_ki / 1000), (unsigned int)(current_config.paper_spd_ki % 1000),
			(unsigned int) current_config.paper_dead_band);
	USART_printf(&huart1, "homing_coarse=%u homing_fine=%u homing_backoff=%u\n",
			(unsigned int) current_config.homing_coarse,
			(unsigned int) current_config.homing_fine,
			(unsigned int) current_config.homing_backoff);

	/* punch_total: uint32_t，必须用 %lu */
	USART_printf(&huart1, "%s: %lu\n", L("累计打孔数", "Punch Total Count"),
			(unsigned long) current_config.punch_total);

	/* move_total: uint32_t，单位 mm*100（两位小数） */
	USART_printf(&huart1, "%s: %lu.%02lu mm\n", L("累计主轴行程", "Move Total Distance"),
			(unsigned long) (current_config.move_total / 100),
			(unsigned long) (current_config.move_total % 100));

	/* version: uint16_t，按 Vx.xx 显示 */
	USART_printf(&huart1, "%s: V%u.%02u\n", L("版本", "Version"),
			(unsigned int) (current_config.version / 100),
			(unsigned int) (current_config.version % 100));

	USART_printf(&huart1, "© 2019-2027 by HLM, All Rights Reserved\n");
	USART_printf(&huart1, "===========================\n");
}

void TestFlashFunction(void) {
	USART_printf(&huart1, "%s\n", L("==== Flash测试开始 ====", "==== Flash test start ===="));
	SystemConfig_t backup;
	memcpy(&backup, &current_config, sizeof(SystemConfig_t));

	SystemConfig_t tmp_default = default_config;
	tmp_default.checksum = CalculateChecksum(&tmp_default);

	uint32_t bytes = sizeof(SystemConfig_t);
	uint32_t words32 = (bytes + 3U) / 4U;
	if (Flash_HAL_Write_N_Data(FLASH_USER_START_ADDR, (uint32_t*) &tmp_default,
			(uint16_t) words32) != 0) {
		USART_printf(&huart1, "%s\n",
				L("默认配置写入Flash失败", "Failed to write default configuration to Flash"));
		memcpy(&current_config, &backup, sizeof(SystemConfig_t));
		return;
	}
	USART_printf(&huart1, "%s\n", L("默认配置已写入Flash", "Default configuration written to Flash"));

	memset(&current_config, 0, sizeof(SystemConfig_t));
	Flash_HAL_Read_N_Data(FLASH_USER_START_ADDR, (uint32_t*) &current_config,
			words32);

	int vret = ValidateConfig(&current_config);
	if (vret != 0) {
		USART_printf(&huart1, "%s (err=%d)\n",
				L("从Flash读回校验失败", "Validation failed after reading from Flash"), vret);
	} else {
		if (memcmp(&current_config, &tmp_default, sizeof(SystemConfig_t))
				== 0) {
			USART_printf(&huart1, "%s\n",
					L("Flash读写自测通过：读回数据与写入一致",
						"Flash read/write self-test passed: read data matches written data"));
		} else {
			USART_printf(&huart1, "%s\n",
					L("Flash读写自测失败：读回数据与写入不一致",
						"Flash read/write self-test failed: read data does not match written data"));
			PrintSystemConfig();
		}
	}

	PrintSystemConfig();
	memcpy(&current_config, &backup, sizeof(SystemConfig_t));
	USART_printf(&huart1, "%s\n",
			L("Flash测试结束，已恢复备份配置", "Flash test finished, backup configuration restored"));
}

