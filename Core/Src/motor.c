/*
 * motor.c
 *
 *  Created on: 2026年1月22日
 *      Author: huang
 */
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "motor.h"
#include "tim.h"
#include "homing.h"
#include "handle.h"
#include "system.h"
#include "usart.h"
#include "notes.h" /* [新增] 标定 software_max 用 Notes_ConvertPulsesToMM */
#include "mcode.h" /* [修改] 引入 punch_busy 等命令状态标志，用于 Punch 互锁 */
#include "fileio.h" /* [新增] 打孔阈值回零抑制用 FileIO_IsRunning */
#include "autotune.h" /* [新增] 自整定接管（Autotune_Tick） */
/////////////////////////////////////////////////////////////////////////////////
/* 打孔相关全局（保持原样） */
uint8_t punch_target_cnt = 0;
uint8_t punch_done_cnt = 0;
uint16_t punch_low_ms = 0;
static uint16_t punch_stuck_ms = 0; /* [新增] 打孔状态机卡死超时计数 */
static int32_t move_enc_backup = 0; /* [新增] 打孔前 move counter 备份，打孔结束后恢复（丢弃打孔期毛刺） */
punch_state_t punch_state;

/* ============================================================
 * 全局变量（move / paper 分开）
 * ============================================================ */
/* move（主轴） */
volatile int32_t move_encoder_ext_count = 0;
volatile int32_t move_total_pulses = 0; /* [新增] 累计移动脉冲（|diff| 累加，手动+自动都算，主循环转 mm 累加到 move_total） */
volatile int32_t move_target_position = 32500; /* [修改] 加 volatile */

/* paper（进纸轴） */
volatile int32_t paper_encoder_ext_count = 0;
volatile int32_t paper_target_position = 0; /* [修改] 加 volatile */
volatile uint8_t paper_active = 0;
float paper_scale = 1.0f; /* Y 单位 -> 脉冲（可通过 Paper_Set_RollerDiameter_mm 设置） */

/* ============================================================
 * 控制器实例（move / paper）
 * ============================================================ */
/* move */
static pos_ctrl_t move_pos_ctrl = { .kp = 30.0f, .kd = 1.0f, .prev_err = 0.0f,
		.prev_deriv = 0.0f, .deriv_alpha = 0.85f, .vmax = PWM_MAX * 1.0f,
		.dead_band = POS_HOLD_ZONE };
static pi_ctrl_t move_spd_pi = { .kp = 1.5f, .ki = 0.03f, .integral = 0.0f,
		.out_max = PWM_MAX };

/* paper */
static pos_ctrl_t paper_pos_ctrl = { .kp = 40.0f, .kd = 1.5f, .prev_err = 0.0f,
		.prev_deriv = 0.0f, .deriv_alpha = 0.85f, .vmax = PWM_MAX * 1.0f,
		.dead_band = 2.0f }; /* [修改] 进纸到位容忍 ±2 脉冲，避免霍尔 ±1 抖动导致反复微调（move 保持精确对齐） */
static pi_ctrl_t paper_spd_pi = { .kp = 1.2f, .ki = 0.02f, .integral = 0.0f,
		.out_max = PWM_MAX };

/* ============================================================
 * PWM 输出（move / paper）
 * ============================================================ */
void set_move_pwm_duty(int32_t duty) {
	if (duty > PWM_MAX)
		duty = PWM_MAX;
	if (duty < -PWM_MAX)
		duty = -PWM_MAX;

	if (duty == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX);
		return;
	}
	if (duty > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX - duty);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX + duty);
	}
}

void set_paper_pwm_duty(int32_t duty) {
	if (duty > PWM_MAX)
		duty = PWM_MAX;
	if (duty < -PWM_MAX)
		duty = -PWM_MAX;

	if (duty == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX);
		return;
	}
	if (duty > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX - duty);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX + duty);
	}
}

/* ============================================================
 * 编码器扩展（move: TIM2, paper: TIM4）
 * 采用鲁棒差分，处理 16-bit 溢出/翻转
 * ============================================================ */

/* 文件级 last_cnt，用于溢出校正（便于在 reset 时同步） */
static uint16_t last_cnt_move = 0;
static uint16_t last_cnt_paper = 0;

void TIM2_UpdateEncoderExtension(void) {
	uint16_t cnt = TIM2->CNT & 0xFFFF;
	int32_t diff = (int32_t) cnt - (int32_t) last_cnt_move;
	if (diff > 32767)
		diff -= 65536;
	else if (diff < -32768)
		diff += 65536;
	last_cnt_move = cnt;
	move_encoder_ext_count += diff;
	/* [新增] 累计移动距离（|diff|，方向无关；手动+自动移动都算）。
	   打孔期间震动毛刺不计入（无论 punch_lock_move 是 0 还是 1，打孔都非有效行程，
	   原只判 punch_lock_move=1 导致默认 0 时打孔期闭环 hold 微动仍计入）。
	   homing 回零移动也不计入：homing 是每次开机必做的回零，非有效工作行程，
	   且超时粗靠（15s）会累计近 1m，导致累计行程虚增。
	   [修复] 自检/标定的「到限位 + 回中」也不计入：这些是诊断/标定行程，非有效工作行程，
	   否则每次开机自检虚增 2×software_max、每次标定再虚增标定行程。 */
	if (!punch_busy && !Homing_IsActive() && !SelfTest_IsActive() && !FactoryCalib_IsActive()) {
		if (diff < 0)
			diff = -diff;
		move_total_pulses += diff;
	}
}

void TIM4_UpdateEncoderExtension(void) {
	uint16_t cnt = TIM4->CNT & 0xFFFF;
	int32_t diff = (int32_t) cnt - (int32_t) last_cnt_paper;
	if (diff > 32767)
		diff -= 65536;
	else if (diff < -32768)
		diff += 65536;
	last_cnt_paper = cnt;
	paper_encoder_ext_count += diff;
}

/* 读取接口 */
int32_t read_move_encoder_counter(void) {
	return move_encoder_ext_count;
}
int32_t read_paper_encoder_counter(void) {
	return paper_encoder_ext_count;
}

/* [新增] 运行时设置编码器数字滤波（0-15，S87 命令）。
 * 直接改 TIM2/TIM4 的 CCMR1 输入捕获滤波位：IC1F[7:4]、IC2F[15:12]。
 * filter=0 关滤波；filter=15 最强（滤波窗口 ≈16/72MHz ≈ 222ns）。
 * 不改结构体、不重启，编码器接口硬件持续计数不受影响。 */
void Encoder_SetFilter(uint16_t filter) {
	if (filter > 15)
		filter = 15;
	uint32_t f = (uint32_t) filter;

	/* TIM2 移动编码器 */
	TIM2->CCMR1 = (TIM2->CCMR1 & ~((0xFUL << 4) | (0xFUL << 12)))
			| (f << 4) | (f << 12);
	/* TIM4 进纸编码器 */
	TIM4->CCMR1 = (TIM4->CCMR1 & ~((0xFUL << 4) | (0xFUL << 12)))
			| (f << 4) | (f << 12);
}

/* ============================================================
 * 速度测量（基于 CTRL_FREQ 调用）
 * ============================================================ */
/* move */
static int32_t move_enc_last = 0;
static float move_speed_meas = 0.0f;
static void Move_Encoder_Speed_Update(void) {
	int32_t enc_now = read_move_encoder_counter();
	int32_t diff = enc_now - move_enc_last;
	move_enc_last = enc_now;
	/* [修改] 速度测量加一阶低通，抑制光栅抖动噪声进入速度环 */
	move_speed_meas = 0.5f * move_speed_meas
			+ 0.5f * ((float) diff * CTRL_FREQ); // pulse/s
}

/* paper */
static int32_t paper_enc_last = 0;
static float paper_speed_meas = 0.0f;
static void Paper_Encoder_Speed_Update(void) {
	int32_t enc_now = read_paper_encoder_counter();
	int32_t diff = enc_now - paper_enc_last;
	paper_enc_last = enc_now;
	/* [修改] 加一阶低通（同 move）：抑制霍尔 ±1 抖动噪声。
	   原来 paper_speed_meas = diff*1000 无低通，±1 抖动直接变成 ±1000 pulse/s，
	   导致到位判定 |speed|<1 反复不满足，进纸电机静止时被死区补偿驱动微调 */
	paper_speed_meas = 0.5f * paper_speed_meas
			+ 0.5f * ((float) diff * CTRL_FREQ); // pulse/s
}

/* ============================================================
 * 通用位置外环（支持传入 loop_hz）
 * ============================================================ */
static float Position_Loop(pos_ctrl_t *p, int32_t target, int32_t pos,
		float loop_hz) {
	float err = (float) (target - pos);

	if (fabsf(err) <= p->dead_band) {
		p->prev_err = 0.0f;
		p->prev_deriv = 0.0f;
		return 0.0f;
	}

	float raw_deriv = (err - p->prev_err) / (1.0f / loop_hz);
	float filt_deriv = p->deriv_alpha * p->prev_deriv
			+ (1.0f - p->deriv_alpha) * raw_deriv;

	p->prev_err = err;
	p->prev_deriv = filt_deriv;

	float v_ref = p->kp * err + p->kd * filt_deriv;
	if (v_ref > p->vmax)
		v_ref = p->vmax;
	if (v_ref < -p->vmax)
		v_ref = -p->vmax;

	return v_ref;
}

/* ============================================================
 * 速度 PI（共享）
 * ============================================================ */
static float Speed_PI_Update(pi_ctrl_t *pi, float v_ref, float v_meas) {
	float err = v_ref - v_meas;

	pi->integral += pi->ki * err;
	if (pi->integral > pi->out_max)
		pi->integral = pi->out_max;
	if (pi->integral < -pi->out_max)
		pi->integral = -pi->out_max;

	float out = pi->kp * err + pi->integral;
	if (out > pi->out_max)
		out = pi->out_max;
	if (out < -pi->out_max)
		out = -pi->out_max;
	return out;
}

/* ============================================================
 * 死区补偿（共享）
 * ============================================================ */
/* [修改] 死区补偿改用每台可标定的 move_base/paper_base（替代固定 PWM_DEAD） */
static int16_t Apply_DeadZone(float u, int32_t dead) {
	if (u > 0) {
		if (u < dead)
			u = dead;
	} else if (u < 0) {
		if (u > -dead)
			u = -dead;
	}
	return (int16_t) u;
}

/* [autotune] 继电器驱动：给定速度指令 v_ref，跑速度环 PI + 死区补偿，输出 PWM。
   自整定振荡阶段用（跳过位置环 PD，直接以继电器 ±AT_RELAY_V 驱动，让系统在目标点自激振荡）。 */
void Move_RelayDrive(float v_ref) {
	float u = Speed_PI_Update(&move_spd_pi, v_ref, move_speed_meas);
	set_move_pwm_duty(Apply_DeadZone(u, (int32_t) current_config.move_base));
}

/* [autotune] 应用位置环 PD 参数（自整定结果） */
void Move_SetPosCtrl(float kp, float kd) {
	move_pos_ctrl.kp = kp;
	move_pos_ctrl.kd = kd;
}

/* [新增] 设单个 PID 参数（kp/kd/spd_kp 用 ×100，spd_ki 用 ×1000，S59 dead_band 用原始脉冲） */
void Motor_SetPIDParam(uint8_t which, uint16_t val) {
	switch (which) {
	case PID_MOVE_KP:       move_pos_ctrl.kp = val / 100.0f; break;
	case PID_MOVE_KD:       move_pos_ctrl.kd = val / 100.0f; break;
	case PID_MOVE_SPD_KP:   move_spd_pi.kp = val / 100.0f; break;
	case PID_MOVE_SPD_KI:   move_spd_pi.ki = val / 1000.0f; break;
	case PID_PAPER_KP:      paper_pos_ctrl.kp = val / 100.0f; break;
	case PID_PAPER_KD:      paper_pos_ctrl.kd = val / 100.0f; break;
	case PID_PAPER_SPD_KP:  paper_spd_pi.kp = val / 100.0f; break;
	case PID_PAPER_SPD_KI:  paper_spd_pi.ki = val / 1000.0f; break;
	case PID_PAPER_DEAD_BAND: paper_pos_ctrl.dead_band = (float) val; break;
	default: break;
	}
}

/* [新增] 上电从 current_config 全量加载 4 个控制器 PID 参数 */
void Motor_LoadPIDFromConfig(void) {
	move_pos_ctrl.kp = current_config.move_kp / 100.0f;
	move_pos_ctrl.kd = current_config.move_kd / 100.0f;
	move_spd_pi.kp = current_config.move_spd_kp / 100.0f;
	move_spd_pi.ki = current_config.move_spd_ki / 1000.0f;
	paper_pos_ctrl.kp = current_config.paper_kp / 100.0f;
	paper_pos_ctrl.kd = current_config.paper_kd / 100.0f;
	paper_pos_ctrl.dead_band = (float) current_config.paper_dead_band;
	paper_spd_pi.kp = current_config.paper_spd_kp / 100.0f;
	paper_spd_pi.ki = current_config.paper_spd_ki / 1000.0f;
}

/* ============================================================
 * [新增] 自动标定状态（变量前置，供控制任务判断是否跳过闭环）
 * ============================================================ */
static uint8_t calib_type = CALIB_NONE;   /* 当前标定类型 */
static uint32_t calib_tick = 0;           /* 标定计时（ms） */
static int32_t calib_pwm = 0;             /* 标定 base 时当前 PWM */
static int32_t calib_last_pos = 0;        /* 上次编码器位置（检测电机动） */
static int32_t calib_max_pos = 0;         /* 标定 sw_max 时最大位置 */
static uint32_t calib_stuck_ms = 0;       /* 撞限位判定（编码器不变计数） */
static uint32_t calib_elapsed_ms = 0;     /* [修复] sw_max 标定总时长（防编码器噪声反复清零 stuck 卡死） */

/* ============================================================
 * Move 控制任务（以 CTRL_FREQ 周期调用）
 * - 位置环按 MOVE_POS_LOOP_HZ 更新（pos_div_move）
 * ============================================================ */
void Move_Control_Task(void) {
	static uint16_t pos_div_move = 0;
	static uint16_t pos_cnt_move = 0;
	static float v_ref_last_move = 0.0f;

	if (pos_div_move == 0) {
		pos_div_move = (uint16_t) (CTRL_FREQ / MOVE_POS_LOOP_HZ + 0.5f);
		if (pos_div_move < 1)
			pos_div_move = 1;
	}

	/* 更新 move 编码器与速度 */
	TIM2_UpdateEncoderExtension();
	Move_Encoder_Speed_Update();

	/* [新增] 标定期间跳过闭环（标定直接控制 PWM） */
	if (calib_type != CALIB_NONE)
		return;

	/* [修改] 打孔时（开关开启）：停 PWM + 闭环断开（不读 CNT 做闭环），
	   电机定在打孔位置不跳；打孔期间 counter 的毛刺变动由打孔结束时恢复（punch_restore_move_counter） */
	if (punch_busy && current_config.punch_lock_move) {
		set_move_pwm_duty(0);
		move_speed_meas = 0.0f;
		return;
	}

//	Homing_Debounce_Update();

	if (Homing_Task() || handle_enable) {
		return;
	}

	/* [autotune] 自整定振荡阶段接管 PWM（继电器驱动 + 测量），跳过正常闭环 */
	if (Autotune_Tick())
		return;

	/* [修复] 定位表未初始化（上电到首次 homing 完成之间）不驱动电机，
	   否则 pulses_per_mm=0 使软件限位失效，编码器噪声可能让 err≈±32768 全速冲 home */
	if (!Notes_IsInited()) {
		set_move_pwm_duty(0);
		return;
	}

	int32_t pos = read_move_encoder_counter();
	/* [修改] 软件限位：目标钳位到 [-software_max, 0]（工作区在 home 负方向，防止超行程） */
	int32_t target = move_target_position;
	int32_t sw_max_pulses = Notes_GetSwMaxPulses(); /* [修复] 用整数缓存，避免每 1ms 软浮点 roundf */
	if (target < -sw_max_pulses)
		target = -sw_max_pulses;
	if (target > 0)
		target = 0;
	int32_t err = target - pos;

	/* [修复] 到位容忍至少 1 脉冲：move_dead_band=0 要求 err 精确=0，叠加死区补偿（最小 PWM=move_base）
	   导致极限环、到位判定物理不可达（外力/毛刺/惯性过冲让 err 卡 ±1 回不到 0），最小 1 兜底不死锁 */
	int32_t dead_band = (int32_t) current_config.move_dead_band;
	if (dead_band < 1)
		dead_band = 1;

	if (abs(err) <= dead_band && fabsf(move_speed_meas) < 1.0f) {
		set_move_pwm_duty(0);
		move_spd_pi.integral = 0.0f;
		v_ref_last_move = 0.0f;
		pos_cnt_move = 0;
		MCodeSimple_NotifyMoveDone();
		return;
	}

	if (++pos_cnt_move >= pos_div_move) {
		pos_cnt_move = 0;
		/* [修复] 用钳位后的 target 驱动位置环（原传未钳位的 move_target_position，
		   目标越界时电机被驱动越限/顶 home，且 err 已满足但速度环仍出力、永不完成卡死） */
		v_ref_last_move = Position_Loop(&move_pos_ctrl, target,
				pos, MOVE_POS_LOOP_HZ);
	}

	float u = Speed_PI_Update(&move_spd_pi, v_ref_last_move, move_speed_meas);
	set_move_pwm_duty(Apply_DeadZone(u, (int32_t) current_config.move_base));
}

/* ============================================================
 * [新增] 累计移动距离：把中断里累加的 |脉冲| 转 mm*100 累加到 move_total
 * 主循环调用（非中断），读完清零 move_total_pulses
 * ============================================================ */
void Move_AccumulateDistance(void) {
	if (move_total_pulses == 0)
		return;
	/* 临界区读-清零，避免与 TIM2 编码器中断竞态 */
	__disable_irq();
	int32_t pulses = move_total_pulses;
	move_total_pulses = 0;
	__enable_irq();

	/* 整数定点（无浮点）：720 脉冲 = 25.4mm = 2540 (mm×100)
	   所以 mm×100 = pulses × 2540 / 720 = pulses × 127 / 36。
	   余数分子保留到下次，避免每次截断丢精度。 */
	static uint32_t rem_num = 0;
	uint64_t num = (uint64_t)(uint32_t) pulses * 127ULL + rem_num;
	uint32_t mm100 = (uint32_t) (num / 36ULL);
	rem_num = (uint32_t) (num % 36ULL);
	current_config.move_total += mm100;
}

/* ============================================================
 * Move 复位
 * ============================================================ */
void Move_Control_Reset(void) {
	move_encoder_ext_count = 0;
	move_enc_last = 0;
	move_speed_meas = 0.0f;
	move_spd_pi.integral = 0.0f;
	move_pos_ctrl.prev_err = 0.0f;
	move_pos_ctrl.prev_deriv = 0.0f;

	/* 同步 TIM2 last cnt，避免启动跳变 */
	last_cnt_move = TIM2->CNT & 0xFFFF;
}

/* ============================================================
 * Paper 复位（方案 A：同步 last_cnt_paper）
 * ============================================================ */
void Paper_Control_Reset(void) {
	paper_encoder_ext_count = 0;
	paper_enc_last = 0;
	paper_speed_meas = 0.0f;

	paper_spd_pi.integral = 0.0f;
	paper_pos_ctrl.prev_err = 0.0f;
	paper_pos_ctrl.prev_deriv = 0.0f;
	paper_active = 0;

	/* 同步 last_cnt_paper 与 TIM4 当前计数，避免启动瞬变 */
	last_cnt_paper = TIM4->CNT & 0xFFFF;
}

/* ============================================================
 * Paper 控制任务（以 CTRL_FREQ 周期调用）
 * - 位置环按 PAPER_POS_LOOP_HZ 更新（pos_div_paper）
 * ============================================================ */
void Paper_Control_Task(void) {
	static uint16_t pos_div_paper = 0;
	static uint16_t pos_cnt_paper = 0;
	static float v_ref_last_paper = 0.0f;

	/* [修改] 编码器更新移到最前：无论 paper_active 与否都持续追踪，
	   避免手动模式/停止时 paper_encoder_ext_count 与 TIM4 脱节（手柄操作进纸后位置不准、退回操作前位置） */
	TIM4_UpdateEncoderExtension();
	Paper_Encoder_Speed_Update();

	/* [新增] 标定期间跳过闭环（标定直接控制 PWM） */
	if (calib_type != CALIB_NONE)
		return;

	if (!paper_active)
		return;

	if (pos_div_paper == 0) {
		pos_div_paper = (uint16_t) (CTRL_FREQ / PAPER_POS_LOOP_HZ + 0.5f);
		if (pos_div_paper < 1)
			pos_div_paper = 1;
	}

	/* [修改] Punch 互锁：打孔未到位，禁止进纸 */
	if (punch_busy) {
		set_paper_pwm_duty(0); /* 停 PWM，避免 punch 期间 paper 轴惯性运动 */
		return;
	}

	if (Homing_Task() || handle_enable) {
		return;
	}

	int32_t pos = read_paper_encoder_counter();
	int32_t err = paper_target_position - pos;

	if (abs(err) <= (int) paper_pos_ctrl.dead_band
			&& fabsf(paper_speed_meas) < 1.0f) {
		set_paper_pwm_duty(0);
		paper_spd_pi.integral = 0.0f;
		paper_active = 0;
		MCodeSimple_NotifyFeedDone();
		return;
	}

	if (++pos_cnt_paper >= pos_div_paper) {
		pos_cnt_paper = 0;
		v_ref_last_paper = Position_Loop(&paper_pos_ctrl, paper_target_position,
				pos, PAPER_POS_LOOP_HZ);
	}

	float u = Speed_PI_Update(&paper_spd_pi, v_ref_last_paper,
			paper_speed_meas);
	set_paper_pwm_duty(Apply_DeadZone(u, (int32_t) current_config.paper_base));
}

/* ============================================================
 * 打孔电机相关（保留原逻辑）
 * ============================================================ */
/* [新增] 打孔结束恢复 move counter 到打孔前值，并同步 last_cnt 避免接回闭环时跳变 */
static void punch_restore_move_counter(void) {
	if (current_config.punch_lock_move) {
		move_encoder_ext_count = move_enc_backup;
		last_cnt_move = TIM2->CNT & 0xFFFF;
		/* [修复] 同步速度观测上一值并清速度，否则打孔期噪声值导致下一 tick diff 大跳变、
		   经低通灌入速度环造成瞬时抖振 */
		move_enc_last = move_enc_backup;
		move_speed_meas = 0.0f;
	}
}

void Punch_Stop(void) {
	HAL_GPIO_WritePin(punch_swich_GPIO_Port, punch_b_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(punch_swich_GPIO_Port, punch_a_Pin, GPIO_PIN_SET);
}

void Punch_Start(uint16_t p) {
	if (p != 0) {
		if (p > 255)
			p = 255; /* [修改] clamp，避免 punch_target_cnt(uint8_t) 截断 */
		punch_target_cnt = p;  /* P 值 = 物理打孔次数（P2 打两下、P3 打三下） */
		punch_done_cnt = 0;
		punch_low_ms = 0;
		punch_stuck_ms = 0; /* [新增] 重置卡死超时计数 */
		punch_state = PUNCH_WAIT_LEAVE;

		/* [新增] 打孔前记录 move counter，打孔结束后恢复（丢弃打孔期震动毛刺） */
		if (current_config.punch_lock_move) {
			move_enc_backup = move_encoder_ext_count;
		}

		HAL_GPIO_WritePin(punch_swich_GPIO_Port, punch_b_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(punch_swich_GPIO_Port, punch_a_Pin, GPIO_PIN_RESET);
	} else {
		MCodeSimple_NotifyPunchDone();
	}
}

void Punch_Control_Task(void) {
	if (punch_done_cnt >= punch_target_cnt)
		return;

	GPIO_PinState sw = HAL_GPIO_ReadPin(punch_swich_GPIO_Port, punch_swich_Pin);

	switch (punch_state) {
	case PUNCH_WAIT_LEAVE:
		if (sw == GPIO_PIN_SET) {
			punch_state = PUNCH_WAIT_TOP;
			punch_low_ms = 0;
			punch_stuck_ms = 0;
		} else if (++punch_stuck_ms >= PUNCH_TIMEOUT_MS) {
			/* [修复] 超时保护：置 punch_done_cnt=punch_target_cnt 终止状态机，避免每 1ms 重复触发 */
			punch_restore_move_counter(); /* [新增] 恢复打孔前 counter */
			Punch_Stop();
			punch_done_cnt = punch_target_cnt;
			MCodeSimple_NotifyPunchDone();
		}
		break;

	case PUNCH_WAIT_TOP:
		if (sw == GPIO_PIN_RESET) {
			punch_low_ms++;
			punch_stuck_ms = 0; /* [新增] 复位卡死超时计数 */
			if (punch_low_ms >= PUNCH_AT_TOP_MS) {
					punch_done_cnt++;
					punch_low_ms = 0;
					if (punch_done_cnt >= punch_target_cnt) {
						/* [修改] 整个 P 命令完成（一个孔位）才累计打孔数 +1；
						   P2/P3 是物理上打两下/三下，但相当于一个孔，计数只 +1 */
						current_config.punch_total++;
						if (current_config.punch_total
								>= current_config.home_per_points) {
							current_config.punch_total = 0;
							save_config_request = 1; /* 请求主循环写 Flash（掉电保存清零） */
							/* [修复] 离线执行中抑制阈值回零，避免中途把主轴拉回原点打断文件；
							   文件结束的 M80 会统一回零 */
							if (!FileIO_IsRunning())
								homing_request = 1; /* 请求回零（动作空闲时执行） */
						}
						punch_restore_move_counter(); /* [新增] 恢复打孔前 counter */
						Punch_Stop();
						MCodeSimple_NotifyPunchDone();
					} else {
						punch_state = PUNCH_WAIT_LEAVE;
					}
				}
		} else {
			punch_low_ms = 0;
			if (++punch_stuck_ms >= PUNCH_TIMEOUT_MS) {
				/* [修复] 超时保护：置 punch_done_cnt=punch_target_cnt 终止状态机 */
				punch_restore_move_counter(); /* [新增] 恢复打孔前 counter */
				Punch_Stop();
				punch_done_cnt = punch_target_cnt;
				MCodeSimple_NotifyPunchDone();
			}
		}
		break;
	}
}

/* ============================================================
 * Paper 命令接口
 * ============================================================ */
void Paper_Feed(int16_t Y) {

	if (Y == 0) {
		// 停止进纸电机
		paper_active = 0;
		set_paper_pwm_duty(0);

		// 重置控制状态
		paper_spd_pi.integral = 0.0f;
		paper_pos_ctrl.prev_err = 0.0f;
		paper_pos_ctrl.prev_deriv = 0.0f;

		MCodeSimple_NotifyFeedDone();
		return;
	}

	float delta_f = (float) Y * paper_scale;
	int32_t delta_cnt = (int32_t) (delta_f + (delta_f >= 0 ? 0.5f : -0.5f));
	int32_t cur = read_paper_encoder_counter();
	paper_target_position = cur + delta_cnt;

	paper_spd_pi.integral = 0.0f;
	paper_pos_ctrl.prev_err = 0.0f;
	paper_pos_ctrl.prev_deriv = 0.0f;

	paper_active = 1;
}

/* ============================================================
 * Paper: 使用滚筒直径（mm）设置 paper_scale（脉冲 / mm）
 * paper_scale = counts_per_rev / (pi * diameter_mm)
 * ============================================================ */
void Paper_Set_RollerDiameter_mm(float diameter_mm) {
	if (diameter_mm <= 0.0f)
		return;
	float circumference_mm = (float) M_PI * diameter_mm;
	paper_scale = (float) PAPER_COUNTS_PER_REV / circumference_mm; /* pulses / mm */
}

/* ============================================================
 * [新增] 自动标定（出厂标定 + 每台差异化）
 *  - CALIB_MOVE_BASE / CALIB_PAPER_BASE：PWM 从 0 逐步加，电机动的那刻 = 死区
 *  - CALIB_SW_MAX：往远离 home 方向细靠走，撞限位后记录行程
 * ============================================================ */
uint8_t Calibration_IsActive(void) {
	return (calib_type != CALIB_NONE);
}

void Calibration_Start(uint8_t type) {
	calib_type = type;
	calib_tick = 0;
	calib_pwm = 0;
	calib_last_pos = 0;
	calib_max_pos = 0;
	calib_stuck_ms = 0;
	calib_elapsed_ms = 0;

	switch (type) {
	case CALIB_MOVE_BASE:
		calib_last_pos = read_move_encoder_counter();
		calib_max_pos = calib_last_pos;  /* [修改] 记录标定起始位置，用于最小移动量判定 */
		break;
	case CALIB_PAPER_BASE:
		calib_last_pos = read_paper_encoder_counter();
		calib_max_pos = calib_last_pos;  /* [修复] 记录 paper 标定起始位置，用于最小移动量判定 */
		break;
	case CALIB_SW_MAX:
		calib_max_pos = read_move_encoder_counter();
		calib_last_pos = calib_max_pos;
		break;
	default:
		calib_type = CALIB_NONE;
		break;
	}
}

/* 打印带符号 mm 值（两位小数）。用整数拆分避免浮点 printf：
   newlib-nano 的 %f 在 1ms 中断上下文占栈大、参数传递偶发异常，导致打印出天文数字 */
static void print_mm_2f(int32_t pulses) {
	float mm = Notes_ConvertPulsesToMM(pulses);
	int32_t mm100 = (int32_t) roundf(mm * 100.0f);
	int32_t ip = mm100 / 100;
	int32_t fp = mm100 % 100;
	if (fp < 0)
		fp = -fp;
	/* [修复] ip==0 且 mm100<0（如 -0.50mm）时整数除法 ip=0 丢负号，补 '-' */
	if (ip == 0 && mm100 < 0)
		USART_printf(&huart1, "-%ld.%02ld", (long) ip, (long) fp);
	else
		USART_printf(&huart1, "%ld.%02ld", (long) ip, (long) fp);
}

/* 标定 move_base：PWM 从 0 逐步加（远离 home 方向），电机动的那刻 = 死区 */
static void Calib_MoveBase(void) {
	if (calib_tick >= CALIB_PWM_STEP_MS) {
		calib_tick = 0;
		if (calib_pwm < PWM_MAX) {
			calib_pwm += CALIB_PWM_STEP;
			/* [修改] 远离 home 方向（负 PWM）：homing 后电机停在 home 开关上，
			   往 home 方向顶开关会堵转、测不出真实死区，须往远端走 */
			set_move_pwm_duty(-calib_pwm);
		}
	}
	int32_t pos = read_move_encoder_counter();
	/* [修改] 用相对标定起始位置的偏离量判定「电机启动」，过滤 ±1 编码器噪声 */
	int32_t moved = pos - calib_max_pos;
	if (moved < 0)
		moved = -moved;
	if (moved >= CALIB_MOVE_MIN_PULSES) {
		/* 电机动了：base = 触发 PWM + 余量 */
		int32_t base = calib_pwm + CALIB_BASE_MARGIN;
		if (base > PWM_MAX)
			base = PWM_MAX;
		current_config.move_base = (uint16_t) base;
		if (!calib_is_factory) current_config.calibrated = 1; /* [新增] 标记已标定 */
		save_config_request = 1;
		set_move_pwm_duty(0);
		calib_type = CALIB_NONE;
		USART_printf(&huart1, "[Calib] move_base = %u (pos=", (unsigned int) base);
		print_mm_2f(pos);
		USART_printf(&huart1, "mm)\n");
		homing_request = 1; /* 标定动了电机，回零 */
	} else if (calib_pwm >= PWM_MAX) {
		/* [新增] 超时保护：PWM 加到最大电机都没动，可能方向反了或机械卡死 */
		USART_printf(&huart1, "[Calib] move_base failed: PWM max, pos=");
		print_mm_2f(pos);
		USART_printf(&huart1, "mm\n");
		current_config.move_base = PWM_MAX;
		if (!calib_is_factory) current_config.calibrated = 1;
		save_config_request = 1;
		set_move_pwm_duty(0);
		calib_type = CALIB_NONE;
		homing_request = 1;
	}
	calib_last_pos = pos;
}

/* 标定 paper_base：同 move */
static void Calib_PaperBase(void) {
	if (calib_tick >= CALIB_PWM_STEP_MS) {
		calib_tick = 0;
		if (calib_pwm < PWM_MAX) {
			calib_pwm += CALIB_PWM_STEP;
			set_paper_pwm_duty(calib_pwm);
		}
	}
	int32_t pos = read_paper_encoder_counter();
	/* [修复] 用相对标定起始位置的偏离量判定「电机启动」，过滤 paper 霍尔 ±1 噪声
	   （原 pos != calib_last_pos 会因噪声误标 paper_base） */
	int32_t moved = pos - calib_max_pos;
	if (moved < 0)
		moved = -moved;
	if (moved >= CALIB_MOVE_MIN_PULSES) {
		int32_t base = calib_pwm + CALIB_BASE_MARGIN;
		if (base > PWM_MAX)
			base = PWM_MAX;
		current_config.paper_base = (uint16_t) base;
		if (!calib_is_factory) current_config.calibrated = 1; /* [新增] 标记已标定 */
		save_config_request = 1;
		set_paper_pwm_duty(0);
		Paper_Control_Reset(); /* 标定动了 paper，同步位置 */
		paper_target_position = 0; /* [修改] 同步 target 到 Reset 后位置，避免闭环拉回旧位置 */
		calib_type = CALIB_NONE;
		USART_printf(&huart1, "[Calib] paper_base = %u\n",
				(unsigned int) base);
	} else if (calib_pwm >= PWM_MAX) {
		/* [修复] 超时保护：PWM 加到最大电机都没动，避免 calib_type 永不复位瘫痪双轴闭环 */
		USART_printf(&huart1, "[Calib] paper_base failed: PWM max\n");
		current_config.paper_base = PWM_MAX;
		if (!calib_is_factory) current_config.calibrated = 1;
		save_config_request = 1;
		set_paper_pwm_duty(0);
		Paper_Control_Reset();
		paper_target_position = 0;
		calib_type = CALIB_NONE;
	}
	calib_last_pos = pos;
}

/* 标定 software_max：往远离 home 方向细靠走（用标出的 move_base），撞限位后记录行程 */
static void Calib_SwMax(void) {
	/* 远离 home 方向（软件限位方向），方向如需反向请调 HOMING_DIR 前的符号 */
	/* [修改] 用刚标出的 move_base + 余量，而非固定 HOMING_FINE_DUTY：
	   move_base 未标定前用默认值可能起不来电机，导致 software_max 误标成 0 */
	set_move_pwm_duty(-HOMING_DIR * ((int32_t) current_config.move_base + current_config.homing_fine));
	int32_t pos = read_move_encoder_counter();
	/* [修复] 整体超时：编码器噪声让 calib_stuck_ms 反复清零时，加总时长上限防卡死 */
	if (++calib_elapsed_ms >= CALIB_SWMAX_TIMEOUT_MS) {
		USART_printf(&huart1, "[Calib] software_max timeout\n");
		set_move_pwm_duty(0);
		calib_type = CALIB_NONE;
		homing_request = 1;
		return;
	}
	/* [调试] 每 100ms 打印一次位置（mm），看电机实际往哪走、走多远 */
	if ((calib_tick % 100) == 0) {
		USART_printf(&huart1, "[Calib] swmax pos=");
		print_mm_2f(pos);
		USART_printf(&huart1, "mm max=");
		print_mm_2f(calib_max_pos);
		USART_printf(&huart1, "mm stuck=%u\n", (unsigned int) calib_stuck_ms);
	}
	if (pos < calib_max_pos) {
		/* 远离 home = 位置减小（负方向），记录最远端（最小值） */
		calib_max_pos = pos;
		calib_stuck_ms = 0;
	} else if (++calib_stuck_ms >= CALIB_STUCK_MS) {
		/* 撞限位：software_max = |最远端| 的距离 - 余量 */
		int32_t dist = (calib_max_pos < 0) ? -calib_max_pos : calib_max_pos;
		float max_mm = Notes_ConvertPulsesToMM(dist);
		int32_t sw = (int32_t) (max_mm * 100.0f) - (int32_t) current_config.calib_sw_margin;
		if (sw < 0)
			sw = 0;
		/* [修复] 标定结果校验：sw 过小（<1mm）说明方向反/没起步/余量过大，
		   保留旧值避免 software_max=0 让主轴被钳死 */
		if (sw < 100) {
			USART_printf(&huart1, "[Calib] software_max failed: too small (%ld.%02ld mm)\n",
					(long) (sw / 100), (long) (sw % 100));
		} else {
			current_config.software_max = (uint16_t) sw;
			Notes_SetSwMaxMM100((uint16_t) sw); /* [新增] 同步软件限位脉冲缓存 */
			if (!calib_is_factory) current_config.calibrated = 1; /* [新增] 标记已标定 */
			save_config_request = 1;
			USART_printf(&huart1, "[Calib] software_max = %ld.%02ld mm\n",
					(long) (sw / 100), (long) (sw % 100));
		}
		set_move_pwm_duty(0);
		calib_type = CALIB_NONE;
		homing_request = 1; /* 标定走到头了，回零 */
	}
}

void Calibration_Task(void) {
	if (calib_type == CALIB_NONE)
		return;

	calib_tick++;

	switch (calib_type) {
	case CALIB_MOVE_BASE:
		Calib_MoveBase();
		break;
	case CALIB_PAPER_BASE:
		Calib_PaperBase();
		break;
	case CALIB_SW_MAX:
		Calib_SwMax();
		break;
	default:
		calib_type = CALIB_NONE;
		break;
	}
}

/* [新增] 停止标定（S0 急停用） */
void Calibration_Stop(void) {
	calib_type = CALIB_NONE;
	set_move_pwm_duty(0);
	set_paper_pwm_duty(0);
}

/* ============================================================
 * 结束
 * ============================================================ */
