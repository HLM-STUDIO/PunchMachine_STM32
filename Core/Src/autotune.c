/*
 * autotune.c
 * 移动轴 PID 自整定（继电反馈法）
 *
 * 关键参数（仿真验证见 D:\PUNCH\autotune_verify.py）：
 *   AT_RELAY_V=500 pulse/s：继电器速度指令幅度，行程1000~2000/时间常数5~30ms/死区200~500 全工况整定成功
 *   AT_SAFETY=0.5：ZN 系数缩放，实机偏激进/偏肉调这一个数即可
 *   AT_MARGIN=150 脉冲：振荡离原点/限位的安全余量（约 5.3mm）
 *
 * 浮点计算只在 Service（主循环）里做；Tick（1ms 中断）只用整数（误差/过零/峰值），
 * 避免 STM32F103 无 FPU 在中断里软浮点拖慢控制周期。
 */
#include "autotune.h"
#include "motor.h"    /* set_move_pwm_duty / read_move_encoder_counter / Move_Control_Reset / Move_RelayDrive / Move_SetPosCtrl / Move_Get_Speed */
#include "mcode.h"    /* move_busy */
#include "homing.h"   /* Home_Start / Home_IsDone / Home_IsFailed */
#include "notes.h"    /* Notes_GetSwMaxPulses */
#include "system.h"   /* current_config / L */
#include "usart.h"    /* USART_printf */

/* ============ 整定参数 ============ */
#define AT_RELAY_V     500.0f   /* 继电器速度指令幅度 (pulse/s)，约 27% 满速 */
#define AT_MARGIN      150      /* 振荡离原点/限位安全余量 (pulse) */
#define AT_SAFETY      0.5f     /* ZN 安全缩放系数 */
#define AT_HOMING_TMO  10000UL  /* 回零超时 ms */
#define AT_MOVE_TMO    5000UL   /* 移动超时 ms */
#define AT_OSC_TMO     5000UL   /* 振荡超时 ms */
#define AT_MIN_CROSS   8        /* 最少有效过零次数（测出稳定 Tu/a） */
#define AT_WARMUP_MS   1500UL   /* 进入振荡的过渡期（不统计） */

/* ============ 状态 ============ */
static autotune_state_t at_state = AT_IDLE;
static uint32_t at_phase_start;     /* 当前阶段起始 HAL_GetTick */
static uint32_t at_tick;            /* 振荡阶段 tick 计数（相对，ms） */

/* ============ 振荡测量 ============ */
static int32_t  at_err_prev;        /* 上一 tick 误差 */
static uint32_t at_last_cross;      /* 上次过零 tick */
static int32_t  at_half_peak;       /* 当前半周期峰值 */
static uint16_t at_cross_cnt;       /* 有效过零计数（warmup 后） */
static uint32_t at_half_sum;        /* 半周期累加 (ms) */
static int32_t  at_amp_sum;         /* 幅度累加 (pulse) */

/* ============ 结果 ============ */
static float at_kp, at_kd;
static uint32_t at_Tu_ms;
static int32_t at_target;           /* 整定点（工作区中点） */

/* ============ 工具 ============ */
static uint8_t at_in_progress(void) {
	return (at_state != AT_IDLE && at_state != AT_DONE && at_state != AT_FAILED);
}

/* ============ 对外接口 ============ */
void Autotune_Start(void) {
	if (at_in_progress())
		return;                     /* 整定中重复触发忽略 */
	at_state = AT_HOMING;
	at_phase_start = HAL_GetTick();
	at_cross_cnt = 0;
	at_half_sum = 0;
	at_amp_sum = 0;
	at_kp = at_kd = 0.0f;
	at_Tu_ms = 0;
	at_target = 0;
	Home_Start();                   /* 从已知零点出发，软限位坐标才有意义 */
}

void Autotune_Abort(void) {
	set_move_pwm_duty(0);
	at_state = AT_IDLE;
}

uint8_t Autotune_IsActive(void) {
	return at_in_progress();
}

autotune_state_t Autotune_GetState(void) {
	return at_state;
}

/* 1ms 中断：只在振荡阶段接管，做继电器控制 + 测量（纯整数） */
uint8_t Autotune_Tick(void) {
	if (at_state != AT_OSCILLATE)
		return 0;

	at_tick++;

	int32_t pos = read_move_encoder_counter();
	int32_t err = at_target - pos;

	/* 继电器：err>0 正向，err<0 反向 */
	float v_ref = (err > 0) ? AT_RELAY_V : -AT_RELAY_V;

	/* 软限位保护兜底：逼近原点/限位强制反向，绝不允许顶机械 */
	int32_t sw_max = Notes_GetSwMaxPulses();
	if (pos > -AT_MARGIN) {
		v_ref = -AT_RELAY_V;        /* 太靠原点(0)方向 */
	} else if (pos < -sw_max + AT_MARGIN) {
		v_ref = AT_RELAY_V;         /* 太靠软件限位方向 */
	}

	Move_RelayDrive(v_ref);         /* 速度环 PI + 死区补偿 + 输出 PWM */

	/* 过零检测 + 半周期/峰值统计 */
	if ((at_err_prev > 0 && err <= 0) || (at_err_prev < 0 && err >= 0)) {
		if (at_tick > AT_WARMUP_MS && at_last_cross != 0) {
			at_half_sum += (at_tick - at_last_cross);
			at_amp_sum  += at_half_peak;
			at_cross_cnt++;
		}
		at_last_cross = at_tick;
		at_half_peak = 0;
	}
	if (abs(err) > at_half_peak)
		at_half_peak = abs(err);
	at_err_prev = err;

	return 1;
}

/* 主循环：状态机推进 */
void Autotune_Service(void) {
	switch (at_state) {
	case AT_HOMING:
		if (Home_IsDone()) {
			/* 回零完成：定整定点 = 工作区中点，闭环移过去 */
			at_target = -(Notes_GetSwMaxPulses() / 2);
			Move_Control_Reset();
			move_target_position = at_target;
			move_busy = 1;
			at_state = AT_MOVE;
			at_phase_start = HAL_GetTick();
		} else if (Home_IsFailed() || (HAL_GetTick() - at_phase_start > AT_HOMING_TMO)) {
			at_state = AT_FAILED;
		}
		break;

	case AT_MOVE:
		if (move_busy == 0) {
			/* 到位：进入振荡，清测量状态。只清积分/微分，保留当前位置（-sw_max/2），
			   否则 Move_Control_Reset 会把位置清回 0，err=-sw_max/2 冲向软件限位 */
			Move_SpdIntegralReset();
			at_tick = 0;
			at_err_prev = 0;
			at_last_cross = 0;
			at_half_peak = 0;
			at_cross_cnt = 0;
			at_half_sum = 0;
			at_amp_sum = 0;
			at_state = AT_OSCILLATE;
			at_phase_start = HAL_GetTick();
		} else if (HAL_GetTick() - at_phase_start > AT_MOVE_TMO) {
			at_state = AT_FAILED;
		}
		break;

	case AT_OSCILLATE:
		if (at_cross_cnt >= AT_MIN_CROSS) {
			at_state = AT_CALC;     /* 振荡稳定，进主循环算参数 */
		} else if (HAL_GetTick() - at_phase_start > AT_OSC_TMO) {
			at_state = AT_FAILED;
		}
		break;

	case AT_CALC: {
		/* 浮点计算（主循环，无 FPU 也可接受，一次性） */
		set_move_pwm_duty(0);
		float half_ms = (at_cross_cnt > 0) ? ((float) at_half_sum / at_cross_cnt) : 0.0f;
		float amp = (at_cross_cnt > 0) ? ((float) at_amp_sum / at_cross_cnt) : 0.0f;
		at_Tu_ms = (uint32_t) (2.0f * half_ms);
		if (amp < 0.5f) {
			at_state = AT_FAILED;   /* 幅度异常小，判定失败 */
			break;
		}
		float Ku = 4.0f * AT_RELAY_V / (3.14159265f * amp);
		float Tu_s = at_Tu_ms / 1000.0f;
		at_kp = 0.6f * Ku * AT_SAFETY;
		at_kd = at_kp * Tu_s / 8.0f;
		at_state = AT_APPLY;
		break;
	}

	case AT_APPLY:
		Move_SetPosCtrl(at_kp, at_kd);   /* 应用整定结果 */
		if (pid_save_enable) {
			/* [新增] S63=1 时整定结果掉电保存：写 current_config + 请求主循环写 Flash */
			current_config.move_kp = (uint16_t) (at_kp * 100.0f);
			current_config.move_kd = (uint16_t) (at_kd * 100.0f);
			save_config_request = 1;
		}
		at_state = AT_DONE;
		break;

	case AT_DONE: {
		/* [修复] newlib-nano 无 printf_float，禁 %f（会打印空）。拆整数/小数打印 */
		int32_t kp100 = (int32_t) (at_kp * 100.0f + 0.5f);
		int32_t kd1000 = (int32_t) (at_kd * 1000.0f + 0.5f);
		USART_printf(&huart1, "[Tune] OK: kp=%ld.%02ld kd=%ld.%03ld Tu=%lu ms\r\n",
				(long) (kp100 / 100), (long) (kp100 % 100),
				(long) (kd1000 / 1000), (long) (kd1000 % 1000),
				(unsigned long) at_Tu_ms);
		at_state = AT_IDLE;
		break;
	}

	case AT_FAILED:
		USART_printf(&huart1, "[Tune] FAILED\r\n");
		set_move_pwm_duty(0);
		at_state = AT_IDLE;
		break;

	default:
		break;
	}
}
