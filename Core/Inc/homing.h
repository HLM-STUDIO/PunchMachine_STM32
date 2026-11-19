/*
 * homing.h
 *
 *  Created on: 2026年1月23日
 *      Author: huang
 */

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 用户可在此处修改参数，也可以在 homing.c 中直接修改宏 */
#define HOMING_DIR 1 /* 方向：+1 或 -1，按你的机械调 */
#define HOMING_COARSE_DUTY 1000 /* 粗靠 PWM（开环） */
#define HOMING_FINE_DUTY 500 /* 细靠 PWM（慢速） */
#define HOMING_BACKOFF_DUTY 500 /* 退开 PWM */

/* [已废弃] homing 三 margin 已迁移到配置字段 current_config.homing_coarse/fine/backoff（S60-S62 可调）。
   此处宏仅作默认值参考（640/60/140），代码中勿再使用（2026-11 迁移）。 */
#define HOMING_COARSE_MARGIN  640  /* 默认值参考：粗靠阈值 move_base + 640 */
#define HOMING_FINE_MARGIN    60   /* 默认值参考：细靠阈值 move_base + 60（<30 堵转） */
#define HOMING_BACKOFF_MARGIN 140  /* 默认值参考：退开阈值 move_base + 140 */
#define HOMING_COARSE_TIMEOUT_S 15.0f /* 粗靠超时（秒） */
#define HOMING_BACKOFF_MS 500 /* 退开时间（毫秒） */
#define HOMING_FINE_TIMEOUT_S 5.0f /* 细靠超时（秒） */
#define HOMING_BACKOFF_INIT_MS 800 /*开始卡住反向时间*/
#define HOME_FINAL_OFFSET 0 /* 找到后编码器计数置为该值 */

#define HOMING_RELEASE_DEBOUNCE_MS   10    /* 等待开关释放并连续高电平的去抖时间（ms） */
#define HOMING_RELEASE_TIMEOUT_MS   500   /* 如果一直无法释放，最多尝试多少 ms 后放弃（安全保护） */

/* 去抖相关参数（按需调整） */
#define HOMING_DEBOUNCE_MS_COUNT   10    /* 需要连续多少个 1ms 采样为低才确认（例如 20 => 20ms） */


/* homing 状态 */
typedef enum {
	HOMING_IDLE = 0,
	HOMING_COARSE,
	HOMING_BACKOFF,
	HOMING_FINE,
	HOMING_DONE,
	HOMING_START_FROM_HOME,
	HOMING_FAILED /* [新增] 回零失败（超时等异常，区别于 DONE） */
} homing_state_e;

//extern volatile uint8_t raw_exti_flag;
//extern volatile uint32_t exti_start_ms;

void Home_Init(void); /* 可选：初始化（内部也会懒初始化） */
void Home_Start(void); /* 启动找零 */
void Home_Stop(void); /* 取消找零 */
uint8_t Home_IsDone(void); /* 是否完成 */
uint8_t Home_IsFailed(void); /* [新增] 是否回零失败（超时等异常） */
void Homing_Debounce_Update(void);

/* 在你的控制循环最开始调用：如果返回 1 则表示 homing 正在处理，主控制任务应直接 return */
int Homing_Task(void);
uint8_t Homing_IsActive(void); /* [新增] 返回 homing 是否正在进行中 */

extern volatile uint8_t homing_request; /* [新增] 回零请求标志(1=请求，动作空闲时执行) */

#endif /* INC_HOMING_H_ */
