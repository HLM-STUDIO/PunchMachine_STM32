/*
 * autotune.h
 * 移动轴 PID 自整定模块（继电反馈法 Åström-Hägglund）
 *
 * 功能：S88 触发，全自动整定位置环 PD 参数（kp/kd），速度环 PI 保持现有参数。
 * 流程：回零 -> 闭环移动到工作区中点 -> 继电器振荡测 Tu/a -> 算 Ku -> ZN(×安全系数) -> 应用参数。
 *
 * 安全设计：
 *   1) 整定点取工作区中点（离原点和软件限位都最远）；
 *   2) 移动阶段用正常闭环（自带软限位钳位）；
 *   3) 振荡阶段软限位保护兜底（逼近边界强制反向）；
 *   4) 每阶段超时保护，卡住即 FAILED 停 PWM，不死锁。
 *
 * 仿真验证（D:\PUNCH\autotune_verify.py）：
 *   继电器幅度 500 pulse/s 在行程 1000~2000/时间常数 5~30ms/死区 200~500 全工况
 *   整定成功且振荡不越界；外力±5脉冲、编码器毛刺均稳健。
 */
#ifndef INC_AUTOTUNE_H_
#define INC_AUTOTUNE_H_

#include <stdint.h>

typedef enum {
	AT_IDLE = 0,     /* 空闲 */
	AT_HOMING,       /* 回零中 */
	AT_MOVE,         /* 闭环移动到整定点 */
	AT_OSCILLATE,    /* 继电器振荡测量 */
	AT_CALC,         /* 计算参数（主循环，浮点） */
	AT_APPLY,        /* 应用参数 */
	AT_DONE,         /* 完成 */
	AT_FAILED,       /* 失败（超时/振荡异常） */
} autotune_state_t;

/* 触发自整定（S88）。整定中重复触发忽略。 */
void Autotune_Start(void);

/* 中止自整定（S0 急停）。立即停 PWM、回 IDLE。 */
void Autotune_Abort(void);

/* 是否在整定中（IDLE/DONE/FAILED 以外都算进行中） */
uint8_t Autotune_IsActive(void);

autotune_state_t Autotune_GetState(void);

/* 1ms 中断调用（Move_Control_Task 内，正常闭环之前）。
   返回 1 = 本模块接管 PWM 输出（调用方应 return，跳过正常闭环）；
   返回 0 = 不接管。 */
uint8_t Autotune_Tick(void);

/* 主循环调用：状态机推进（触发回零/移动、计算、应用、回传结果） */
void Autotune_Service(void);

#endif /* INC_AUTOTUNE_H_ */
