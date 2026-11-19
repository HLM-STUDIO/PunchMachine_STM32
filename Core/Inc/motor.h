/*
 * motor.h
 *
 *  Created on: 2026年1月22日
 *      Author: huang
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

/* ============================================================
 * 参数定义
 * ============================================================ */
#define PWM_MAX        3600
#define PWM_DEAD       230      // ⭐启动死区补偿（必须）
#define CTRL_FREQ      1000.0f  // 1kHz 控制（任务调用频率）

/* ============================================================
 * [新增] 自动标定参数（出厂标定 + 每台差异化）
 * ============================================================ */
#define CALIB_NONE       0   /* 未在标定 */
#define CALIB_SW_MAX     1   /* 标定 software_max（行程范围） */
#define CALIB_MOVE_BASE  2   /* 标定 move_base（move 死区） */
#define CALIB_PAPER_BASE 3   /* 标定 paper_base（paper 死区） */

#define CALIB_PWM_STEP        10    /* 标定 base 时 PWM 步进 */
#define CALIB_PWM_STEP_MS     100    /* 标定 base 时每步周期（ms） */
#define CALIB_BASE_MARGIN     30    /* base 标定余量（PWM）：触发值 + 余量保证稳定启动 */
#define CALIB_SWMAX_TIMEOUT_MS 30000 /* [修复] software_max 标定总时长上限（防噪声反复清零 stuck 卡死） */
#define CALIB_STUCK_MS        500   /* software_max 撞限位判定：编码器连续多少 ms 不变 */
#define CALIB_SWMAX_PWM_THRESHOLD 2700 /* software_max 闭环标定：闭环输出 PWM 超此阈值(75% PWM_MAX)判撞限位 */
#define CALIB_SWMAX_SPEED       500  /* software_max 闭环标定：限速慢行(pulse/s)，避免目标无限远致 PWM 立即顶满误判撞限位 */
#define CALIB_SWMAX_PWM_LIMIT   2000 /* software_max 标定：PWM 上限(±2000)，撞限位堵转不全功率(3600)防过热 */
/* software_max 标定余量已改为运行期配置 current_config.calib_sw_margin（S86 设置，mm×100） */
#define CALIB_MOVE_MIN_PULSES 5     /* move_base 标定：编码器至少动这么多脉冲才算「电机启动」，过滤 ±1 噪声 */
#define POS_HOLD_ZONE  0        // [修改] 位置死区改回 0：打孔时已锁死 counter，移动到位要求精确对齐

/* 位置环目标频率（可以分别设定 move 与 paper） */
#define MOVE_POS_LOOP_HZ   500.0f   // move 位置环目标频率 (Hz)
#define PAPER_POS_LOOP_HZ  200.0f   // paper 位置环目标频率 (Hz)

/* 进纸编码器参数 */
#define PAPER_ENCODER_PPR      88      // 霍尔编码器 per motor rev
#define PAPER_REDUCTION        78      // 减速比
#define PAPER_COUNTS_PER_REV  (PAPER_ENCODER_PPR * PAPER_REDUCTION) /* 88*78 = 6864 */

/* ============================================================
 * 控制结构体
 * ============================================================ */
typedef struct {
	float kp;
	float kd;
	float prev_err;
	float prev_deriv;
	float deriv_alpha;
	float vmax;
	float dead_band;
} pos_ctrl_t;

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_max;
} pi_ctrl_t;

/* ========== move（主轴） ========== */
extern volatile int32_t move_encoder_ext_count;
extern volatile int32_t move_total_pulses;  /* [新增] 累计移动脉冲（|diff|），主循环转 mm 累加到 move_total */
extern volatile int32_t move_target_position; /* [修改] 加 volatile：1ms 中断读、命令回调写 */

int32_t read_move_encoder_counter(void); /* [新增] 读取 move 光栅计数（退出手动模式时同步位置用） */
void Move_AccumulateDistance(void);       /* [新增] 把累计移动脉冲转 mm×100（整数定点）累加到 move_total */
void Encoder_SetFilter(uint16_t filter);  /* [新增] 运行时设置编码器数字滤波(0-15)，S87；改 TIM2/TIM4 CCMR1 的 IC1F/IC2F */

void set_move_pwm_duty(int32_t duty);
void Move_Control_Task(void);        // 建议每 CTRL_FREQ 周期调用
void Move_Control_Reset(void);
float Move_Get_Speed(void);
void Move_RelayDrive(float v_ref);   /* [autotune] 继电器驱动：速度环PI+死区补偿输出PWM，自整定振荡阶段用 */
void Move_SetPosCtrl(float kp, float kd); /* [autotune] 应用位置环 PD 参数（自整定结果） */
void Move_SpdIntegralReset(void);    /* [autotune] 只清速度环积分/微分状态，不清编码器位置（进入振荡前用，保留当前位置） */

/* [新增] PID 参数开放接口（S51-S62） */
enum {
	PID_MOVE_KP = 0, PID_MOVE_KD, PID_MOVE_SPD_KP, PID_MOVE_SPD_KI,
	PID_PAPER_KP, PID_PAPER_KD, PID_PAPER_SPD_KP, PID_PAPER_SPD_KI,
	PID_PAPER_DEAD_BAND
};
void Motor_SetPIDParam(uint8_t which, uint16_t val); /* 设单个参数（kp/kd/ki ×1000，dead_band 原始脉冲） */
void Motor_LoadPIDFromConfig(void);                   /* 上电从 current_config 全量加载 4 个控制器 */

/* ========== paper（进纸轴） ========== */
extern volatile int32_t paper_encoder_ext_count;
extern volatile int32_t paper_target_position; /* [修改] 加 volatile */
extern volatile uint8_t paper_active; /* 1: 正在进纸 */
extern float paper_scale; /* Y 单位 -> 脉冲倍率 */

int32_t read_paper_encoder_counter(void); /* [新增] 读取 paper 编码器计数（退出手动模式时同步位置用） */

/* paper PWM 接口 */
void set_paper_pwm_duty(int32_t duty);

/* 控制任务（独立） */
void Paper_Control_Task(void);       // 建议每 CTRL_FREQ 周期调用
void Paper_Control_Reset(void);

/* 辅助接口 */
void Paper_Feed(int16_t Y); /* 外部命令接口（保持名字兼容） */
void Paper_Set_RollerDiameter_mm(float diameter_mm); /* 根据滚筒直径设置 paper_scale（脉冲/mm）*/
int32_t Paper_Get_Position(void); /* 线程安全性视调用端，必要时在外部临界区保护 */

/* ========== 打孔相关 ========== */
typedef enum {
    PUNCH_WAIT_LEAVE = 0,
    PUNCH_WAIT_TOP,
} punch_state_t;
extern punch_state_t punch_state;

extern uint8_t  punch_target_cnt;
extern uint8_t  punch_done_cnt;
extern uint16_t punch_low_ms;

#define PUNCH_AT_TOP_MS   50
#define PUNCH_TIMEOUT_MS  3000 /* [新增] 打孔状态机卡死超时保护(3s)，开关异常时强制完成，避免连锁卡死 move/paper */

void Punch_Start(uint16_t times);
void Punch_Stop(void); /* [新增] 打孔电机停止（S0 急停用） */
void Punch_Control_Task(void);

/* 上层通知回调（在其它模块实现） */
void MCodeSimple_NotifyMoveDone(void);
void MCodeSimple_NotifyFeedDone(void);
void MCodeSimple_NotifyPunchDone(void);

/* ========== 自动标定 ========== */
void Calibration_Start(uint8_t type);  /* [新增] 触发标定（CALIB_*） */
void Calibration_Task(void);           /* [新增] 标定任务（1ms 调用） */
uint8_t Calibration_IsActive(void);    /* [新增] 标定是否进行中 */
void Calibration_Stop(void);           /* [新增] 停止标定（S0 急停用） */

#endif /* INC_MOTOR_H_ */
