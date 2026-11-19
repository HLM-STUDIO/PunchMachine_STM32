/*
 * homing.c
 *
 *  Created on: 2026年1月23日
 *      Author: huang
 */
#include "homing.h"
#include "system.h" /* [修改] 访问 current_config.home_bias，替换硬编码 Notes_Init(59.5f) */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "motor.h"
#include "notes.h"
#include "main.h"

volatile uint8_t homing_request = 0; /* [新增] 回零请求标志：由 M80/累计满N孔 置1，动作空闲时执行 */

/* 内部静态变量 */
static volatile homing_state_e homing_state = HOMING_IDLE;
static volatile uint8_t homing_active = 0;
static volatile uint8_t home_switch_flag = 0; /* 由外部中断置位 */

/* [修改] homing 各阶段计时器（文件级，供 Home_Start 重置；原来在 Homing_Task 内为 static，重复回零会沿用旧时间戳立即超时） */
static uint32_t homing_reverse_start_ms = 0;
static uint32_t homing_pause_start_ms = 0;
static uint32_t homing_coarse_start_ms = 0;
static uint32_t homing_backoff_start_ms = 0;
static uint32_t homing_fine_start_ms = 0;

/* 开关去抖相关变量 */
static uint8_t switch_debounce_counter = 0;
static GPIO_PinState last_switch_state = GPIO_PIN_SET; /* 假设默认高电平 */
static uint8_t debounce_done = 0; /* 去抖已完成标志 */
static uint8_t fine_zero_latched = 0; /* [新增] 细靠检测到按下即归零的锁存，防弹跳重复归零 */
static uint8_t fine_reached = 0; /* [新增] 细靠阶段是否曾归零过，区分「正常细靠」与「弹跳恢复高」 */

/* 延迟将以调用频率（例如 1kHz）为单位计算；如果你的控制循环频率不同，请确保调用频率与 motor 中一致 */

void Home_Init(void) {
    homing_state = HOMING_IDLE;
    homing_active = 0;
    home_switch_flag = 0;

    /* 初始化去抖变量 */
    switch_debounce_counter = 0;
    last_switch_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin); /* [修改] 实际读取开关状态（原硬编码 SET，注释与实际不符） */
    debounce_done = 0;
}

void Home_Start(void) {
    /* 首先检查开关状态 */
    GPIO_PinState switch_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin);

    /* 如果开关已经按下（低电平），说明已经在原点位置 */
    if (switch_state == GPIO_PIN_RESET) {
        homing_state = HOMING_START_FROM_HOME;  /* 新增状态 */
    } else {
        homing_state = HOMING_COARSE;
    }

    homing_active = 1;
    home_switch_flag = 0;

    /* [修改] 重置各阶段计时器，避免重复回零沿用旧时间戳 */
    homing_reverse_start_ms = 0;
    homing_pause_start_ms = 0;
    homing_coarse_start_ms = 0;
    homing_backoff_start_ms = 0;
    homing_fine_start_ms = 0;

    /* 重置去抖状态 */
    switch_debounce_counter = 0;
    debounce_done = 0;
    fine_zero_latched = 0; /* [新增] 重置细靠归零锁存 */
    fine_reached = 0; /* [新增] 重置「曾归零」标记 */
    /* [修改] 同步 last_switch_state 到当前开关状态，避免上次 homing 残留导致首次去抖判态偏差 */
    last_switch_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin);
}

void Home_Stop(void) {
    homing_active = 0;
    homing_state = HOMING_IDLE;
    home_switch_flag = 0;
    set_move_pwm_duty(0);
}

uint8_t Home_IsDone(void) {
    return (homing_state == HOMING_DONE);
}

/* [新增] 返回回零是否失败（超时等异常） */
uint8_t Home_IsFailed(void) {
    return (homing_state == HOMING_FAILED);
}

/* [新增] 返回 homing 是否正在进行中 */
uint8_t Homing_IsActive(void) {
    return homing_active;
}

/* 内部函数：检查开关状态并进行去抖 */
static void Check_Switch_State(void) {
    GPIO_PinState current_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin);

    /* 状态变化检测 */
    if (current_state != last_switch_state) {
        switch_debounce_counter = 0;
        debounce_done = 0;
    } else {
        switch_debounce_counter++;

        /* 连续相同状态达到去抖时间（比如10ms） */
        if (switch_debounce_counter >= HOMING_DEBOUNCE_MS_COUNT && !debounce_done) {
            debounce_done = 1;

            /* 如果是低电平（按下）且之前没有标记 */
            if (current_state == GPIO_PIN_RESET) {
                home_switch_flag = 1;
            }
        }
    }

    last_switch_state = current_state;
}

/* Homing_Task: 在你的主控制循环（例如 Motor_Control_Task 的起始）以固定频率调用。
 * 如果返回 1，表示 homing 正在处理（主循环应直接 return），否则返回 0。
 */
int Homing_Task(void) {
    if (!homing_active)
        return 0;

    /* [修复] 防重入：Move_Control_Task 和 Paper_Control_Task 各调一次（同一 1ms tick），
       第二次调用直接返回"homing 中"（让调用方 return），但不重复去抖/状态机，避免跑双倍 */
    static uint32_t homing_last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now == homing_last_tick)
        return 1;
    homing_last_tick = now;

    /* 在Homing_Task中直接检查开关状态（1ms调用一次，正好用于去抖） */
    Check_Switch_State();

    switch (homing_state) {
    /* 新增状态：从原点位置开始，先反向移动一小段时间 */
    case HOMING_START_FROM_HOME: {
        /* [修改] 修复「反向退开距离翻倍」bug：原代码退开完成后把 homing_reverse_start_ms 清零，
           导致下一周期重新计时、重新输出反向 PWM。现改为 if/else 平级分支，
           退开完成即停电机并进入停稳暂停，不再重新计时 */
        if (homing_reverse_start_ms == 0)
            homing_reverse_start_ms = HAL_GetTick();

        if ((HAL_GetTick() - homing_reverse_start_ms) < HOMING_BACKOFF_INIT_MS) {
            /* 反向退开，离开原点开关（退开用 BACKOFF_MARGIN，与 HOMING_BACKOFF 一致，勿用 FINE_MARGIN） */
            set_move_pwm_duty(-HOMING_DIR * ((int32_t) current_config.move_base + current_config.homing_backoff));
        } else {
            /* 退开完成：停电机，短暂停顿让电机停稳 */
            set_move_pwm_duty(0);
            if (homing_pause_start_ms == 0)
                homing_pause_start_ms = HAL_GetTick();

            if ((HAL_GetTick() - homing_pause_start_ms) >= 100) {
                homing_pause_start_ms = 0;
                homing_reverse_start_ms = 0;
                /* 现在可以开始正常的回零流程 */
                homing_state = HOMING_COARSE;
                home_switch_flag = 0;
                /* 重置去抖状态，准备检测开关 */
                switch_debounce_counter = 0;
                debounce_done = 0;
                last_switch_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin);
            }
        }
        break;
    }

    case HOMING_COARSE: {
        /* 粗靠：持续给固定方向 PWM，直到开关触发或超时 */
        set_move_pwm_duty(HOMING_DIR * ((int32_t) current_config.move_base + current_config.homing_coarse));

        /* 触发开关 */
        if (home_switch_flag) {
            home_switch_flag = 0;
            homing_state = HOMING_BACKOFF;
            /* 重置去抖状态 */
            switch_debounce_counter = 0;
            debounce_done = 0;
            /* 立即切到反向退开 */
        } else {
            /* 使用 HAL_GetTick 更准确 */
            if (homing_coarse_start_ms == 0)
                homing_coarse_start_ms = HAL_GetTick();
            if ((HAL_GetTick() - homing_coarse_start_ms)
                    >= (uint32_t) (HOMING_COARSE_TIMEOUT_S * 1000.0f)) {
                homing_coarse_start_ms = 0;
                /* [修改] 超时标记失败（原为 HOMING_DONE，把超时当成功是隐患） */
                set_move_pwm_duty(0);
                homing_active = 0;
                homing_state = HOMING_FAILED;
            }
        }
        break;
    }

    case HOMING_BACKOFF: {
        /* 退开一小段时间避免连续接触 */
        set_move_pwm_duty(-HOMING_DIR * ((int32_t) current_config.move_base + current_config.homing_backoff));

        /* 计算退开 ticks（以 ms）: 退开时间由 HAL_GetTick 统计更稳健 */
        if (homing_backoff_start_ms == 0)
            homing_backoff_start_ms = HAL_GetTick();
        if ((HAL_GetTick() - homing_backoff_start_ms) >= HOMING_BACKOFF_MS) {
            homing_backoff_start_ms = 0;
            home_switch_flag = 0;
            homing_state = HOMING_FINE;
            /* 重置去抖状态，准备细靠检测 */
            switch_debounce_counter = 0;
            debounce_done = 0;
            last_switch_state = HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin);
        }
        break;
    }

    case HOMING_FINE: {
        /* 细靠：慢速靠近直到触发或超时。
           [修改] 检测到开关按下（低电平）【立即】归零，不去抖（homing 优先级高、立刻响应），
           消除「去抖 10ms 期间电机惯性滑行」导致的零点漂移；去抖只用于确认开关稳定、结束 homing。 */
        if (HAL_GPIO_ReadPin(home_switch_GPIO_Port, home_switch_Pin) == GPIO_PIN_RESET) {
            if (!fine_zero_latched) {
                fine_zero_latched = 1;
                fine_reached = 1; /* 标记已归零过 */
                set_move_pwm_duty(0);
                Move_Control_Reset(); /* 触发瞬间归零：先 Reset（内部清零+同步 last_cnt）再设目标 */
                move_encoder_ext_count = HOME_FINAL_OFFSET;
                move_target_position = HOME_FINAL_OFFSET;
            }
        } else {
            if (fine_reached) {
                /* [修复] 已归零过、弹跳恢复高：停 PWM 不顶开关（避免把开关越顶越深、零点漂移），
                   清锁存等下次按下重新归零；若开关真离开（非弹跳），细靠超时兜底 FAILED */
                fine_zero_latched = 0;
                set_move_pwm_duty(0);
            } else {
                /* 未归零：正常细靠，正向驱动靠近开关 */
                set_move_pwm_duty(HOMING_DIR * ((int32_t) current_config.move_base + current_config.homing_fine));
            }
        }
        if (home_switch_flag) {
        	home_switch_flag = 0;
        	/* 去抖确认开关稳定按下，结束 homing（零点已在触发瞬间归零，这里不再 Reset） */
        	set_move_pwm_duty(0);
        	homing_active = 0;
        	homing_state = HOMING_DONE;
        	notes_recalc_request = 1; /* [修复] 不在 1ms 中断里直接 Notes_Init（软浮点挤爆周期），置标志由主循环执行 */
        } else {
            /* 细靠超时保护（使用 HAL_GetTick） */
            if (homing_fine_start_ms == 0)
                homing_fine_start_ms = HAL_GetTick();
            if ((HAL_GetTick() - homing_fine_start_ms)
                    >= (uint32_t) (HOMING_FINE_TIMEOUT_S * 1000.0f)) {
                homing_fine_start_ms = 0;
                /* [修改] 超时标记失败 */
                set_move_pwm_duty(0);
                homing_active = 0;
                homing_state = HOMING_FAILED;
            }
        }
        break;
    }

    case HOMING_DONE:
    default: {
        set_move_pwm_duty(0);
        homing_active = 0;
        break;
    }
    }

    return 1; /* homing was active / processed */
}
