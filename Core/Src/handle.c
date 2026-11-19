/*
 * handle.c
 *
 *  Created on: 2026年1月23日
 *      Author: huang
 */
#include "handle.h"
#include "adc.h"
#include "motor.h"
#include "button.h"
#include "usart.h"
#include "homing.h"
#include "notes.h"
#include "system.h" /* [新增] 手柄限位用 current_config.software_max */
#include "main.h" /* [新增] 手柄限位用 home_switch GPIO */
#include "mcode.h" /* [新增] 手柄互锁用 punch_busy */

volatile uint8_t handle_enable = 0; //手柄模式

//符合坐标轴象限
static Button btn_panel_1;
static Button btn_panel_2;
static Button btn_panel_3;
static Button btn_panel_4;

static Button btn_handle;
/* 内部状态（静态文件作用域） */
/* 超简单的滑动平均滤波 */
#define FILTER_SIZE 64
#define ADC_MAX 4095

static uint16_t filter_buf1[FILTER_SIZE];
static uint16_t filter_buf2[FILTER_SIZE];
static uint8_t filter_idx = 0;
static uint32_t filter_sum1 = 0;
static uint32_t filter_sum2 = 0;
static uint8_t filter_init_count = 0;

/* 1ms中断中调用 */
void Simple_Filter_Update(uint16_t ch1, uint16_t ch2) {
	uint16_t oldest1, oldest2;

	/* 限制输入范围 */
	if (ch1 > ADC_MAX)
		ch1 = ADC_MAX;
	if (ch2 > ADC_MAX)
		ch2 = ADC_MAX;

	/* 获取将被替换的旧值 */
	oldest1 = filter_buf1[filter_idx];
	oldest2 = filter_buf2[filter_idx];

	/* 存储新值 */
	filter_buf1[filter_idx] = ch1;
	filter_buf2[filter_idx] = ch2;

	/* 更新索引 */
	filter_idx = (filter_idx + 1) % FILTER_SIZE;

	/* 更新总和 */
	if (filter_init_count < FILTER_SIZE) {
		/* 初始填充阶段 */
		filter_sum1 += ch1;
		filter_sum2 += ch2;
		filter_init_count++;
	} else {
		/* 正常滑动阶段 */
		filter_sum1 = filter_sum1 - oldest1 + ch1;
		filter_sum2 = filter_sum2 - oldest2 + ch2;
	}
}

/* 获取滤波后的值 */
uint16_t Simple_Filter_GetCh1(void) {
	if (filter_init_count == 0)
		return 0;
	return (uint16_t) (filter_sum1 / filter_init_count);
}

uint16_t Simple_Filter_GetCh2(void) {
	if (filter_init_count == 0)
		return 0;
	return (uint16_t) (filter_sum2 / filter_init_count);
}

//输入值分别是：输入值，输入最小值，输入最大值，输出最小值，输出最大值
float mapValue(float value, float minIn, float maxIn, float minOut,
		float maxOut) {
	return (value - minIn) * ((maxOut - minOut) / (maxIn - minIn)) + minOut;
}

uint8_t read_handle_button(uint8_t button_id) {
	(void) button_id;
	return HAL_GPIO_ReadPin(handle_button_GPIO_Port, handle_button_Pin);
}

uint8_t read_panel_button(uint8_t button_id) {

	switch (button_id) {
	case 1:
		return HAL_GPIO_ReadPin(button1_GPIO_Port, button1_Pin);
	case 2:
		return HAL_GPIO_ReadPin(button2_GPIO_Port, button2_Pin);
	case 3:
		return HAL_GPIO_ReadPin(button3_GPIO_Port, button3_Pin);
	case 4:
		return HAL_GPIO_ReadPin(button4_GPIO_Port, button4_Pin);
	default:
		return 0;
	}
}

void handle1_single_click_handler(Button *btn_handle) {
	(void) btn_handle;
	USART_printf(&huart1, "%s", "handle 1: Single Click\n");

}
void handle1_double_click_handler(Button *btn_handle) {
	(void) btn_handle;
	USART_printf(&huart1, "%s", "handle 1: Double Click\n");

}

void handle1_long_handler(Button *btn_handle) {
	(void) btn_handle;
	static uint8_t count = 0;
	if (count++ % 2 == 0) {
		set_handle_mode();   /* [修改] 统一走 set_handle_mode，进入时停电机 */
	} else {
		reset_handle_mode(); /* [修改] 统一走 reset_handle_mode，退出时同步位置 */
	}
	USART_printf(&huart1, "%s", "handle 1: long\n");

}

void btn1_single_click_handler(Button *btn_handle) {
	(void) btn_handle;
	USART_printf(&huart1, "%s", "Button 1: Single Click\n");

}
void btn2_single_click_handler(Button *btn_handle) {
	(void) btn_handle;
	USART_printf(&huart1, "%s", "Button 2: Single Click\n");

}
void btn3_single_click_handler(Button *btn_handle) {
	(void) btn_handle;
	USART_printf(&huart1, "%s", "Button 3: Single Click\n");
	Notes_MoveToIndex(1);
}
void btn4_single_click_handler(Button *btn_handle) {
	(void) btn_handle;
	/* [修复] 不走 Home_Start() 直启回零，改为置 homing_request：
	   由 mcode.c 的安全入口在动作空闲时执行，避免 move/feed/punch 进行中被强行打断 */
	homing_request = 1;
	USART_printf(&huart1, "%s", "Button 4: Single Click\n");

}

void handle_button_init() {
	button_init(&btn_handle, read_handle_button, 0, 5);
	button_attach(&btn_handle, BTN_SINGLE_CLICK, handle1_single_click_handler);
	button_attach(&btn_handle, BTN_DOUBLE_CLICK, handle1_double_click_handler);
	button_attach(&btn_handle, BTN_LONG_PRESS_START, handle1_long_handler);

	button_start(&btn_handle);
}

#define HANDLE_OFFSET  300 //死区让开
#define handleXCenter 2048
#define handleYCenter 2048

void Handle_Move_Task() {

	/* [修复] 打孔互锁：打孔进行中禁止手柄驱动两轴（否则打孔针下降时纸/主轴被移动，
	   孔位错位/卡纸安全隐患）。与闭环 Move_Control_Task/Paper_Control_Task 的互锁对齐。 */
	if (punch_busy) {
		set_move_pwm_duty(0);
		set_paper_pwm_duty(0);
		return;
	}

	uint16_t handleX = Simple_Filter_GetCh1();
	uint16_t handleY = Simple_Filter_GetCh2();

	/*          Move begin         */
	/* [新增] 手柄限位：读当前位置 + 软件限位 + homing 开关状态，防止顶开关/超行程 */
	int32_t move_pos = read_move_encoder_counter();
	int32_t sw_max_pulses = Notes_ConvertMMToPulses(
			(float) current_config.software_max / 100.0f);
	uint8_t at_home_switch = (HAL_GPIO_ReadPin(home_switch_GPIO_Port,
			home_switch_Pin) == GPIO_PIN_RESET);

	if (handleY < handleYCenter + HANDLE_OFFSET
			&& handleY > handleYCenter - HANDLE_OFFSET) {
		set_move_pwm_duty(0);
	}

	if (handleY >= handleYCenter + HANDLE_OFFSET) {
		/* 负方向（远离 home，往 software_max）：到软件限位停 */
		/* [修改] 软件限位在 home 负方向，判断 move_pos <= -sw_max_pulses */
		if (move_pos <= -sw_max_pulses) {
			set_move_pwm_duty(0);
		} else {
			set_move_pwm_duty(mapValue(handleY, handleYCenter + HANDLE_OFFSET, 4095,
					PWM_MAX * -0.1, PWM_MAX * -0.5));
		}
	}

	if (handleY <= handleYCenter - HANDLE_OFFSET) {
		/* 正方向（往 home 开关）：触发开关停 */
		if (at_home_switch) {
			set_move_pwm_duty(0);
		} else {
			set_move_pwm_duty(mapValue(handleY, handleYCenter - HANDLE_OFFSET, 0,
				PWM_MAX * 0.1, PWM_MAX * 0.5));
		}
	}
///////////////////////////////////
	if (handleX > handleXCenter - HANDLE_OFFSET
			&& handleX < handleXCenter + HANDLE_OFFSET) {
		set_paper_pwm_duty(0);
	}

	if (handleX <= handleXCenter - HANDLE_OFFSET) {
		/* [修复] 进纸方向反了：X 往左改为正 PWM */
		set_paper_pwm_duty(mapValue(handleX, handleXCenter - HANDLE_OFFSET, 0,
				PWM_MAX * 0.1, PWM_MAX));
	}

	if (handleX >= handleXCenter + HANDLE_OFFSET) {
		/* [修复] 进纸方向反了：X 往右改为负 PWM */
		set_paper_pwm_duty(mapValue(handleX, handleXCenter + HANDLE_OFFSET, 4095,
				PWM_MAX * -0.1, -PWM_MAX));
	}

}

void panel_button_init() {
	button_init(&btn_panel_1, read_panel_button, 0, 1);
	button_init(&btn_panel_2, read_panel_button, 0, 2);
	button_init(&btn_panel_3, read_panel_button, 0, 3);
	button_init(&btn_panel_4, read_panel_button, 0, 4);
	button_attach(&btn_panel_1, BTN_SINGLE_CLICK, btn1_single_click_handler);
	button_attach(&btn_panel_2, BTN_SINGLE_CLICK, btn2_single_click_handler);
	button_attach(&btn_panel_3, BTN_SINGLE_CLICK, btn3_single_click_handler);
	button_attach(&btn_panel_4, BTN_SINGLE_CLICK, btn4_single_click_handler);

	button_start(&btn_panel_1);
	button_start(&btn_panel_2);
	button_start(&btn_panel_3);
	button_start(&btn_panel_4);

}

void set_handle_mode() {
	handle_enable = 1;
	/* [修改] 进入手动模式时停电机，避免上一个闭环动作的惯性带着走 */
	set_move_pwm_duty(0);
	set_paper_pwm_duty(0);
	/* [修复] 清 paper_active：进纸命令进行中切入手动模式时，若不清，退出手动同步位置后
	   err=0 会立刻命中 done 分支误发 NotifyFeedDone（把「打断进纸」误判成「进纸完成」） */
	paper_active = 0;
	feed_busy = 0; /* [修复] 进纸打断后清 feed_busy，否则 homing_request 永远无法执行、含 Y 的 pending 卡死 */
}
void reset_handle_mode() {
	handle_enable = 0;
	/* [修改] 退出手动模式：
	   - move 轴保持原打孔目标不动 → 闭环会驱动电机自动回到打孔位置（不在原地打孔）
	   - paper 轴同步到当前位置 → 纸张停在手动调整的位置，不退回 */
	paper_target_position = read_paper_encoder_counter();
}
