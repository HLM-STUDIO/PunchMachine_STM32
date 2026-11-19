/*
 * handle.h
 *
 *  Created on: 2026年1月23日
 *      Author: huang
 */

#ifndef INC_HANDLE_H_
#define INC_HANDLE_H_

#include <stdint.h>
#include <string.h>



extern volatile uint8_t handle_enable;

uint16_t Simple_Filter_GetCh1(void);

uint16_t Simple_Filter_GetCh2(void);
void Simple_Filter_Update(uint16_t ch1, uint16_t ch2);

void Handle_Move_Task();

void set_handle_mode();
void reset_handle_mode();
void handle_button_init();
void panel_button_init();
#endif /* INC_HANDLE_H_ */
