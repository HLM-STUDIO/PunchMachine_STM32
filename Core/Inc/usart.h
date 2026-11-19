/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define UART_RX_BUF_SIZE 256  // 接收缓冲区大�?

extern uint8_t uart_rx_buf[3][UART_RX_BUF_SIZE + 1];  // [修复] 三缓冲 +1 字节给 '\0'，防 Size==256 覆盖最后 1 字节
extern volatile uint8_t uart_rx_ready[2];  // [修改] 就绪缓冲索引队列
extern volatile uint8_t uart_rx_ready_cnt;  // [修改] 就绪数量
extern volatile uint8_t uart_rx_ready_head;  // [修改] 队列�?
extern volatile uint8_t uart_rx_ready_tail;  // [修改] 队列�?
extern volatile uint8_t uart_ok;//接收完成标志
extern volatile uint8_t deferred_ok;// [修复] 协议 OK 延迟到主循环发送（fileio_ok_cb 置位，主循环发 "OK\n"，避免 tx_busy 并发跳过丢 ACK）

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART_printf(UART_HandleTypeDef *huart,char *fmt, ...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

