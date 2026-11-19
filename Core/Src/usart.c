/* USER CODE BEGIN Header */
/*
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "mcode.h"


uint8_t uart_rx_buf[3][UART_RX_BUF_SIZE + 1];  // [修复] 三缓冲 +1 字节给 '\0'：DMA 永远写空闲的第 3 个，就绪队列最多 2 个
volatile uint8_t uart_rx_dma = 0;          // DMA 当前写入的缓冲索引（0..2）
volatile uint8_t uart_rx_ready[2];         // 就绪缓冲索引队列（最多 2 个未消费）
volatile uint8_t uart_rx_ready_cnt = 0;    // 就绪缓冲数量
volatile uint8_t uart_rx_ready_head = 0;   // 队列头（主循环消费）
volatile uint8_t uart_rx_ready_tail = 0;   // 队列尾（中断写入）
volatile uint8_t uart_ok = 0; /* [修复] ISR 写、主循环读，加 volatile */
volatile uint8_t deferred_ok = 0; /* [修复] 协议 OK 延迟发送标志：fileio_ok_cb 置位，主循环消费发 "OK\n" */
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	// 启动DMA接收（初始写入缓�? 0�?
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf[0], UART_RX_BUF_SIZE)
			!= HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void USART_printf(UART_HandleTypeDef *huart, char *fmt, ...) {
	static char buffer[256];
	/* [新增] 发送互斥锁：主循环 + 1ms 中断可能并发调用 USART_printf，
	   并发时 static buffer 会互相覆盖、HAL_UART_Transmit 字节交错 → 中文乱码/丢字。
	   若正在发送（并发重入），跳过本次打印（偶发丢一条进度提示，可接受，避免乱码）。 */
	static volatile uint8_t tx_busy = 0;
	/* 临界区保护「检查 + 置位」，防止主循环检查后、置位前被中断打断导致双重进入 */
	__disable_irq();
	if (tx_busy) {
		__enable_irq();
		return;
	}
	tx_busy = 1;
	__enable_irq();

	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args); /* [修改] vsprintf→vsnprintf，防长字符串栈溢出 */
	va_end(args);
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer),0xffffffff);

	tx_busy = 0;
}

void VOFA_ParseData(uint8_t *data, uint16_t len) {
	(void) data;
	(void) len;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		// Size 是本次收到的字节数（�? HAL 提供�?
		if (Size == 0) {
			// 没数据，直接重启接收
			HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buf[uart_rx_dma], UART_RX_BUF_SIZE);
			return;
		}

		/* [修复] 缓冲数组多 1 字节（[UART_RX_BUF_SIZE+1]），Size==UART_RX_BUF_SIZE 时写 buf[Size] 不再覆盖最后 1 字节 */
		uart_rx_buf[uart_rx_dma][Size] = '\0';
		uint8_t ready = uart_rx_dma;

		/* 队列满（2 个都没消费）时丢弃最旧，保留最新 */
		if (uart_rx_ready_cnt >= 2) {
			uart_rx_ready_head = (uart_rx_ready_head + 1) & 1;
			uart_rx_ready_cnt--;
		}
		uart_rx_ready[uart_rx_ready_tail] = ready;
		uart_rx_ready_tail = (uart_rx_ready_tail + 1) & 1;
		uart_rx_ready_cnt++;

		/* 找空闲缓冲（3 个里不在就绪队列的那个）作为新的 DMA 写目标 */
		uint8_t next = 0;
		for (uint8_t i = 0; i < 3; i++) {
			uint8_t occupied = 0;
			for (uint8_t j = 0; j < uart_rx_ready_cnt; j++) {
				if (uart_rx_ready[(uart_rx_ready_head + j) & 1] == i) {
					occupied = 1;
					break;
				}
			}
			if (!occupied) {
				next = i;
				break;
			}
		}
		uart_rx_dma = next;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buf[uart_rx_dma], UART_RX_BUF_SIZE);
		uart_ok = 1; // 主循环或其他任务可以�?测这个标�?
	}
}
/* USER CODE END 1 */
