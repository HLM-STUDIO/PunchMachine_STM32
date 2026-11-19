/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define button1_Pin GPIO_PIN_14
#define button1_GPIO_Port GPIOC
#define button2_Pin GPIO_PIN_15
#define button2_GPIO_Port GPIOC
#define move_encoder_a_Pin GPIO_PIN_0
#define move_encoder_a_GPIO_Port GPIOA
#define move_encoder_b_Pin GPIO_PIN_1
#define move_encoder_b_GPIO_Port GPIOA
#define handle_x_Pin GPIO_PIN_2
#define handle_x_GPIO_Port GPIOA
#define handle_y_Pin GPIO_PIN_3
#define handle_y_GPIO_Port GPIOA
#define move_a_Pin GPIO_PIN_6
#define move_a_GPIO_Port GPIOA
#define move_b_Pin GPIO_PIN_7
#define move_b_GPIO_Port GPIOA
#define paper_a_Pin GPIO_PIN_0
#define paper_a_GPIO_Port GPIOB
#define paper_b_Pin GPIO_PIN_1
#define paper_b_GPIO_Port GPIOB
#define temp_test_Pin GPIO_PIN_10
#define temp_test_GPIO_Port GPIOB
#define punch_b_Pin GPIO_PIN_12
#define punch_b_GPIO_Port GPIOB
#define punch_a_Pin GPIO_PIN_13
#define punch_a_GPIO_Port GPIOB
#define button3_Pin GPIO_PIN_14
#define button3_GPIO_Port GPIOB
#define button4_Pin GPIO_PIN_15
#define button4_GPIO_Port GPIOB
#define handle_button_Pin GPIO_PIN_8
#define handle_button_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define paper_encoder_b_Pin GPIO_PIN_6
#define paper_encoder_b_GPIO_Port GPIOB
#define paper_encoder_a_Pin GPIO_PIN_7
#define paper_encoder_a_GPIO_Port GPIOB
#define punch_swich_Pin GPIO_PIN_8
#define punch_swich_GPIO_Port GPIOB
#define punch_swich_EXTI_IRQn EXTI9_5_IRQn
#define home_switch_Pin GPIO_PIN_9
#define home_switch_GPIO_Port GPIOB
#define home_switch_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
