/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "notes.h"
#include "mcode.h"
#include "homing.h"
#include "handle.h"
#include "system.h"

/* [修改] 累计值空闲保存阈值（调大避免刷屏）：打孔每满 500 次 或 移动每满 5000mm（mm×100=500000）才写 Flash 一次 */
#define PUNCH_SAVE_THRESHOLD  500UL
#define MOVE_SAVE_THRESHOLD   500000UL
#include "flash.h"
#include "w25q32.h"
#include "fileio.h"
#include "autotune.h" /* [新增] 自整定状态机推进（Autotune_Service） */
#include "progress.h"
#include <string.h>  /* [新增] memcpy 串口缓冲拷贝到本地 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* [修改] M80 交互结束回调：请求回零（延后到动作空闲时执行，不打断正在进行中的动作�?? */
void on_batch_end(void) {
	/* [修改] homing 进行中时不重复请求回零（避免当前回零结束后又回零�??次） */
	if (!Homing_IsActive()) {
		homing_request = 1;
	}
	save_config_request = 1; /* [修改] 交互结束保存累计打孔数，掉电不丢（避免只在达到阈值清零时才写 Flash�?? */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* [修改] 累计值空闲阈值保存的基准（动作空闲时，打孔/移动变化达阈值才写 Flash） */
  static uint32_t last_saved_punch = 0;
  static uint32_t last_saved_move = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	//adc手柄采样
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, 2);

	//移动电机通道
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//进纸电机通道
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	//移动电机编码�??
	__HAL_TIM_SET_COUNTER(&htim2, 32500);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	/* [fix] 上电冲一下：TIM2 上电 CNT=32500，而 last_cnt_move 初始 0，
	   首次 TIM2_UpdateEncoderExtension 会把 move_encoder_ext_count 跳到 32500（pos=32500），
	   此时 move_target_position 被软件限位钳到 0 → err=-32500，电机被驱动冲一下。
	   这里同步 last_cnt_move + 清零 ext_count，让启动 pos=0、err=0，电机不动。 */
	Move_Control_Reset();

	//进纸电机编码�??
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	//�??tim1中断
	HAL_TIM_Base_Start_IT(&htim1);

	//面板按键
	panel_button_init();
	//手柄按键
	handle_button_init();



	//小蝌蚪找妈妈
	Home_Init();


	//注册回调函数
	MCodeSimple_Init(Notes_MoveToIndex, Paper_Feed, Punch_Start, system_command,
	NULL,
	on_batch_end); /* [修改] M80 交互结束 �?? 请求回零 */

////////////////////////////////
	/* [W25Q32] 初始�? + 未焊�?测（Storage_Load 双冗余读�?�? w25q32_present�? */
	W25Q32_Init();
	LoadSystemConfig();
	Motor_LoadPIDFromConfig(); /* [新增] 上电从配置加载 PID 参数到 4 个控制器 */
	Encoder_SetFilter(current_config.encoder_filter); /* [新增] 应用编码器数字滤波（S87，默认 15） */
	Notes_SetSwMaxMM100(current_config.software_max); /* [新增] 初始化软件限位脉冲缓存 */
	/* [修复] 恢复 PVD 掉电保存的打孔/移动计数器：
	   页 61 是最后写入的快照（PVD 掉电 / M80 交互结束 / 回零清零时写），比页 63/W25Q32 更新，
	   因此用 last-write-wins 直接覆盖，不用 max()。原 max() 对 punch_total 是错的——
	   punch_total 每满 home_per_points 回零（非单调），回零瞬间掉电时页 61=0、页 63=旧大值，
	   max(0, 旧大值) 会错误恢复到旧大值，导致提前触发 homing、计数错位。 */
	{
		uint32_t saved_punch = 0, saved_move = 0;
		if (Counters_Load(&saved_punch, &saved_move)) {
			current_config.punch_total = saved_punch;
			current_config.move_total = saved_move;
		}
		/* 页 61 空（从未写过）→ 沿用页 63/W25Q32 已加载的值 */
	}
	System_ApplyNoteConfig(); /* [新增] 应用音数+间距配置到 notes，初始化定位 */
	Paper_Set_RollerDiameter_mm(current_config.tick_mul);
	/* [离线] 初始化文件交�? + 断电进度 */
	FileIO_Init();
	Progress_Init();
	Progress_PVDConfig();
	showHelpInfo(); /* [修改] 移到配置加载之后，打印的当前音数才是实际�? */
	/* [修复] 清零上电编码器毛刺累积的脉冲：TIM2 上电瞬间 A/B 相噪声会让 CNT 跳到
	   约 32768（=1.15m），被 TIM2_UpdateEncoderExtension 当成移动距离累加进 move_total。
	   这里在自检/标定（homing）前丢弃，避免每次上电累计行程虚增 1m 多。 */
	move_total_pulses = 0;
	if (current_config.calibrated == 0) {
		FactoryCalib_Start(); /* [修改] 首次烧录（未标定）：出厂标定（标软件限位 + 主轴/进纸死区�?? */
	} else {
		SelfTest_Start(); /* 已标定：�??机自�??（homing �?? 到软件限�?? �?? 回中�?? */
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* [修复] 音数/间距重算：中断只置标志，主循环执行 Notes_Init（软浮点避免挤爆 1ms 周期） */
		if (notes_recalc_request) {
			notes_recalc_request = 0;
			Notes_Init((float) current_config.home_bias / 100.0f);
		}

		/* [新增] 打印中断里设置的动态文本（SelfTest/FactoryCalib 进度提示），
		   与主循环其它打印统一在这里串行输出，彻底避免并发覆盖 buffer 乱码 */
		if (deferred_print_text) {
			const char *txt = (const char *) deferred_print_text;
			deferred_print_text = NULL;
			USART_printf(&huart1, "%s\n", txt);
		}

		/* [修复] 协议 OK 延迟发送：fileio_ok_cb 在 1ms 中断里只置 deferred_ok，
		   这里在主循环串行发 "OK\n"，不与 USART_printf 的 tx_busy 并发，ACK 不丢 */
		if (deferred_ok) {
			deferred_ok = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*)"OK\n", 3, 0xffffffff);
		}

		/* [autotune] 自整定状态机推进（回零/移动/计算/应用/回传） */
		Autotune_Service();

		/* [修改] S 命令长文本在主循环打印，避免 1ms 中断里阻塞串口发送 */
		if (print_request == PRINT_HELP) {
			print_request = PRINT_NONE;
			showHelpInfo();
		} else if (print_request == PRINT_CONFIG) {
			print_request = PRINT_NONE;
			PrintSystemConfig();
		} else if (print_request == PRINT_W25TEST) {
			/* [存储器] 主循环执行自测，避免 1ms 中断里擦写 Flash 阻塞控制 */
			print_request = PRINT_NONE;
			W25Q32_Init();
			W25Q32_SelfTest();
		} else if (print_request == PRINT_W25ERASE) {
			/* [存储器] 主循环执行全片擦除（约 40s，清空配置/离线文件） */
			print_request = PRINT_NONE;
			W25Q32_Init();
			W25Q32_EraseChip();
			/* [新增] 擦存储器时同步清内部 Flash 的计数器页 + 进度页（用户期望「擦掉存储器=清零」） */
			Counters_Clear();
			Progress_Clear();
			current_config.punch_total = 0;
			current_config.move_total = 0;
			/* [修复] 把清零后的配置写回页 63 + W25Q32：否则重启后 Storage_Load 从页 63
			   读到旧的 punch_total/move_total 复活（S31 只擦了 W25Q32/页61/页62，没擦页 63） */
			SaveSystemConfig();
		} else if (print_request == PRINT_TRAVEL_WARN) {
			/* [新增] 行程不足警告：音数×间距超可用行程，提示用户减小间距或增大行程 */
			print_request = PRINT_NONE;
			PrintTravelWarning();
		}

		/* [离线] 推进执行/回读状态机 */
		FileIO_Service();

		/* [新增] 累计移动距离：把编码器中断累加的 |脉冲| 转 mm*100 累加到 move_total */
		Move_AccumulateDistance();

		/* [修改] 空闲阈值保存（阈值已调大：打孔 500 次 / 移动 500mm，避免刷屏）：
		   动作空闲 + 打孔/移动变化达阈值才写 Flash，作为 PVD 掉电之外的正常断电兜底 */
		if (move_busy == 0 && feed_busy == 0 && punch_busy == 0) {
			uint32_t dp = (current_config.punch_total >= last_saved_punch)
					? (current_config.punch_total - last_saved_punch)
					: (last_saved_punch - current_config.punch_total);
			uint32_t dm = (current_config.move_total >= last_saved_move)
					? (current_config.move_total - last_saved_move)
					: (last_saved_move - current_config.move_total);
			if (dp >= PUNCH_SAVE_THRESHOLD || dm >= MOVE_SAVE_THRESHOLD) {
				last_saved_punch = current_config.punch_total;
				last_saved_move = current_config.move_total;
				save_config_request = 1;
				/* [新增] 同步写计数器环形日志（页 61，逐字编程，掉电恢复用） */
				Counters_Save(current_config.punch_total, current_config.move_total);
			}
		}

		/* [修改] 主循环保存配置请求：累计打孔数清零后掉电保存（避免在 1ms 中断里擦写 Flash） */
		if (save_config_request) {
			save_config_request = 0;
			SaveSystemConfig();
			/* [修复] 同步写计数器页 61，让页 61 与 W25Q32/页 63 的 punch_total/move_total 一致。
			   否则 punch_total 回零后，SaveSystemConfig 已写 0，但页 61 仍是旧大值，
			   上电 max() 恢复会取到旧大值（回零丢失，偏差最多 home_per_points-1） */
			Counters_Save(current_config.punch_total, current_config.move_total);
		}

		if (uart_ok == 1) {
			/* [修改] 逐个消费就绪队列，主循环�? Flash 保存阻塞期间连收两批也不�? */
			while (1) {
				uint8_t idx;
				uint8_t line_copy[UART_RX_BUF_SIZE + 1];
				__disable_irq();
				if (uart_rx_ready_cnt == 0) {
					uart_ok = 0; /* [修复] 临界区内清，防 ISR 在 break 后入队却标志被覆盖滞留一帧 */
					__enable_irq();
					break;
				}
				idx = uart_rx_ready[uart_rx_ready_head];
				uart_rx_ready_head = (uart_rx_ready_head + 1) & 1;
				uart_rx_ready_cnt--;
				/* [修复] 临界区内拷贝到本地，出队后缓冲立即可复用，
				   DMA 只需避开就绪队列即可，不再占用 consuming 槽位（避免无空闲缓冲竞态） */
				memcpy(line_copy, uart_rx_buf[idx], UART_RX_BUF_SIZE + 1);
				__enable_irq();
				/* [离线] 先给 fileio 识别 UPLOAD/END/OK/上传数据；未处理再走 MCode */
				if (FileIO_ProcessLine((char*) line_copy) == 0) {
					MCodeSimple_ProcessBuffer((char*) line_copy);
				}
			}
		}

//		USART_printf(&huart1, "%d,%d\n", paper_target_position, TIM4->CNT);
//				punch_busy);
//		USART_printf(&huart1, "%d,%d\n",Simple_Filter_GetCh1(),Simple_Filter_GetCh2());

//		USART_printf(&huart1, "%d\n",temppp);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
