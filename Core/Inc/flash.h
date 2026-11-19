/*
 * flash.h
 *
 *  Created on: 2026年1月25日
 *      Author: huang
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * STM32F103C8T6的FLASH大小为64KB，分为128页，每页512字节
 * 注意：F103系列FLASH页大小根据容量不同而不同：
 * - 小容量产品(16K/32K)：每页1KB
 * - 中容量产品(64K/128K)：每页1KB
 * - 大容量产品(256K/512K)：每页2KB
 * 但实际通过文档查看，64KB的产品每页是1KB
 *
 * 修正：经过实际验证，STM32F103C8T6的FLASH页大小为1KB
 * 这里我们按照64KB容量计算，共64页，每页1KB
 */

// FLASH起始地址
#define FLASH_BASE_ADDR        ((uint32_t)0x08000000)

// 每页大小（1KB = 1024字节）
//#define FLASH_PAGE_SIZE        ((uint32_t)0x400)      // 1024字节

// 总页数（64KB / 1KB = 64页）
#define FLASH_TOTAL_PAGES      ((uint32_t)64)

#define ADDR_FLASH_PAGE_60     (FLASH_BASE_ADDR + 60 * FLASH_PAGE_SIZE)
#define ADDR_FLASH_PAGE_61     (FLASH_BASE_ADDR + 61 * FLASH_PAGE_SIZE)
#define ADDR_FLASH_PAGE_62     (FLASH_BASE_ADDR + 62 * FLASH_PAGE_SIZE)
#define ADDR_FLASH_PAGE_63     (FLASH_BASE_ADDR + 63 * FLASH_PAGE_SIZE)

// 用户Flash区域定义（根据实际应用调整）
// 示例：使用最后1页作为用户数据存储区（第63-63页，共1KB）
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_63   /* 用户Flash区域起始地址 */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE - 1)   /* 用户Flash区域结束地址 */

// 函数声明
int Flash_HAL_Write_Data(uint32_t addr, uint32_t data);
int Flash_HAL_Write_N_Data(uint32_t addr, uint32_t *data, uint16_t num);
void Flash_HAL_Read_N_Data(uint32_t addr, uint32_t *data, uint32_t num);
void Flash_HAL_Read_N_Byte(uint32_t addr, uint8_t *data, uint32_t num);

#endif /* INC_FLASH_H_ */
