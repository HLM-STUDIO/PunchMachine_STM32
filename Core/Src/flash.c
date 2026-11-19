/*
 * flash.c
 *
 *  Created on: 2026年1月25日
 *      Author: huang
 */


#include "flash.h"

// 内部函数声明
static uint32_t Flash_GetPageFromAddress(uint32_t addr);
static uint32_t Flash_GetAddressFromPage(uint32_t page);

/**
  * @brief  获取地址对应的页号
  * @param  addr: 地址
  * @retval 页号，若地址不在 FLASH 范围内则返回 0xFFFFFFFF
  */
static uint32_t Flash_GetPageFromAddress(uint32_t addr)
{
    uint32_t flash_end = FLASH_BASE_ADDR + FLASH_TOTAL_PAGES * FLASH_PAGE_SIZE;
    if (addr < FLASH_BASE_ADDR || addr >= flash_end)
        return 0xFFFFFFFFU;

    return (addr - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
}

/**
  * @brief  获取页号对应的起始地址
  * @param  page: 页号
  * @retval 起始地址，若页号越界返回 0xFFFFFFFF
  */
static uint32_t Flash_GetAddressFromPage(uint32_t page)
{
    if (page >= FLASH_TOTAL_PAGES)
        return 0xFFFFFFFFU;

    return FLASH_BASE_ADDR + page * FLASH_PAGE_SIZE;
}

/**
  * @brief  HAL库版写一个uint32_t类型的数据（会擦除该页）
  * @param  addr: 存储数据的地址（必须 4 字节对齐）
  * @param  data: 写入的数据
  * @retval 成功返回0， 失败返回-1
  */
int Flash_HAL_Write_Data(uint32_t addr, uint32_t data)
{
    // 地址对齐检查（可选，但强烈建议）
    if (addr % 4U != 0U) return -1;

    // 1、FLASH解锁
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    // 2、获取地址对应的页号
    uint32_t page = Flash_GetPageFromAddress(addr);
    if (page == 0xFFFFFFFFU)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 3、擦除该页（这里擦除整页）
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = Flash_GetAddressFromPage(page);
    EraseInitStruct.NbPages = 1U;  // 只擦除1页

    uint32_t PageError = 0U;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 4、写入一个 32-bit 单元
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)data) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 5、FLASH上锁
    HAL_FLASH_Lock();
    return 0;
}

/**
  * @brief  HAL库版写N个uint32_t类型的数据（会擦除所有涉及页）
  * @param  addr: 存储数据的起始地址（必须 4 字节对齐）
  * @param  data: uint32_t 数据数组
  * @param  num: 数据个数
  * @retval 成功返回0， 失败返回-1
  */
int Flash_HAL_Write_N_Data(uint32_t addr, uint32_t *data, uint16_t num)
{
    if (addr % 4U != 0U) return -1;
    if (data == NULL || num == 0U) return -1;

    // 1、FLASH解锁
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    // 2、计算需要擦除的页数
    uint32_t start_page = Flash_GetPageFromAddress(addr);
    uint32_t end_addr = addr + (uint32_t)num * sizeof(uint32_t);
    uint32_t end_page = Flash_GetPageFromAddress(end_addr - 1U);

    if (start_page == 0xFFFFFFFFU || end_page == 0xFFFFFFFFU)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    uint32_t pages_to_erase = end_page - start_page + 1U;

    // 3、FLASH 擦除
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = Flash_GetAddressFromPage(start_page);
    EraseInitStruct.NbPages = pages_to_erase;

    uint32_t PageError = 0U;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    // 4、FLASH 写入（逐个 32-bit 写）
    uint32_t current_addr = addr;
    for (uint16_t i = 0U; i < num; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_addr, (uint64_t)data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
        current_addr += sizeof(uint32_t);
    }

    // 5、FLASH 上锁
    HAL_FLASH_Lock();
    return 0;
}

/**
  * @brief  HAL库版读取N个uint32_t类型的数据
  * @param  addr: 读取数据的地址
  * @param  data: 输出数据数组（uint32_t）
  * @param  num: 数据的个数
  * @retval NONE
  */
void Flash_HAL_Read_N_Data(uint32_t addr, uint32_t *data, uint32_t num)
{
    if (data == NULL) return;

    for (uint32_t i = 0U; i < num; i++)
    {
        data[i] = *(volatile uint32_t*)addr;
        addr += sizeof(uint32_t);
    }
}

/**
  * @brief  HAL库版读取N个uint8_t类型的数据（按字节读取）
  * @param  addr: 读取数据的地址
  * @param  data: 输出数据数组（uint8_t）
  * @param  num: 字节个数
  * @retval NONE
  */
void Flash_HAL_Read_N_Byte(uint32_t addr, uint8_t *data, uint32_t num)
{
    if (data == NULL) return;

    for (uint32_t i = 0U; i < num; i++)
    {
        data[i] = *(volatile uint8_t*)addr;
        addr += 1U;
    }
}
