/*
 * w25q32.h
 * 外挂 SPI NOR Flash（W25Q32 4MB）驱动
 *
 * SPI1 重映射（由 CubeMX 生成 spi.c 配置，见 MX_SPI1_Init）：
 *   PB3 = SCK, PB4 = MISO, PB5 = MOSI, PA15 = 软件片选（本模块 GPIO 控制）
 *
 * 注意：W25Q32 的读/写/擦除命令要求 CS 在「命令 + 地址 + 数据」全程保持低电平，
 * 所以必须用软件片选（手动拉低/拉高包住整个命令），不能用硬件 NSS。
 */

#ifndef INC_W25Q32_H_
#define INC_W25Q32_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ---- W25Q32 命令码 ---- */
#define W25Q_CMD_WRITE_ENABLE   0x06  /* 写使能 */
#define W25Q_CMD_WRITE_DISABLE  0x04  /* 写禁止 */
#define W25Q_CMD_READ_STATUS    0x05  /* 读状态寄存器（bit0 = BUSY） */
#define W25Q_CMD_READ_ID        0x9F  /* 读 JEDEC ID（W25Q32 应为 EF 40 16） */
#define W25Q_CMD_READ_DATA      0x03  /* 读数据 */
#define W25Q_CMD_PAGE_PROGRAM   0x02  /* 页编程（256 字节/页，不能跨页） */
#define W25Q_CMD_SECTOR_ERASE   0x20  /* 扇区擦除（4KB） */
#define W25Q_CMD_CHIP_ERASE     0xC7  /* 全片擦除（4MB，耗时约 40s，慎用） */

/* ---- 引脚定义 ---- */
#define W25Q_CS_PIN      GPIO_PIN_15
#define W25Q_CS_PORT     GPIOA

#define W25Q_PAGE_SIZE   256   /* 页编程单位 */
#define W25Q_SECTOR_SIZE 4096  /* 扇区擦除单位 */

/* ---- 存储布局（见设计文档 3.1） ---- */
#define W25Q_ADDR_SYSVAR      0x000000UL  /* 系统变量区：3 × 4KB 扇区（副本 A/B/C） */
#define W25Q_ADDR_SYSVAR_SIZE 0x003000UL  /* 系统变量区总大小 12KB */
#define W25Q_ADDR_FILE_DIR    0x200000UL  /* 文件目录（4KB） */
#define W25Q_ADDR_FILE_DATA   0x201000UL  /* 文件数据区（约 2MB） */

/* ---- 容量代码（JEDEC ID 第 3 字节，W25Q 系列通用） ---- */
#define W25Q_CAP_8MBIT   0x14  /* W25Q80  1MB  */
#define W25Q_CAP_16MBIT  0x15  /* W25Q16  2MB  */
#define W25Q_CAP_32MBIT  0x16  /* W25Q32  4MB  */
#define W25Q_CAP_64MBIT  0x17  /* W25Q64  8MB  */

/* ---- 未焊检测 ---- */
extern uint8_t w25q32_present;   /* 0=未焊/未识别，1=已识别 OK */
extern uint32_t w25q_capacity;   /* 芯片容量（字节），自动识别：1/2/4/8 MB */

/* ---- 接口 ---- */
int      W25Q32_Init(void);       /* 初始化 PA15 片选 + 检测芯片（置 w25q32_present + 容量） */
uint32_t W25Q32_ReadID(void);     /* 读 JEDEC ID（低 24 位，W25Q32 = 0xEF4016） */
void     W25Q32_Read(uint32_t addr, uint8_t *buf, uint32_t len);      /* 任意地址任意长度读 */
void     W25Q32_Write(uint32_t addr, const uint8_t *buf, uint32_t len); /* 跨页写（写前需已擦除） */
void     W25Q32_EraseSector(uint32_t addr);   /* 擦 4KB 扇区 */
void     W25Q32_EraseChip(void);              /* 全片擦（约 40s） */
void     W25Q32_SelfTest(void);               /* 自测：读ID → 擦扇区0 → 写 → 读回对比 */

#endif /* INC_W25Q32_H_ */
