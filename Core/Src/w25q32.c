/*
 * w25q32.c
 * 外挂 SPI NOR Flash（W25Q32 4MB）驱动
 *
 * SPI1 由 CubeMX 生成（spi.c 的 MX_SPI1_Init，main.c 已调用），
 * 本模块只负责：PA15 软件片选 + W25Q32 命令 + 未焊检测。
 */

#include "w25q32.h"
#include "spi.h"      /* 复用 CubeMX 生成的 hspi1 */
#include "usart.h"    /* USART_printf + huart1 */
#include "system.h"   /* [新增] current_config / L() 中英文切换 */

uint8_t w25q32_present = 0;
uint32_t w25q_capacity = 0;   /* 芯片容量（字节），0 = 未识别 */

/* ==================== 片选 ==================== */
static void W25Q_CS_Low(void)
{
	HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_RESET);
}
static void W25Q_CS_High(void)
{
	HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
}

/* ==================== 初始化 + 未焊检测 ==================== */
/* PA15 软片选已由 CubeMX 的 MX_GPIO_Init 配成 GPIO 输出（SPI1_NSS 软片选），
   SPI1 由 MX_SPI1_Init 配置；这里只拉高 CS + 读 ID 检测是否焊接。 */
int W25Q32_Init(void)
{
	W25Q_CS_High();

	/* 上电读 JEDEC ID，自动识别 W25Q 系列（厂商 0xEF + 类型 0x40 + 容量 0x14~0x17） */
	uint32_t id = W25Q32_ReadID();
	uint8_t mfr  = (uint8_t) ((id >> 16) & 0xFF);
	uint8_t type = (uint8_t) ((id >> 8)  & 0xFF);
	uint8_t cap  = (uint8_t) (id & 0xFF);

	w25q32_present = 0;
	w25q_capacity  = 0;
	if (mfr == 0xEF && type == 0x40) {
		switch (cap) {
			case W25Q_CAP_8MBIT:  w25q_capacity = 0x100000UL; w25q32_present = 1; break; /* W25Q80  1MB */
			case W25Q_CAP_16MBIT: w25q_capacity = 0x200000UL; w25q32_present = 1; break; /* W25Q16  2MB */
			case W25Q_CAP_32MBIT: w25q_capacity = 0x400000UL; w25q32_present = 1; break; /* W25Q32  4MB */
			case W25Q_CAP_64MBIT: w25q_capacity = 0x800000UL; w25q32_present = 1; break; /* W25Q64  8MB */
			default: break; /* 未知容量，视为未识别 */
		}
	}
	return 0;
}

/* ==================== 底层命令 ==================== */
static void W25Q_WriteEnable(void)
{
	uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
	W25Q_CS_Low();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	W25Q_CS_High();
}

static uint8_t W25Q_ReadStatus(void)
{
	uint8_t cmd = W25Q_CMD_READ_STATUS;
	uint8_t status = 0;
	W25Q_CS_Low();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Receive(&hspi1, &status, 1, 100);
	W25Q_CS_High();
	return status;
}

/* [修复] 加超时参数（ms）：页编程/扇区擦除用短超时(1s)防卡死，
   全片擦除用长超时(60s，W25Q32 全片擦需 40~60s，不能再被 1s 提前放弃) */
static int W25Q_WaitBusy(uint32_t timeout_ms)
{
	while (W25Q_ReadStatus() & 0x01) {
		if (timeout_ms == 0) {
			return -1;
		}
		timeout_ms--;
		HAL_Delay(1);
	}
	return 0;
}

/* 发送「命令 + 24 位地址」头 */
static void W25Q_SendCmdAddr(uint8_t cmd, uint32_t addr)
{
	uint8_t hdr[4] = {
		cmd,
		(uint8_t) ((addr >> 16) & 0xFF),
		(uint8_t) ((addr >> 8) & 0xFF),
		(uint8_t) (addr & 0xFF)
	};
	HAL_SPI_Transmit(&hspi1, hdr, 4, 100);
}

/* ==================== 公开接口 ==================== */
uint32_t W25Q32_ReadID(void)
{
	uint8_t cmd = W25Q_CMD_READ_ID;
	uint8_t id[3] = {0, 0, 0};
	W25Q_CS_Low();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Receive(&hspi1, id, 3, 100);
	W25Q_CS_High();
	return ((uint32_t) id[0] << 16) | ((uint32_t) id[1] << 8) | (uint32_t) id[2];
}

/* 任意地址任意长度读（普通读 0x03，无跨页限制） */
void W25Q32_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
	W25Q_CS_Low();
	W25Q_SendCmdAddr(W25Q_CMD_READ_DATA, addr);
	while (len > 0) {
		uint32_t chunk = len;
		if (chunk > 65535U) {
			chunk = 65535U;   /* HAL_SPI_Receive 的 Size 是 uint16_t */
		}
		HAL_SPI_Receive(&hspi1, buf, (uint16_t) chunk, 1000);
		buf += chunk;
		len -= chunk;
	}
	W25Q_CS_High();
}

/* 跨页写：按 256 字节页边界拆分，每页写使能 + 页编程（写前需已擦除目标区） */
void W25Q32_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
	while (len > 0) {
		uint32_t page_remain = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
		uint32_t chunk = len;
		if (chunk > page_remain) {
			chunk = page_remain;   /* 一页最多 256 字节，不能跨页 */
		}

		W25Q_WriteEnable();
		W25Q_CS_Low();
		W25Q_SendCmdAddr(W25Q_CMD_PAGE_PROGRAM, addr);
		HAL_SPI_Transmit(&hspi1, (uint8_t *) buf, (uint16_t) chunk, 1000);
		W25Q_CS_High();
		W25Q_WaitBusy(1000);   /* 页编程约 3ms，1s 短超时防卡死 */

		addr += chunk;
		buf += chunk;
		len -= chunk;
	}
}

/* 擦 4KB 扇区 */
void W25Q32_EraseSector(uint32_t addr)
{
	W25Q_WriteEnable();
	W25Q_CS_Low();
	W25Q_SendCmdAddr(W25Q_CMD_SECTOR_ERASE, addr);
	W25Q_CS_High();
	W25Q_WaitBusy(1000);   /* 扇区擦除约 300ms，1s 短超时防卡死 */
}

/* 全片擦除（约 40s，慎用；会清空配置三副本 + 文件目录 + 文件数据区） */
void W25Q32_EraseChip(void)
{
	if (!w25q32_present) {
		USART_printf(&huart1, L("[存储器] 未检测到存储器，跳过擦除\n", "[Memory] Memory not detected, skip erase\n"));
		return;
	}
	USART_printf(&huart1, L("[存储器] 全片擦除中（约 40s）...\n", "[Memory] Full chip erase in progress (~40s)...\n"));
	W25Q_WriteEnable();
	uint8_t cmd = W25Q_CMD_CHIP_ERASE;
	W25Q_CS_Low();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	W25Q_CS_High();
	W25Q_WaitBusy(60000);  /* [修复] 全片擦除需 40~60s，用 60s 长超时（原 1s 提前放弃致擦除未完成误报） */
	USART_printf(&huart1, L("[存储器] 全片擦除完成\n", "[Memory] Full chip erase done\n"));
}

/* ==================== 自测 ==================== */
void W25Q32_SelfTest(void)
{
	uint32_t id = W25Q32_ReadID();
	USART_printf(&huart1, L("[存储器] JEDEC ID = 0x%06lX\n", "[Memory] JEDEC ID = 0x%06lX\n"), (unsigned long) id);

	if (id != 0xEF4016) {
		USART_printf(&huart1, L("[存储器] 失败：ID 不符（期望 EF4016）\n",
				"[Memory] FAIL: unexpected ID (expect EF4016)\n"));
		return;
	}
	USART_printf(&huart1, L("[存储器] 芯片识别正常\n", "[Memory] chip detected OK\n"));

	uint32_t addr = 0x000000;
	uint8_t wbuf[16];
	uint8_t rbuf[16] = {0};
	for (int i = 0; i < 16; i++) {
		wbuf[i] = (uint8_t) (0xA0 + i);
	}

	W25Q32_EraseSector(addr);
	USART_printf(&huart1, L("[存储器] 扇区0已擦除\n", "[Memory] sector 0 erased\n"));

	W25Q32_Write(addr, wbuf, 16);
	USART_printf(&huart1, L("[存储器] 已写入16字节\n", "[Memory] wrote 16 bytes\n"));

	W25Q32_Read(addr, rbuf, 16);

	USART_printf(&huart1, L("[存储器] 读回：", "[Memory] readback: "));
	for (int i = 0; i < 16; i++) {
		USART_printf(&huart1, "%02X ", rbuf[i]);
	}
	USART_printf(&huart1, "\n");

	int ok = 1;
	for (int i = 0; i < 16; i++) {
		if (rbuf[i] != wbuf[i]) {
			ok = 0;
			break;
		}
	}

	if (ok) {
		USART_printf(&huart1, L("[存储器] 自测通过\n", "[Memory] SELF-TEST PASS\n"));
	} else {
		USART_printf(&huart1, L("[存储器] 自测失败：数据不一致\n", "[Memory] SELF-TEST FAIL: data mismatch\n"));
	}
}
