/*
 * crc32.h
 * CRC32（IEEE 802.3，多项式 0xEDB88320）校验，用于 W25Q32 存储/文件的完整性校验。
 * 提供一次性 + 流式两种接口。
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include <stdint.h>

/* 一次性：对整块数据算 CRC */
uint32_t crc32(const uint8_t *data, uint32_t len);

/* 流式：分块累加（上传边收边算用） */
uint32_t crc32_init(void);                                        /* 返回初始 crc */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);
uint32_t crc32_final(uint32_t crc);                               /* 取最终值 */

#endif /* INC_CRC32_H_ */
