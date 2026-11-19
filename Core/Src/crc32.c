/*
 * crc32.c
 * CRC32（IEEE 802.3）查表实现，首次调用时生成 256 项表。
 * 支持一次性 crc32() 和流式 crc32_init/update/final。
 */

#include "crc32.h"
#include <stddef.h>
static uint32_t crc_table[256];
static uint8_t crc_table_ready = 0;

static void crc32_make_table(void)
{
	if (crc_table_ready)
		return;
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int j = 0; j < 8; j++) {
			c = (c & 1U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
		}
		crc_table[i] = c;
	}
	crc_table_ready = 1;
}

uint32_t crc32_init(void)
{
	crc32_make_table();
	return 0xFFFFFFFFU;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
	if (data == NULL)
		return crc;
	for (uint32_t i = 0; i < len; i++) {
		crc = crc_table[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8);
	}
	return crc;
}

uint32_t crc32_final(uint32_t crc)
{
	return crc ^ 0xFFFFFFFFU;
}

uint32_t crc32(const uint8_t *data, uint32_t len)
{
	uint32_t crc = crc32_init();
	crc = crc32_update(crc, data, len);
	return crc32_final(crc);
}
