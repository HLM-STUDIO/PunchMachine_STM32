/*
 * storage.c
 * 系统变量三副本冗余存储（W25Q32）+ 内部 Flash / W25Q32 双冗余。
 */

#include "storage.h"
#include "crc32.h"
#include "flash.h"
#include "system.h"   /* [修复] ValidateConfig 版本校验 */
#include <string.h>   /* [新增] memset 用于配置迁移按旧 len 读 */

/* ---- 写单副本（先擦扇区，再写头部 + 数据） ---- */
static void storage_write_copy(uint32_t addr, uint32_t seq, const SystemConfig_t *cfg)
{
	W25Q32_EraseSector(addr);

	SysVarHeader_t hdr;
	hdr.magic = SYSVAR_MAGIC;
	hdr.seq = seq;
	hdr.len = sizeof(SystemConfig_t);
	hdr._pad = 0;
	hdr.crc32 = crc32((const uint8_t *) cfg, sizeof(SystemConfig_t));

	W25Q32_Write(addr, (const uint8_t *) &hdr, sizeof(hdr));
	W25Q32_Write(addr + sizeof(SysVarHeader_t), (const uint8_t *) cfg, sizeof(SystemConfig_t));
}

/* ---- 读三副本，选 seq 最大且 CRC 正确的 ---- */
int Storage_W25Load(SystemConfig_t *cfg)
{
	static const uint32_t addrs[3] = { SYSVAR_COPY_A, SYSVAR_COPY_B, SYSVAR_COPY_C };
	uint32_t best_seq = 0;
	int best_idx = -1;

	for (int i = 0; i < 3; i++) {
		SysVarHeader_t hdr;
		W25Q32_Read(addrs[i], (uint8_t *) &hdr, sizeof(hdr));
		if (hdr.magic != SYSVAR_MAGIC || hdr.len != sizeof(SystemConfig_t))
			continue;

		SystemConfig_t tmp;
		W25Q32_Read(addrs[i] + sizeof(SysVarHeader_t), (uint8_t *) &tmp, sizeof(SystemConfig_t));
		if (crc32((const uint8_t *) &tmp, sizeof(SystemConfig_t)) != hdr.crc32)
			continue;
		/* [修复] 版本校验：W25Q32 是权威副本，须过 ValidateConfig(version/magic/checksum)，
		   否则未来结构体字段复用(不改变 sizeof)时旧配置会带垃圾字段被加载 */
		if (ValidateConfig(&tmp) != 0)
			continue;

		/* >= 保证 seq=0 的首次有效副本也能被选中 */
		if (best_idx < 0 || hdr.seq >= best_seq) {
			best_seq = hdr.seq;
			best_idx = i;
		}
	}

	if (best_idx < 0)
		return -1;

	W25Q32_Read(addrs[best_idx] + sizeof(SysVarHeader_t), (uint8_t *) cfg, sizeof(SystemConfig_t));
	return 0;
}

/* ---- 轮流写三副本：选 seq 最小的（最旧）写，seq = max+1 ---- */
int Storage_W25Save(const SystemConfig_t *cfg)
{
	static const uint32_t addrs[3] = { SYSVAR_COPY_A, SYSVAR_COPY_B, SYSVAR_COPY_C };
	uint32_t max_seq = 0;
	int min_idx = 0;
	uint32_t min_seq = 0xFFFFFFFFUL;

	for (int i = 0; i < 3; i++) {
		SysVarHeader_t hdr;
		W25Q32_Read(addrs[i], (uint8_t *) &hdr, sizeof(hdr));
		uint32_t seq = (hdr.magic == SYSVAR_MAGIC) ? hdr.seq : 0;
		if (seq > max_seq)
			max_seq = seq;
		if (seq < min_seq) {
			min_seq = seq;
			min_idx = i;
		}
	}

	storage_write_copy(addrs[min_idx], max_seq + 1, cfg);
	return 0;
}

/* ---- 双冗余读：W25Q32(权威) → 内部 Flash(回退) ---- */
int Storage_Load(SystemConfig_t *cfg)
{
	/* 1. W25Q32（固件升级后仍在，最权威） */
	if (w25q32_present && Storage_W25Load(cfg) == 0) {
		return 0;
	}

	/* 2. 内部 Flash（回退） */
	Flash_HAL_Read_N_Data(FLASH_USER_START_ADDR, (uint32_t *) cfg,
			(uint16_t) ((sizeof(SystemConfig_t) + 3U) / 4U));
	if (ValidateConfig(cfg) == 0) {
		return 0;
	}

	/* 3. 都无效 */
	return -1;
}

/* [新增] 读旧配置（版本不匹配时迁移用）：按 hdr.len（旧结构体大小）读 + 旧 CRC 校验，
   这样即使未来结构体变大（新增字段）也能正确读旧数据。读出的旧字段在前 N 字节，
   新结构体末尾的新字段保持 0，由 MigrateConfig 填默认。 */
static int storage_w25load_compat(SystemConfig_t *cfg) {
	static const uint32_t addrs[3] = { SYSVAR_COPY_A, SYSVAR_COPY_B, SYSVAR_COPY_C };
	uint32_t best_seq = 0;
	int best_idx = -1;
	uint16_t best_len = 0;

	for (int i = 0; i < 3; i++) {
		SysVarHeader_t hdr;
		W25Q32_Read(addrs[i], (uint8_t *) &hdr, sizeof(hdr));
		if (hdr.magic != SYSVAR_MAGIC)
			continue;
		if (hdr.len == 0 || hdr.len > sizeof(SystemConfig_t))
			continue; /* 旧结构体比新结构体还大，无法迁移，跳过 */

		uint8_t tmp[sizeof(SystemConfig_t)];
		memset(tmp, 0, sizeof(tmp));
		W25Q32_Read(addrs[i] + sizeof(SysVarHeader_t), tmp, hdr.len); /* 按旧 len 读 */
		if (crc32(tmp, hdr.len) != hdr.crc32)
			continue; /* 旧数据 CRC（按旧 len 算）*/

		if (best_idx < 0 || hdr.seq >= best_seq) {
			best_seq = hdr.seq;
			best_idx = i;
			best_len = hdr.len;
		}
	}

	if (best_idx < 0)
		return -1;

	memset(cfg, 0, sizeof(*cfg));
	W25Q32_Read(addrs[best_idx] + sizeof(SysVarHeader_t), (uint8_t *) cfg, best_len);
	return 0;
}

int Storage_LoadCompat(SystemConfig_t *cfg) {
	/* 1. W25Q32（权威，按旧 len + 旧 CRC 校验）*/
	if (w25q32_present && storage_w25load_compat(cfg) == 0) {
		return 0;
	}

	/* 2. 内部 Flash（回退：按当前大小读）*/
	memset(cfg, 0, sizeof(*cfg));
	Flash_HAL_Read_N_Data(FLASH_USER_START_ADDR, (uint32_t *) cfg,
			(uint16_t) ((sizeof(SystemConfig_t) + 3U) / 4U));
	if (cfg->magic != 0xA5A5) {
		return -1;
	}
	/* [修复] version==当前版本时照常校验 checksum（防损坏数据被加载后重算固化）；
	   version<当前版本时跳过 checksum（旧 checksum 按旧大小算，结构体变大后不再匹配）*/
	if (cfg->version == CONFIG_VERSION && cfg->checksum != CalculateChecksum(cfg)) {
		return -1;
	}
	return 0;
}

/* ---- 双冗余写：同时写内部 Flash + W25Q32 ---- */
int Storage_Save(const SystemConfig_t *cfg)
{
	int ret = 0;

	/* 内部 Flash（带 checksum） */
	SystemConfig_t tmp = *cfg;
	tmp.checksum = CalculateChecksum(&tmp);
	if (Flash_HAL_Write_N_Data(FLASH_USER_START_ADDR, (uint32_t *) &tmp,
			(uint16_t) ((sizeof(SystemConfig_t) + 3U) / 4U)) != 0) {
		ret = -1;
	}

	/* W25Q32（三副本） */
	if (w25q32_present) {
		if (Storage_W25Save(cfg) != 0) {
			ret = -1;
		}
	}

	return ret;
}
