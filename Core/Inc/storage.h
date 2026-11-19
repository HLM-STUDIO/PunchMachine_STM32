/*
 * storage.h
 * 系统变量在 W25Q32 的三副本冗余存储（CRC + seq）+ 内部 Flash / W25Q32 双冗余接口。
 *
 * 三副本：每份占一个 4KB 扇区，轮流写（选 seq 最小的写，seq=max+1），磨损均衡；
 * 读时选 seq 最大且 CRC 正确的副本。写一半断电只坏一份，另两份仍有效。
 */

#ifndef INC_STORAGE_H_
#define INC_STORAGE_H_

#include "system.h"
#include "w25q32.h"

#define SYSVAR_MAGIC  0xA5A5C3C3UL   /* 副本魔数 */

/* 副本扇区地址（系统变量区 0x000000，3 × 4KB 扇区） */
#define SYSVAR_COPY_A  (W25Q_ADDR_SYSVAR + 0 * W25Q_SECTOR_SIZE)
#define SYSVAR_COPY_B  (W25Q_ADDR_SYSVAR + 1 * W25Q_SECTOR_SIZE)
#define SYSVAR_COPY_C  (W25Q_ADDR_SYSVAR + 2 * W25Q_SECTOR_SIZE)

/* 副本头部（16 字节），后面紧跟 SystemConfig_t 数据 */
typedef struct {
	uint32_t magic;   /* offset 0 */
	uint32_t seq;     /* offset 4：写入序号，越大越新 */
	uint16_t len;     /* offset 8：sizeof(SystemConfig_t) */
	uint16_t _pad;    /* offset 10 */
	uint32_t crc32;   /* offset 12：数据 CRC */
	/* offset 16 起：SystemConfig_t 数据本体 */
} SysVarHeader_t;

/* ---- W25Q32 三副本 ---- */
int Storage_W25Load(SystemConfig_t *cfg);        /* 读三副本选最新有效；无有效返回 -1 */
int Storage_W25Save(const SystemConfig_t *cfg);  /* 轮流写三副本；返回 0 成功 */

/* ---- 双冗余（内部 Flash + W25Q32） ---- */
int Storage_Load(SystemConfig_t *cfg);   /* W25Q32(权威) → 内部Flash(回退) → 默认值 */
int Storage_LoadCompat(SystemConfig_t *cfg);  /* [新增] 读旧配置迁移用：W25Q32 按旧 len+旧 CRC，内部 Flash 按 magic 弱校验 */
int Storage_Save(const SystemConfig_t *cfg);  /* 同时写内部 Flash + W25Q32 */

#endif /* INC_STORAGE_H_ */
