/*
 * progress.h
 * 离线执行进度断电持久化：内部 Flash 预留一页（1KB = 256 字），逐字编程存「下一行文件偏移」，
 * 不擦除（写满才擦一次）；PVD 掉电中断兜底补写。
 */

#ifndef INC_PROGRESS_H_
#define INC_PROGRESS_H_

#include "flash.h"
#include <stdint.h>

/* 预留页：第 62 页（0x0800F800）= 离线执行进度（文件偏移环形日志）
   第 61 页 = 打孔/移动计数器（2 字一组环形日志，页对齐可擦除）
   第 63 页 = 用户配置区 */
#define PROGRESS_ADDR    ADDR_FLASH_PAGE_62
#define PROGRESS_WORDS   (FLASH_PAGE_SIZE / sizeof(uint32_t))  /* 256 字 */
#define COUNTER_ADDR     ADDR_FLASH_PAGE_61                    /* 计数器区（独立页） */

void     Progress_Init(void);         /* 读当前进度（找最后一个非 0xFF 的字） */
void     Progress_Clear(void);        /* 擦除预留页（S17 从头执行时重置） */
void     Progress_Save(uint32_t offset);/* 逐字写「下一行文件偏移」（不擦除） */
uint32_t Progress_Get(void);          /* 返回进度（文件偏移，0 = 无进度） */
uint8_t  Progress_HasResume(void);    /* 是否有可恢复进度（偏移 > 0） */

/* [新增] 打孔/移动计数器断电持久化（第 61 页环形日志，2 字一组） */
void     Counters_Save(uint32_t punch, uint32_t move);  /* 写一组（主循环空闲 + PVD 掉电） */
int      Counters_Load(uint32_t *punch, uint32_t *move);/* 读最后一组（上电恢复），返回 1=有有效组 0=页空 */
void     Counters_Clear(void);                          /* 擦除计数器页（S31 擦存储器时清零） */

/* PVD 掉电检测 */
void     Progress_PVDConfig(void);    /* 配置 PVD（阈值 2.9V）+ 使能中断 */
void     Progress_PVDSave(void);      /* PVD 掉电回调里调用：补写当前行号 + 计数器 */

#endif /* INC_PROGRESS_H_ */
