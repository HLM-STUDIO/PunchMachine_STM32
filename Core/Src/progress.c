/*
 * progress.c
 * 离线执行进度断电持久化：内部 Flash 预留页逐字编程 + PVD 掉电兜底。
 *
 * 进度页是「环形日志」：每执行完一行，把下一行文件偏移写到下一个空字（0xFFFFFFFF）。
 * 读进度 = 找最后一个非 0xFF 的字。写满 256 字后擦一次重来。
 */

#include "progress.h"
#include "system.h"   /* [新增] current_config.punch_total / move_total（PVD 掉电保存计数器） */

/* ---- 逐字编程（不擦除，直接 HAL_FLASH_Program） ---- */
static int flash_program_word_noerase(uint32_t addr, uint32_t data)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t) data);
	HAL_FLASH_Lock();
	return (st == HAL_OK) ? 0 : -1;
}

/* ---- 擦整页 ---- */
static void flash_erase_page(uint32_t addr)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = addr;
	EraseInitStruct.NbPages = 1;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}

/* [修复] 进度改「2 字一组冗余」：第 1 字 = 偏移，第 2 字 = ~偏移（反码）。
   掉电半写（HAL_FLASH_Program 写 32 位 = 2 个 16 位，中间断电）时两字不一致，
   读时校验丢弃，避免恢复成错误偏移导致 resume 起点错位 */
#define PROGRESS_GROUPS  (PROGRESS_WORDS / 2)

/* ---- 读最后一个有效组（偏移 + 反码校验）。found 输出：1=找到有效组（含偏移 0），0=全空 */
static uint32_t progress_read_last(int *found)
{
	uint32_t last = 0;
	*found = 0;
	for (uint32_t g = 0; g < PROGRESS_GROUPS; g++) {
		uint32_t off = *(volatile uint32_t *) (PROGRESS_ADDR + g * 2 * sizeof(uint32_t));
		uint32_t inv = *(volatile uint32_t *) (PROGRESS_ADDR + (g * 2 + 1) * sizeof(uint32_t));
		if (off != 0xFFFFFFFFU && inv != 0xFFFFFFFFU && inv == ~off) {
			last = off;
			*found = 1;
		} else if (off == 0 && inv == 0xFFFFFFFFU) {
			/* [修复] 偏移 0 的合法组：~0=0xFFFFFFFF，写全 1 到 Flash 等于不写（保持擦除态），
			   原判断 inv!=0xFFFFFFFFU 会误判为未写入，导致首行断点丢失 */
			last = 0;
			*found = 1;
		}
	}
	return last;
}

/* ---- 写下一组（不擦除；写满返回 -1） ---- */
static int progress_write_next_noerase(uint32_t data)
{
	for (uint32_t g = 0; g < PROGRESS_GROUPS; g++) {
		uint32_t off = *(volatile uint32_t *) (PROGRESS_ADDR + g * 2 * sizeof(uint32_t));
		if (off == 0xFFFFFFFFU) {
			flash_program_word_noerase(PROGRESS_ADDR + g * 2 * sizeof(uint32_t), data);
			flash_program_word_noerase(PROGRESS_ADDR + (g * 2 + 1) * sizeof(uint32_t), ~data);
			return 0;
		}
	}
	return -1;   /* 写满 */
}

/* ---- 写下一组（写满则擦除重来）——主循环用，不跑在中断里 ---- */
static void progress_write_next(uint32_t data)
{
	if (progress_write_next_noerase(data) != 0) {
		flash_erase_page(PROGRESS_ADDR);
		flash_program_word_noerase(PROGRESS_ADDR, data);
		flash_program_word_noerase(PROGRESS_ADDR + sizeof(uint32_t), ~data);
	}
}

/* ==================== [新增] 打孔/移动计数器持久化 ====================
 * 计数器区：第 61 页（独立页，页对齐），2 字一组（punch_total, move_total）环形日志。
 * 主循环空闲/PVD 掉电都写一组；读 = 找最后一组（首个字非 0xFF）。 */
#define COUNTER_GROUPS  (PROGRESS_WORDS / 2)   /* 128 组 */

static int counters_write_group_noerase(uint32_t punch, uint32_t move)
{
	for (uint32_t g = 0; g < COUNTER_GROUPS; g++) {
		uint32_t a = *(volatile uint32_t *) (COUNTER_ADDR + g * 2 * sizeof(uint32_t));
		if (a == 0xFFFFFFFFU) {
			flash_program_word_noerase(COUNTER_ADDR + g * 2 * sizeof(uint32_t), punch);
			flash_program_word_noerase(COUNTER_ADDR + (g * 2 + 1) * sizeof(uint32_t), move);
			return 0;
		}
	}
	return -1;   /* 写满 */
}

static void counters_write_group(uint32_t punch, uint32_t move)
{
	if (counters_write_group_noerase(punch, move) != 0) {
		flash_erase_page(COUNTER_ADDR);
		counters_write_group_noerase(punch, move);
	}
}

static int counters_read_last(uint32_t *punch, uint32_t *move)
{
	int found = 0;
	*punch = 0;
	*move = 0;
	for (uint32_t g = 0; g < COUNTER_GROUPS; g++) {
		uint32_t a = *(volatile uint32_t *) (COUNTER_ADDR + g * 2 * sizeof(uint32_t));
		uint32_t b = *(volatile uint32_t *) (COUNTER_ADDR + (g * 2 + 1) * sizeof(uint32_t));
		/* 一组必须两个 word 都有效（非 0xFF）。掉电中断写到一半（punch 写了 move 没写）时，
		   第二个 word 还是 0xFF，这种情况丢弃这组，避免恢复成 0xFFFFFFFF 的天文数字 */
		if (a != 0xFFFFFFFFU && b != 0xFFFFFFFFU) {
			*punch = a;
			*move = b;
			found = 1;
		}
	}
	return found;  /* 1=页 61 有有效组（曾被写入），0=页 61 全空（从未写过） */
}

void Counters_Save(uint32_t punch, uint32_t move)
{
	counters_write_group(punch, move);
}

int Counters_Load(uint32_t *punch, uint32_t *move)
{
	return counters_read_last(punch, move);
}

void Counters_Clear(void)
{
	flash_erase_page(COUNTER_ADDR);
}

/* ==================== 公开接口 ==================== */
void Progress_Init(void)
{
	/* 上电读进度，不擦除（保留供恢复） */
	int found;
	(void) progress_read_last(&found);
}

void Progress_Clear(void)
{
	flash_erase_page(PROGRESS_ADDR);
}

void Progress_Save(uint32_t line)
{
	progress_write_next(line);
}

uint32_t Progress_Get(void)
{
	int found;
	return progress_read_last(&found);
}

uint8_t Progress_HasResume(void)
{
	int found;
	(void) progress_read_last(&found);
	return found ? 1 : 0;
}

/* ---- PVD 掉电检测（阈值 2.9V，上升沿触发 = 掉电） ---- */
void Progress_PVDConfig(void)
{
	PWR_PVDTypeDef sConfigPVD = {0};
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;            /* 2.9V 阈值 */
	/* 掉电 = VDD 跌破阈值 = PVDO 0→1 = 上升沿，须用 RISING（FALLING 是上电恢复才触发） */
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
	HAL_PWR_ConfigPVD(&sConfigPVD);
	HAL_PWR_EnablePVD();
	/* PVD_IRQn 的 NVIC 优先级/使能已由 CubeMX 在 HAL_MspInit 里配置 */
}

/* PVD 中断回调里调用：补写当前进度（由 fileio 传入当前文件偏移）+ 打孔/移动计数器。
   掉电瞬间只做逐字编程（几十 µs），不擦除（擦除 20~40ms 来不及）。 */
void Progress_PVDSave(void)
{
	extern volatile uint32_t g_resume_offset;
	progress_write_next_noerase(g_resume_offset);
	/* [新增] 掉电兜底保存打孔/移动计数器（逐字编程，写满则放弃） */
	counters_write_group_noerase(current_config.punch_total, current_config.move_total);
}

/* HAL PVD 回调（weak 重定义）：掉电瞬间补写进度 */
void HAL_PWR_PVDCallback(void)
{
	Progress_PVDSave();
}
