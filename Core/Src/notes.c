#include "notes.h"
#include "homing.h" /* 使用 HOME_FINAL_OFFSET */
#include "motor.h"
#include "stm32f1xx_hal.h"
#include <math.h>

/* [新增] 主循环重算请求标志（system.c 定义）。Notes_MoveToIndex 用它做门控：
   重算未完成时拒绝移动，防止读旧表/零值表冲 home */
extern volatile uint8_t notes_recalc_request;

/* 内部存储 */
static int32_t notes_pos_pulses[NOTES_MAX]; /* [修改] 数组容量为最大音数 50 */
static float pulses_per_mm = 0.0f;
static uint16_t notes_count = NOTES_DEFAULT_COUNT; /* [新增] 运行期音数（默认 30） */
static float spacing_mm = NOTE_SPACING_MM; /* [新增] 运行期孔间距（mm），默认 2.0，可用 Notes_SetSpacing 修改 */
static float spacing_pulses_f = 0.0f; /* 精确的浮点步长（pulse） */
static float base_offset_pulses_f = 0.0f; /* 精确的基准脉冲（浮点） */
static uint8_t notes_inited = 0;
static int32_t notes_sw_max_pulses = 0; /* [新增] 软件限位脉冲缓存（整数定点，避免每 tick 软浮点） */

/* [新增] 设置软件限位脉冲缓存：mm×100 → 脉冲。
   720 脉冲 = 25.4mm → sw_max_pulses = mm100 * 720 / 2540，纯整数无软浮点，可在中断调用 */
void Notes_SetSwMaxMM100(uint16_t mm100) {
	notes_sw_max_pulses = ((int32_t) mm100 * 720 + 1270) / 2540;
}

int32_t Notes_GetSwMaxPulses(void) {
	return notes_sw_max_pulses;
}

uint8_t Notes_IsInited(void) {
	return notes_inited;
}

/* 将 mm 转为脉冲（四舍五入） */
int32_t Notes_ConvertMMToPulses(float mm) {
	if (pulses_per_mm <= 0.0f) /* [修改] 未初始化时返回 0，防除零/异常 */
		return 0;
	return (int32_t) (roundf(mm * pulses_per_mm));
}

float Notes_ConvertPulsesToMM(int32_t pulses) {
	if (pulses_per_mm <= 0.0f) /* [修改] 未初始化时返回 0，防除零 */
		return 0.0f;
	return ((float) pulses) / pulses_per_mm;
}

/* 说明：所有对外接口都使用 1-based 索引（1..NOTES_COUNT） */

/* 返回第 idx (1..NOTES_COUNT) 点的脉冲绝对位置（相对于 encoder 计数） */
int32_t Notes_GetPulseForIndex(uint16_t idx) {
	if (!notes_inited)
		return 0;
	if (idx < 1)
		idx = 1;
	if (idx > notes_count)
		idx = notes_count;
	return notes_pos_pulses[idx - 1];
}

/* 返回第 idx (1..NOTES_COUNT) 点相对于 base 的毫米值（基准为 Notes_Init 的 home_offset_mm） */
float Notes_GetMMForIndex(uint16_t idx) {
	if (!notes_inited)
		return 0.0f;
	if (idx < 1)
		idx = 1;
	if (idx > notes_count)
		idx = notes_count;
	/* 使用浮点基准计算，减少舍入误差 */
	float pos_pulses_f = (float) notes_pos_pulses[idx - 1];
	float rel_pulses_f = pos_pulses_f - base_offset_pulses_f; /* 相对基准的脉冲（浮点） */
	return rel_pulses_f / pulses_per_mm; /* 转为 mm 并返回 */
}

/* 初始化 notes：必须在 homing 完成后调用
 * home_offset_mm：相对于原点的偏移（mm），例如 200.0f 表示在原点负方向 200mm 处为第 1 个参考基点
 */
void Notes_Init(float home_offset) { //这里传入正。函数里面处理成负的了
	if (home_offset < 0) { //小于0就初始化失败！
		notes_inited = 0;
		return;
	}
	notes_inited = 0; /* [修复] 进入重算先清标志，防 ISR 在重算期间读半更新表 */
	float home_offset_mm = -home_offset;
	pulses_per_mm = (ENCODER_PULSES_PER_25_4MM / 25.4f);
	/* 浮点步长（pulse） */
	spacing_pulses_f = spacing_mm * pulses_per_mm; /* [修改] 用运行期间距 spacing_mm（原为 NOTE_SPACING_MM 宏） */

	/* 以浮点精确计算基准脉冲位置 */
	base_offset_pulses_f = (float) HOME_FINAL_OFFSET
			+ (home_offset_mm * pulses_per_mm);

	for (uint16_t i = 0; i < notes_count; ++i) {
		/* [修改] 浮点等距排列：base + i × 浮点步长，roundf 存入整数脉冲数组。
		   不用整数脉冲步长，直接用浮点步长乘 i，避免取整后再累乘的偏差 */
		float pos_f = base_offset_pulses_f + (float) i * spacing_pulses_f;
		notes_pos_pulses[i] = (int32_t) roundf(pos_f);
	}

	notes_inited = 1;
}

/* [新增] 设置孔间距（mm）。只更新间距，定位点需再调 Notes_Init 重算 */
void Notes_SetSpacing(float mm) {
	if (mm > 0.0f) {
		spacing_mm = mm;
	}
}

/* [新增] 设置音数（15/20/30/40/50），需再调 Notes_Init 重算定位点 */
void Notes_SetCount(uint16_t count) {
	if (count >= 1 && count <= NOTES_MAX) {
		notes_count = count;
	}
}

/* 将定位点索引（1..NOTES_COUNT）转换并设为全局 target_position（motor.c 将负责闭环移动） */
void Notes_MoveToIndex(uint16_t idx) {
	if (!notes_inited)
		return;
	/* [修复] 重算请求未消费（音数/间距刚改，主循环尚未 Notes_Init）时拒绝，
	   防止读旧表/零值表（切更大 bank 时 notes_pos[] 未初始化=0，电机冲向 home） */
	if (notes_recalc_request)
		return;
	if (idx >= 1 && idx <= notes_count) {
		move_target_position = notes_pos_pulses[idx - 1];
	}
}
