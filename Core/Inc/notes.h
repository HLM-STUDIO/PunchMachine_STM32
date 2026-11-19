/*
 * notes.h
 *
 *  Created on: 2026年1月23日
 *      Author: huang
 */

#ifndef INC_NOTES_H_
#define INC_NOTES_H_

#include <stdint.h>
#include <stddef.h>


#define NOTES_MAX 50            /* [修改] 最大音数（数组容量 50） */
#define NOTES_DEFAULT_COUNT 30  /* 默认音数 30 */
#define NOTE_SPACING_MM 2.0f
#define ENCODER_PULSES_PER_25_4MM 720.0f

/* API */
void Notes_Init(float home_offset_mm); /* 在 homing 完成后调用，参数为相对于原点的偏移（单位 mm），例如 -200.0f */
void Notes_SetSpacing(float mm); /* [新增] 设置孔间距（mm），配合 Notes_Init 重算定位点 */
void Notes_SetCount(uint16_t count); /* [新增] 设置音数（15/20/30/40/50），配合 Notes_Init 重算定位点 */
int32_t Notes_GetPulseForIndex(uint16_t idx);
float Notes_GetMMForIndex(uint16_t idx);
int32_t Notes_ConvertMMToPulses(float mm);
float Notes_ConvertPulsesToMM(int32_t pulses);
void Notes_MoveToIndex(uint16_t idx); /* 将全局 target_position 设为该位置（会被 motor 控制循环使用） */

/* [新增] 软件限位脉冲缓存：mm×100 → 脉冲（整数定点 720/2540，无软浮点，可在中断调用）。
   避免 Move_Control_Task 每 1ms 在中断里跑 roundf 软浮点 */
void Notes_SetSwMaxMM100(uint16_t mm100);
int32_t Notes_GetSwMaxPulses(void);
uint8_t Notes_IsInited(void); /* [新增] 定位表是否已初始化（上电到首次 homing 完成前为 0） */



#endif /* INC_NOTES_H_ */
