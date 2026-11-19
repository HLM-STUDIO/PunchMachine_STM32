/*
 * pid.h
 *
 *  Created on: 2026年1月22日
 *      Author: huang
 */

#ifndef INC_MCODE_H_
#define INC_MCODE_H_

#include <stdint.h>
#include <stddef.h>

/* 解析后的命令结构 */
typedef struct {
    int hasM;
    int M; /* e.g. 1..30, or 90/80 */
    int hasY;
    int Y;
    int hasP;
    int P;
    int hasS;
    int S_id;   /* S后面的数字，如 11 或 801 */
    int S_val_present; /* 如果是 S801=87 则为1，否则为0 */
    int S_val;  /* 如果有等号并带值，该值放这里 */
} parsed_cmd_t;


extern volatile uint8_t move_busy;
extern volatile uint8_t feed_busy;
extern volatile uint8_t punch_busy;


extern parsed_cmd_t pending_cmd;
/* 回调类型（用户实现） */
typedef void (*mcb_move_t)(uint16_t M);        /* M command */
typedef void (*mcb_feed_t)(int16_t Y);        /* Y command */
typedef void (*mcb_punch_t)(uint16_t P);       /* P command */
typedef void (*mcb_s_t)(int S_id, uint8_t has_val, int32_t val); /* S command */
typedef void (*mcb_evt_t)(void);          /* batch start/end (M90/M80) */

/* 初始化：注册回调（任何未传入的回调可为 NULL） */
void MCodeSimple_Init(mcb_move_t cbM,
                      mcb_feed_t cbY,
                      mcb_punch_t cbP,
                      mcb_s_t cbS,
                      mcb_evt_t cb_batch_start,
                      mcb_evt_t cb_batch_end);

void MCodeSimple_Service(void);
void MCodeSimple_ProcessBuffer(const char *buf);
void MCodeSimple_NotifyMoveDone(void);
void MCodeSimple_NotifyFeedDone(void);
void MCodeSimple_NotifyPunchDone(void);
void MCodeSimple_Abort(void); /* [新增] 紧急停止：清 pending 与 busy 状态，命令状态机复位 */
void MCodeSimple_SetInMcode(uint8_t v); /* [新增] 置 mcode 批模式标志（断电恢复跳过 M90 时用，避免 pending_send_ok=0 卡死） */

/* [新增] 出厂标定序列（首次烧录 calibrated=0）：homing → 标软件限位 → 回中 → 标主轴死区 → 回中 → 标进纸死区 */
void FactoryCalib_Start(void);
void FactoryCalib_Task(void);
uint8_t FactoryCalib_IsActive(void);
void FactoryCalib_Stop(void);

/* [新增] 开机自检序列（每次开机）：homing → 到软件限位 → 回中 */
void SelfTest_Start(void);
void SelfTest_Task(void);
uint8_t SelfTest_IsActive(void);
void SelfTest_Stop(void);
//void sendOK();

/* 用户可以在回调里或外层实现发送 OK 的函数；默认库不自动发送 OK，
   以便你的动作（可能异步）在完成后再发送 OK。
   但如果你希望库内发送 OK，可以注册这个函数（可选）。 */
typedef void (*send_ok_t)(void);
void MCodeSimple_RegisterSendOK(send_ok_t send_ok_cb);

#endif /* INC_MCODE_H_ */
