/*
 * fileio.h
 * 离线 MCode 文件交互：上传（UPLOAD/END 分包）、执行（S17）、暂停/继续（S18/S19）、
 * 回读预览（S20）。文件存 W25Q32 文件数据区，掉电不丢、可重复执行。
 */

#ifndef INC_FILEIO_H_
#define INC_FILEIO_H_

#include <stdint.h>

/* 离线交互状态机 */
typedef enum {
	OFFLINE_IDLE = 0,     /* 空闲 */
	OFFLINE_UPLOAD,       /* 上传中 */
	OFFLINE_RUNNING,      /* 执行中 */
	OFFLINE_PAUSED,       /* 已暂停（记住当前行号） */
} offline_state_t;

#define FILE_NAME_MAX    64    /* 文件名最大字节（UTF-8） */
#define FILE_UPLOAD_PKT  512   /* 上传/回读分包大小（W25Q32 两页对齐） */

/* 文件目录元数据（存 W25Q32 文件目录区 4KB 开头） */
typedef struct {
	uint8_t  name[FILE_NAME_MAX]; /* 文件名 UTF-8 */
	uint32_t lines;               /* 总行数 */
	uint32_t size;                /* 文件字节数 */
	uint32_t crc;                 /* 内容 CRC32 */
	uint8_t  status;              /* 0=空，1=已上传 */
	uint8_t  reserved[3];
} FileDir_t;

/* 文件目录 / 数据区地址（见 w25q32.h 布局） */
#define FILE_DIR_ADDR   W25Q_ADDR_FILE_DIR
#define FILE_DATA_ADDR  W25Q_ADDR_FILE_DATA

void FileIO_Init(void);
void FileIO_Service(void);   /* 主循环调用：推进执行状态机（读下一行/喂 MCode） */
void FileIO_NotifyLineDoneOnly(void); /* [修复] 只置 fileio_line_done（推进离线执行），不回 OK，供 M80 组合行完成用 */

/* ---- 上传（UPLOAD/END 分包） ---- */
int  FileIO_UploadStart(const char *name, uint32_t lines, uint32_t size); /* 记录元数据 + 擦文件区 */
int  FileIO_UploadFeed(const uint8_t *data, uint16_t len);  /* 喂一包数据，边收边写 */
int  FileIO_UploadFinish(void);   /* 收满校验，写目录，回 OK/ERR */

/* ---- 执行/控制 ---- */
int  FileIO_Run(void);      /* S17：开始/重头执行 */
int  FileIO_Pause(void);    /* S18：行级暂停 */
int  FileIO_Resume(void);   /* S19：继续 */
int  FileIO_Preview(void);  /* S20：回读预览（分包回传） */
int  FileIO_PreviewAck(void); /* 上位机回 OK 时调用（推进回读下一包） */
int  FileIO_Stop(void);     /* S0：停止清进度 */

uint8_t FileIO_HasFile(void);  /* 是否有已上传文件（目录 status==1 且 CRC 有效） */

uint8_t FileIO_IsUploading(void); /* 是否处于上传模式（串口层据此把数据当文件内容喂） */

uint8_t FileIO_IsRunning(void); /* [新增] 是否在离线执行中（S17 RUNNING），供打孔阈值回零抑制用 */

/* 串口层收到一行文本时调用：识别 UPLOAD/END/OK（预览 ACK）/上传数据。
   返回 1 = 已由 fileio 处理；返回 0 = 未处理，交给 MCodeSimple_ProcessBuffer。 */
int FileIO_ProcessLine(const char *line);

#endif /* INC_FILEIO_H_ */
