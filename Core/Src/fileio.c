/*
 * fileio.c
 * 离线 MCode 文件交互：上传/执行/暂停/继续/回读。
 *
 * 执行状态机通过 MCodeSimple_RegisterSendOK 感知「一行执行完」：
 * cb_send_ok 在 MCodeSimple_Service（1ms 中断）里调用，只置标志，
 * FileIO_Service 在主循环里读下一行并喂 MCode（避免中断里做 SPI 传输）。
 */

#include "fileio.h"
#include "w25q32.h"
#include "crc32.h"
#include "mcode.h"
#include "usart.h"
#include "progress.h"
#include <string.h>
#include <stdlib.h>

/* 当前执行的文件偏移（供 PVD 掉电补写用：存偏移而非行号，避免空行/注释导致错位） */
volatile uint32_t g_resume_offset = 0;

/* ---- 状态 ---- */
static offline_state_t offline_state = OFFLINE_IDLE;
static FileDir_t file_dir;

/* [新增] 是否在离线执行中（S17 RUNNING）。用于抑制打孔阈值中途回零，避免打断文件 */
uint8_t FileIO_IsRunning(void) {
	return (offline_state == OFFLINE_RUNNING) ? 1 : 0;
}

/* 上传状态 */
static uint32_t up_offset;
static uint32_t up_size;
static uint32_t up_crc;    /* 流式 crc 中间值 */

/* 执行状态 */
static uint32_t run_offset;         /* 文件数据区读取偏移 */
static uint32_t run_line;           /* 当前行号（1-based） */
static char line_buf[256];
static volatile uint8_t fileio_line_done = 0;   /* 一行执行完标志（cb_send_ok 置位） */

/* 回读状态 */
static uint8_t preview_active = 0;
static uint32_t preview_offset;
static uint8_t preview_wait_ack = 0;   /* 1=等上位机 OK 后再发下一包 */

/* [修复] 存储器就绪检查：未焊 -4，容量不足（文件区固定在 0x200000，需 >=4MB 芯片）-5。
   原只查 w25q32_present，识别出 1MB/2MB（W25Q80/Q16）时文件区越界却仍放行 */
static int fileio_storage_ready(void)
{
	if (!w25q32_present)
		return -4;   /* 未焊 */
	if (w25q_capacity < 0x400000UL)
		return -5;   /* 容量不足（<4MB），文件区 0x200000+ 越界 */
	return 0;
}

/* ---- 目录读写 ---- */
static void dir_read(FileDir_t *d)
{
	W25Q32_Read(FILE_DIR_ADDR, (uint8_t *) d, sizeof(FileDir_t));
}

static void dir_write(const FileDir_t *d)
{
	W25Q32_EraseSector(FILE_DIR_ADDR);
	W25Q32_Write(FILE_DIR_ADDR, (const uint8_t *) d, sizeof(FileDir_t));
}

/* ---- OK 回调（1ms 中断里） ----
   注意：cb_send_ok 一旦注册，mcode 的 sendOK_internal 就只走这里、不再 USART_printf。
   所以必须分两种场景：
   - 离线执行（offline_state == RUNNING）：进度由 FileIO_Service 回，这里只置标志；
   - 串口逐行发送（其它状态）：这里要补回 "OK"，否则上位机收不到 OK 会卡死。 */
static void fileio_ok_cb(void)
{
	fileio_line_done = 1;
	/* [修复] 暂停（PAUSED）期间 OK 重发块每 1s 触发一次，若不排除会在暂停期持续刷 "OK\n" */
	if (offline_state != OFFLINE_RUNNING && offline_state != OFFLINE_PAUSED) {
		deferred_ok = 1; /* [修复] 协议 OK 延迟到主循环发送，避免 tx_busy 并发跳过丢 ACK */
	}
}

/* [修复] 只置 fileio_line_done（推进离线执行），不回 OK：M80 组合行完成时用，
   解耦「置行完成标志」与「回 ACK」，避免 pending_no_ok_on_complete 被 || cb_send_ok 短路失效 */
void FileIO_NotifyLineDoneOnly(void)
{
	fileio_line_done = 1;
}

void FileIO_Init(void)
{
	MCodeSimple_RegisterSendOK(fileio_ok_cb);
	dir_read(&file_dir);
	offline_state = OFFLINE_IDLE;
	fileio_line_done = 0;
	preview_active = 0;
}

/* ---- 从文件数据区读一行（处理 \r\n） ---- */
static uint32_t read_line(uint32_t offset, char *buf, uint32_t maxlen)
{
	uint32_t i = 0;
	while (i < maxlen - 1) {
		uint8_t ch;
		W25Q32_Read(FILE_DATA_ADDR + offset + i, &ch, 1);
		/* 0xFF = 文件末尾（擦除后未写入字节），也当行结束 */
		if (ch == '\n' || ch == '\r' || ch == '\0' || ch == 0xFF) {
			buf[i] = '\0';   /* 先在当前位置结束字符串，避免 \r\n 位置残留上一行字节 */
			i++;             /* 跳过当前换行符 */
			/* 若 \r 后紧跟 \n，多跳一个 */
			if (ch == '\r') {
				uint8_t nx;
				W25Q32_Read(FILE_DATA_ADDR + offset + i, &nx, 1);
				if (nx == '\n')
					i++;
			}
			return i;        /* 返回消费字节数（含换行符） */
		}
		buf[i] = (char) ch;
		i++;
	}
	buf[i] = '\0';
	return i;
}

/* 从文件开头扫描到 end_offset，统计有效行数（跳过空行和 // 注释行）。
   断电恢复时用它反推 run_line，让进度显示准确。 */
static uint32_t count_valid_lines(uint32_t end_offset)
{
	uint32_t off = 0;
	uint32_t count = 0;
	char tmp[256];
	while (off < end_offset) {
		off += read_line(off, tmp, sizeof(tmp));
		if (tmp[0] == '\0' || (tmp[0] == '/' && tmp[1] == '/'))
			continue;   /* 空行 / 注释不计数 */
		count++;
	}
	return count;
}

/* ==================== 上传 ==================== */
int FileIO_UploadStart(const char *name, uint32_t lines, uint32_t size)
{
	int _rc = fileio_storage_ready();
	if (_rc)
		return _rc;   /* 未焊 -4 / 容量不足 -5 */

	if (offline_state != OFFLINE_IDLE)
		return -1;   /* 忙 */

	/* [修复] size 上界校验：数据区 0x201000~0x3FFFFF 约 2MB，
	   超大 size 会导致循环擦百万扇区且地址 24 位回绕擦掉配置三副本区 */
	if (size > (0x400000UL - W25Q_ADDR_FILE_DATA)) {
		return -6;   /* 数据过大 */
	}

	memset(&file_dir, 0, sizeof(file_dir));
	strncpy((char *) file_dir.name, name, FILE_NAME_MAX - 1);
	file_dir.lines = lines;
	file_dir.size = size;
	file_dir.status = 0;

	/* 擦文件数据区相关扇区（按 size 向上取整到 4KB） */
	uint32_t sectors = (size + W25Q_SECTOR_SIZE - 1) / W25Q_SECTOR_SIZE;
	for (uint32_t i = 0; i < sectors; i++) {
		W25Q32_EraseSector(FILE_DATA_ADDR + i * W25Q_SECTOR_SIZE);
	}

	up_offset = 0;
	up_size = size;
	up_crc = crc32_init();
	offline_state = OFFLINE_UPLOAD;
	return 0;
}

int FileIO_UploadFeed(const uint8_t *data, uint16_t len)
{
	if (offline_state != OFFLINE_UPLOAD)
		return -1;

	/* 防止超过总字节数 */
	if (up_offset + len > up_size) {
		len = (uint16_t) (up_size - up_offset);
	}

	W25Q32_Write(FILE_DATA_ADDR + up_offset, data, len);
	up_crc = crc32_update(up_crc, data, len);
	up_offset += len;

	/* 用「总字节数」判定收满，不依赖 END 标记 */
	if (up_offset >= up_size) {
		file_dir.crc = crc32_final(up_crc);
		file_dir.status = 1;
		dir_write(&file_dir);
		offline_state = OFFLINE_IDLE;
		return 2;   /* 收满完成 */
	}
	return 0;   /* 继续 */
}

int FileIO_UploadFinish(void)
{
	if (offline_state != OFFLINE_UPLOAD)
		return -1;

	if (up_offset != up_size) {
		offline_state = OFFLINE_IDLE;
		return -2;   /* 字节数不符 */
	}

	file_dir.crc = crc32_final(up_crc);
	file_dir.status = 1;
	dir_write(&file_dir);
	offline_state = OFFLINE_IDLE;
	return 0;
}

uint8_t FileIO_IsUploading(void)
{
	return (offline_state == OFFLINE_UPLOAD) ? 1 : 0;
}

/* ==================== 执行 ==================== */
uint8_t FileIO_HasFile(void)
{
	return (file_dir.status == 1) ? 1 : 0;
}

int FileIO_Run(void)
{
	int _rc = fileio_storage_ready();
	if (_rc)
		return _rc;   /* 未焊 -4 / 容量不足 -5 */

	if (offline_state == OFFLINE_RUNNING || offline_state == OFFLINE_PAUSED)
		return -1;   /* 已在执行/暂停 */

	if (!FileIO_HasFile())
		return -2;   /* 无文件 */

	/* S17 前校验文件 CRC：读回数据区算 CRC 对比 */
	uint32_t crc = crc32_init();
	uint8_t buf[FILE_UPLOAD_PKT];
	uint32_t left = file_dir.size;
	uint32_t off = 0;
	while (left > 0) {
		uint32_t n = left > FILE_UPLOAD_PKT ? FILE_UPLOAD_PKT : left;
		W25Q32_Read(FILE_DATA_ADDR + off, buf, n);
		crc = crc32_update(crc, buf, n);
		off += n;
		left -= n;
	}
	if (crc32_final(crc) != file_dir.crc) {
		return -3;   /* 文件损坏 ERR_FILE */
	}

	run_offset = 0;
	run_line = 1;
	Progress_Clear();       /* 从头执行，清掉旧进度 */
	fileio_line_done = 1;   /* 触发 Service 立即喂第一行 */
	offline_state = OFFLINE_RUNNING;
	return 0;
}

int FileIO_Pause(void)
{
	if (offline_state == OFFLINE_RUNNING) {
		offline_state = OFFLINE_PAUSED;
		USART_printf(&huart1, "Paused %lu/%lu\n",
				(unsigned long) (run_line - 1), (unsigned long) file_dir.lines);
	}
	return 0;
}

int FileIO_Resume(void)
{
	if (offline_state == OFFLINE_PAUSED) {
		offline_state = OFFLINE_RUNNING;
		fileio_line_done = 1;
		return 0;
	}

	/* 断电恢复：从 Progress_Get 恢复断点（存的是文件偏移，非行号） */
	int _rc = fileio_storage_ready();
	if (_rc)
		return _rc;   /* 未焊 -4 / 容量不足 -5 */
	if (!FileIO_HasFile())
		return -2;
	if (Progress_HasResume()) {
		uint32_t resume_off = Progress_Get();
		if (resume_off >= file_dir.size) {
			resume_off = 0;   /* 偏移越界（文件被重新上传变小），从头执行 */
		}
		run_offset = resume_off;
		run_line = count_valid_lines(run_offset) + 1;   /* 反推有效行号（进度显示） */
		fileio_line_done = 1;
		offline_state = OFFLINE_RUNNING;
		MCodeSimple_SetInMcode(1); /* [修复] 断电恢复跳过首行 M90，须手动置 mcode 批模式，否则动作完成不发 OK 卡死 */
		return 0;
	}
	return -1;
}

int FileIO_Stop(void)
{
	offline_state = OFFLINE_IDLE;
	MCodeSimple_Abort();
	fileio_line_done = 0;
	preview_active = 0;
	preview_wait_ack = 0;
	Progress_Clear();   /* S0 急停：清空断电进度（下次 S19 不再从断点继续） */
	return 0;
}

/* ==================== 回读预览 ==================== */
int FileIO_Preview(void)
{
	int _rc = fileio_storage_ready();
	if (_rc)
		return _rc;   /* 未焊 -4 / 容量不足 -5 */

	if (!FileIO_HasFile())
		return -2;

	USART_printf(&huart1, "NAME:%s\n", file_dir.name);
	USART_printf(&huart1, "LINES:%lu\n", (unsigned long) file_dir.lines);
	USART_printf(&huart1, "SIZE:%lu\n", (unsigned long) file_dir.size);

	preview_active = 1;
	preview_offset = 0;
	preview_wait_ack = 1;   /* 等上位机 OK 后开始回数据 */
	return 0;
}

int FileIO_PreviewAck(void)
{
	preview_wait_ack = 0;   /* 上位机回 OK，可发下一包 */
	return 0;
}

/* ==================== 主循环服务 ==================== */
void FileIO_Service(void)
{
	/* 执行状态机 */
	if (offline_state == OFFLINE_RUNNING && fileio_line_done) {
		fileio_line_done = 0;

		/* 循环：跳过空行/注释，直到有效行或文件末尾 */
		while (1) {
			if (run_offset >= file_dir.size) {
				USART_printf(&huart1, "Done\n");
				offline_state = OFFLINE_IDLE;
				return;
			}

			/* [修复] 记录「当前行」的文件偏移：喂行前记下，喂完后保存这个值，
			   掉电恢复时重新执行当前行（原保存的是推进后的下一行偏移，会丢一行） */
			uint32_t line_start = run_offset;
			uint32_t len = read_line(run_offset, line_buf, sizeof(line_buf));
			run_offset += len;

			/* 跳过空行和 // 注释行（不喂 MCode、不计数） */
			if (line_buf[0] == '\0' || (line_buf[0] == '/' && line_buf[1] == '/')) {
				continue;
			}

			/* 有效行：喂 MCode + 回进度 + 计数 */
			MCodeSimple_ProcessBuffer(line_buf);
			USART_printf(&huart1, "%lu/%lu -> %s\n",
					(unsigned long) run_line, (unsigned long) file_dir.lines, line_buf);

			run_line++;
			/* 每执行完一行，把「当前行文件偏移」持久化进内部 Flash 预留页（断电恢复用） */
			g_resume_offset = line_start;
			Progress_Save(line_start);
			break;
		}
	}

	/* 回读状态机（分包 512B，逐包握手：发一包等上位机 OK） */
	if (preview_active && !preview_wait_ack) {
		uint32_t left = file_dir.size - preview_offset;
		if (left == 0) {
			USART_printf(&huart1, "Done\n");
			preview_active = 0;
		} else {
			uint32_t n = left > FILE_UPLOAD_PKT ? FILE_UPLOAD_PKT : left;
			uint8_t buf[FILE_UPLOAD_PKT];
			W25Q32_Read(FILE_DATA_ADDR + preview_offset, buf, n);
			/* 回传原始字节（可能含 \n 等），用 HAL_UART_Transmit 原样发 */
			HAL_UART_Transmit(&huart1, buf, (uint16_t) n, 1000);
			preview_offset += n;
			preview_wait_ack = 1;   /* 等上位机 OK 再发下一包 */
		}
	}
}

/* ==================== 串口行分发 ==================== */
int FileIO_ProcessLine(const char *line)
{
	if (line == NULL)
		return 0;

	/* 上传模式：收到的数据当文件内容喂，但「整行 S0」急停优先交给 system_command */
	if (offline_state == OFFLINE_UPLOAD) {
		const char *q = line;
		while (*q == ' ' || *q == '	')
			q++;
		/* [修复] 只放行整行 S0（急停），其余一律 UploadFeed。
		   原 line[0]=='S'/'s' 会把「120B 原始数据块首字节恰为 s」整块误当命令丢弃 → 字节错位/CRC 失败 */
		if (q[0] == 'S' && q[1] == '0'
				&& (q[2] == '\0' || q[2] == '\r' || q[2] == '\n')) {
			return 0;   /* 急停 S0：交给 MCodeSimple_ProcessBuffer -> system_command */
		}
		/* [修复] upload 态下「整行 END」也要识别为提前结束上传，
		   否则被当文件内容喂入、字节错位、上传卡死（原 END 兜底分支在 upload 态不可达） */
		if (q[0] == 'E' && q[1] == 'N' && q[2] == 'D'
				&& (q[3] == '\0' || q[3] == '\r' || q[3] == '\n')) {
			FileIO_UploadFinish();
			USART_printf(&huart1, "OK\n");
			return 1;
		}
		FileIO_UploadFeed((const uint8_t *) line, (uint16_t) strlen(line));
		USART_printf(&huart1, "OK\n");
		return 1;
	}

	/* UPLOAD,<文件名>,<总行数>,<总字节数> */
	if (strncmp(line, "UPLOAD,", 7) == 0) {
		const char *p = line + 7;
		char name[FILE_NAME_MAX];
		uint32_t lines = 0, size = 0;

		const char *c1 = strchr(p, ',');
		if (c1) {
			uint32_t nlen = (uint32_t) (c1 - p);
			if (nlen >= FILE_NAME_MAX)
				nlen = FILE_NAME_MAX - 1;
			memcpy(name, p, nlen);
			name[nlen] = '\0';

			lines = (uint32_t) strtoul(c1 + 1, NULL, 10);
			const char *c2 = strchr(c1 + 1, ',');
			if (c2) {
				size = (uint32_t) strtoul(c2 + 1, NULL, 10);
			}
		}

		if (FileIO_UploadStart(name, lines, size) == 0) {
			USART_printf(&huart1, "OK\n");
		} else {
			USART_printf(&huart1, "ERR_BUSY\n");
		}
		return 1;
	}

	/* END（备用兜底结束上传） */
	if (strncmp(line, "END", 3) == 0) {
		FileIO_UploadFinish();
		USART_printf(&huart1, "OK\n");
		return 1;
	}

	/* OK（预览回读 ACK） */
	if (strncmp(line, "OK", 2) == 0) {
		FileIO_PreviewAck();
		return 1;
	}

	return 0;   /* 未处理，交给 MCodeSimple_ProcessBuffer */
}
