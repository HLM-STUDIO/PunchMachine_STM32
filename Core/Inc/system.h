#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "mcode.h"

/* system.h: 修改后的结构体（自然对齐） */
#define CONFIG_VERSION  110  /* [修改] 末尾追加 12 个 PID 参数字段（S51-S62），升版本走迁移 */

typedef struct {
    uint16_t magic;          // 魔数
    uint16_t version;        // 版本

    uint16_t move_base;
    uint16_t paper_base;
    uint16_t punch_delay;
    uint16_t software_max;
    uint16_t handle_thres;
    uint16_t handle_x_mid;
    uint16_t handle_y_mid;
    uint16_t home_bias;
    uint16_t home_per_points;//每打多少孔自动homing一下
    uint16_t punch_lock_move;// [新增] 打孔时锁定 move 编码器计数(1=锁死避免震动毛刺,0=不锁)
    uint16_t note_spacing;   // [新增] 30 音间距，单位 mm×1000（2000=2.000mm）
    uint16_t note_spacing_15;// [新增] 15 音间距，单位 mm×1000
    uint16_t note_spacing_20;// [新增] 20 音间距，单位 mm×1000
    uint16_t note_spacing_40;// [新增] 40 音间距，单位 mm×1000
    uint16_t note_spacing_50;// [新增] 50 音间距，单位 mm×1000
    uint16_t note_count;     // [新增] 当前音数（15/20/30/40/50，默认 30）
    uint16_t calibrated;     // [新增] 是否已标定（0=未标定首次开机自动标定，1=已标定）
    uint16_t language;       // [新增] 语言（0=英文 1=中文，S21 切换）
    uint16_t tick_mul;
    uint16_t tick_mem;
    uint16_t move_dead_band; // [新增] 移动到位允许误差（脉冲），S84 设置，0=精确对齐
    uint16_t calib_sw_margin; // [新增] 标定余量（mm×100，200=2.00mm），S86 设置；复用原 _pad2 对齐位

    uint32_t punch_total;    // 累计打孔数（满 home_per_points 回零）
    uint32_t move_total;     // 累计主轴行程，单位 mm*100（两位小数）

    uint16_t checksum;
    uint16_t _pad;           // 保证 sizeof 是 4 的倍数
    uint16_t encoder_filter; // [新增] 编码器数字滤波(0-15)，S87 设置，默认 15；末尾追加（未来新字段继续追加在此之后）
    uint16_t _pad2;          // [修复] 显式 padding：末尾 uint16 凑偶数，消除隐式 padding（未初始化字节被计入 crc/checksum 的隐患）
    uint16_t move_kp;        // [新增] move 位置环 P（×100，3000=30.00），S51
    uint16_t move_kd;        // [新增] move 位置环 D（×100，100=1.00），S52
    uint16_t move_spd_kp;    // [新增] move 速度环 P（×100，150=1.50），S53
    uint16_t move_spd_ki;    // [新增] move 速度环 I（×1000，30=0.030），S54
    uint16_t paper_kp;       // [新增] paper 位置环 P（×100，4000=40.00），S55
    uint16_t paper_kd;       // [新增] paper 位置环 D（×100，150=1.50），S56
    uint16_t paper_spd_kp;   // [新增] paper 速度环 P（×100，120=1.20），S57
    uint16_t paper_spd_ki;   // [新增] paper 速度环 I（×1000，20=0.020），S58
    uint16_t paper_dead_band;// [新增] paper 到位容忍（脉冲，默认 2），S59
    uint16_t homing_coarse;  // [新增] 回零粗靠 PWM 加值（默认 640），S60
    uint16_t homing_fine;    // 回零细靠 PWM 加值（默认 100，勿<30 堵转），S61
    uint16_t homing_backoff; // 回零退开 PWM 加值（默认 200），S62
} SystemConfig_t;


extern SystemConfig_t current_config;
extern uint8_t pid_save_enable; /* [新增] PID 保存开关（S63）：0=试验只RAM，1=掉电保存；开关本身不持久化 */
extern volatile uint8_t save_config_request; /* [新增] 主循环保存配置请求(1=请求写Flash) */
extern volatile uint8_t notes_recalc_request; /* [新增] 音数/间距重算请求（中断置标志，主循环执行 Notes_Init） */
extern volatile uint8_t calib_is_factory; /* [新增] 1=出厂序列标定（Calib_* 不置 calibrated），0=单步标定 */

/* [新增] 中英文切换宏：根据 current_config.language 选择中文/英文。
   用于「给人看」的提示文本；协议文本(OK/Done/NAME/SIZE 等上位机解析依赖)不用此宏。 */
#define L(cn, en)  ((current_config.language) ? (cn) : (en))

/* [新增] S 命令长文本延迟打印请求（在主循环打印，避免 1ms 中断里阻塞串口） */
typedef enum {
	PRINT_NONE = 0,
	PRINT_HELP,     /* S90 帮助信息 */
	PRINT_CONFIG,   /* S10 系统配置 */
	PRINT_W25TEST,  /* S30 存储器读写自测 */
	PRINT_W25ERASE, /* S31 存储器全片擦除 */
	PRINT_TRAVEL_WARN, /* [新增] 行程不足警告（音数×间距超可用行程） */
} print_request_t;
extern volatile print_request_t print_request;

/* [新增] 动态延迟打印：中断里（SelfTest/FactoryCalib 进度）只设置字符串指针，
   主循环打印。避免 1ms 中断里 USART_printf 与主循环并发导致 static buffer 覆盖乱码。
   指针指向字符串字面量（编译期常量，安全，无需复制）。 */
extern volatile const char *deferred_print_text;

void showHelpInfo();
void system_command(int S_id, uint8_t has_val, int32_t val);
void System_ApplyNoteConfig(void); /* [新增] 应用音数+间距配置到 notes 并重算定位点 */

int ValidateConfig(SystemConfig_t *config);
int MigrateConfig(SystemConfig_t *cfg); /* [新增] 配置迁移（版本分支链）：旧字段保留，新字段填默认 */
uint16_t CalculateChecksum(SystemConfig_t *config);
int LoadSystemConfig(void);
int SaveSystemConfig(void);
void PrintSystemConfig(void);
void PrintTravelWarning(void); /* [新增] 行程不足警告打印 */
void ModifyConfigExample(void);
void TestFlashFunction(void);

#endif /* INC_SYSTEM_H_ */
