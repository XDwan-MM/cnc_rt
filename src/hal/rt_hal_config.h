#pragma once
/* 台架本地 profile —— RT 接入 HAL 的唯一硬件配置来源；只描述硬件事实（从站、当量、IO 地址），机床策略不在此。
 * 只依赖 HAL 公共头，可独立编译校验。 */

#include <stdint.h>
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* profile 标识：启动打印，区分现场版本 */
#define RT_HAL_PROFILE_NAME "bench-6slave"
#define RT_HAL_PROFILE_REV  1

/* RT 轴槽。与 AXIS_TYPE 及 HAL 逻辑轴号同序（X=0/Y=1/Z=2/主轴=3），无换算；不复用 AXIS_TYPE 以便独立编译。 */
typedef enum {
    RT_HAL_AXIS_X = 0,
    RT_HAL_AXIS_Y = 1,
    RT_HAL_AXIS_Z = 2,
    RT_HAL_AXIS_SP = 3,
    RT_HAL_AXIS_COUNT = 4
} RtHalAxisSlot;

/* 一条轴的绑定：RT 轴槽 ↔ 物理从站 ↔ HAL 逻辑轴号 */
typedef struct {
    const char* name;       /* 诊断短名 */
    int         slave_pos;
    int         axis_index; /* 从站内轴序号，从 0 起 */
    int         hal_axis;   /* HalAxisId，须与行号相同（见 rt_hal_config.c） */
    int         is_spindle; /* 非零 = 主轴，须同时出现在 spindles[] 里 */
} RtHalBinding;

/* ---- 主轴定向：蓝海华腾厂商专有，本地 profile 配 ----
 * 控制字 0x040F 保持到状态字 bit10 置起；完成位 0x0400 = 状态字 bit10（6041H.bit10）。 */
#define RT_PLACEHOLDER_ORIENT_CW   0x040Fu
#define RT_PLACEHOLDER_ORIENT_DONE 0x0400u

/* 定向控制字发出后先等这么久再采完成位（ms；0 = 不等）。6041H.bit10 在 CSV 下是"速度到达设定转速"，
 * 不等会把到速误当定位完成。换驱动器只改这个值。 */
#define RT_ORIENT_DELAY_MS 10

/* ---- 速度对象单位（每轴）----
 * 本台架四轴都是 rpm（蓝海华腾须把 C2-00 设成 0，出厂默认 pulse/s）；某轴只能 pulse/s 时改 HAL_SPEED_PULSES_PER_S，
 * 换算常数由 pm_pulse 传入。 */
static const int kSpeedUnit[RT_HAL_AXIS_COUNT] = {
    HAL_SPEED_RPM, HAL_SPEED_RPM, HAL_SPEED_RPM, HAL_SPEED_RPM,
};

/* ---- 未确认参数台账：参数域没给全就挡住运动，不猜值。非空时 motion_allowed() 返回 0；
 * build() 置初值，set_scaling() 按轴清位。 */
#define RT_HAL_TODO_SCALING  (1u << 0)  /* 命令/反馈当量、方向 */
#define RT_HAL_TODO_ENCODER  (1u << 1)  /* 编码器类型、每转反馈计数、feedback_wrap */
#define RT_HAL_TODO_ENC_OFF  (1u << 2)  /* enc_off 坐标偏置 */
#define RT_HAL_TODO_ESTOP    (1u << 4)  /* 急停动作（定为去使能）；当前未置起 */
/* bit3、bit5 是已删位，号位留空不补（免得旧日志把同一掩码当成另一件事）。 */

/* ---- 台账查询 ---- */

/* 读当前未确认台账掩码（只读）。台账只反映"参数域给全了没有"；RT_HAL_TODO_ESTOP 不在列。
 * 返回 RT_HAL_TODO_* 的按位或；0 = 全清，可以开放运动。 */
uint32_t rt_hal_config_todo(void);

/* 把一个台账位翻成给人看的文本（启动打印用）；bit 是单个 RT_HAL_TODO_* 位，不是掩码。
 * 返回静态字符串；未知位返回 "未知项"。 */
const char* rt_hal_config_todo_text(uint32_t bit);

/* 未确认台账里置起了几项；0 = 全清 */
int rt_hal_config_todo_count(void);

/* 该轴的换算量（当量 + enc_off + 每转计数）是否都已装载；只看本轴，不看别的轴。
 * 1 = 已装载；0 = 仍是占位值或槽号越界。 */
int rt_hal_config_scaling_loaded(RtHalAxisSlot slot);

/* 未确认项是否全清 —— 运动总闸门：台账非空返回 0，RT 侧据此挡住运动请求（使能、写位置、写转速、写模式）；
 * 定向、急停、故障复位不走这个判据。1 = 台账全清、可以开放运动；0 = 还有未确认项。 */
int rt_hal_config_motion_allowed(void);

/* RT 轴槽 → HAL 逻辑轴号。当前恒等映射，但跨到 HAL 的编号一律走这里，调用点不要自己写数字。
 * 槽号越界返回 -1。 */
int rt_hal_config_hal_axis(RtHalAxisSlot slot);

/* 该轴槽是不是主轴；判据唯一来源是绑定表 is_spindle 列。主轴专用读接口须先过这一关：快照主轴数组只填主轴槽，
 * 读非主轴会拿到"成功 + 0"，分不出"停着"和"压根没这个数据"。1 = 主轴；0 = 不是主轴或槽号越界。 */
int rt_hal_config_is_spindle(RtHalAxisSlot slot);

/* ---- 装配 ---- */

/* 用本 profile 填 *out（只装配，不校验）；err 非空且 err_len > 0 时保证 NUL 结尾。
 * 0 = 成功；RT_HAL_ERR_NULL_ARG（ABI 不符由编译期 #error 拦下）。 */
int rt_hal_config_build(HalCConfig* out, char* err, uint32_t err_len);

/* 打印 profile 标识、HAL ABI、周期、轴↔从站绑定、IO 区间、未确认台账；cfg 为空直接返回。
 * 在启动路径（rt_hal_create）打一次：没硬件也能先看配置对不对。 */
void rt_hal_config_print(const HalCConfig* cfg);

/* 校验 + 打印，用于 build / set_scaling 之后的收口。0 = 成功；RT_HAL_ERR_NULL_ARG；
 * RT_HAL_ERR_ORDER = 没先 build；其余为 HAL 错误码（见 hal_config_api.h）。 */
int rt_hal_config_finish(HalCConfig* cfg, char* err, uint32_t err_len);

/* 用 profile 占位值装配一套配置并校验（参数域为空时的入口，= build() 后立刻 finish()）：当量、enc_off、
 * 每转计数、主轴上限全占位，台账挂满，motion_allowed() 返回 0。不是 RT 启动入口（那走 rt_hal_create()），
 * 只验 profile 能否过 HAL 校验。返回码见 rt_hal_config_build() 与 rt_hal_config_finish()。 */
int rt_hal_config_build_and_check(HalCConfig* out, char* err, uint32_t err_len);

/* ---- 从机床参数装载换算量：换算量来自共享内存 par_axis，由 RT 侧取值传入。
 * 顺序 build() → set_scaling() × 轴数 → finish()；没参数的轴不要调用（保持占位值并挂台账）。 */

/* 一根轴的换算量，由 RT 从参数域取好传入 */
typedef struct {
    double units_per_count;          /* 用户单位/计数，必须 > 0 且有限 */
    double enc_off;                  /* 坐标偏置，语义同 RT 的 enc_off：直接传不要取反（HAL 内部存成 −enc_off） */
    int    command_invert;           /* 命令方向取反 */
    int    feedback_invert;          /* 反馈方向取反；外接光栅尺可能只需翻反馈 */
    int    feedback_pulses_per_rev;  /* 模循环展开用（PAR_AX_MT_PPR）；0 = 调用方退回 pm_pulse，都不要时传 0 */
    int    command_pulses_per_rev;   /* 每转指令脉冲数（PAR_AX_PM_PULSE）；仅该轴 speed_unit 为 pulse/s 时参与换算 */
} RtHalAxisScaling;

/* 把换算量装进已 build 的配置（按轴）；先全部校验再落字段，装载成功清掉该轴 SCALING / ENC_OFF 台账位并重算掩码。
 * 0 = 成功；RT_HAL_ERR_NULL_ARG；RT_HAL_ERR_ORDER = 没先 build；RT_HAL_ERR_SLOT = 槽号越界或配置里找不到该轴；
 * RT_HAL_ERR_PARAM = 字段非法（配置一个字节都没动）。 */
int rt_hal_config_set_scaling(HalCConfig* cfg, RtHalAxisSlot slot, const RtHalAxisScaling* s);

/* 装载主轴转速上限（rpm，须有限且 > 0）；HAL 拿它做下发前量程检查——超上限的转速指令被 HAL 直接拒收，
 * RT 侧看不出来；不装载时保持占位值。0 = 成功；RT_HAL_ERR_NULL_ARG；RT_HAL_ERR_ORDER = 没先 build；RT_HAL_ERR_PARAM = 非法（配置不变）。 */
int rt_hal_config_set_spindle_max(HalCConfig* cfg, double max_rpm);

/* ---- RT 内部错误码（都是负值；HAL 的错误码是正值，别混）---- */
#define RT_HAL_ERR_NULL_ARG   (-1)  /* 传入空指针 */
#define RT_HAL_ERR_TOO_SMALL  (-2)  /* err 缓冲放不下 */
#define RT_HAL_ERR_ABI        (-3)  /* HAL 公共头 ABI 与预期不一致；现由编译期 #error 拦，本码不由代码返回，留作占位 */
#define RT_HAL_ERR_PARAM      (-4)  /* 参数值非法 */
#define RT_HAL_ERR_ORDER      (-5)  /* 调用顺序错：未先 build */
#define RT_HAL_ERR_SLOT       (-6)  /* 轴槽号越界 */

#ifdef __cplusplus
}
#endif
