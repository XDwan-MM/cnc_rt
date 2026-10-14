#pragma once
/* 台架本地 profile —— RT 接入 HAL 的唯一硬件配置来源。
 * 只描述硬件事实（轴在哪个从站、当量、IO 地址），机床策略不在这里。
 * 只依赖 HAL 公共头，便于独立编译校验。 */

#include <stdint.h>
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* profile 标识：启动时打印，区分现场跑的哪一版 */
#define RT_HAL_PROFILE_NAME "bench-6slave"
#define RT_HAL_PROFILE_REV  1

/**
 * @brief RT 轴槽
 *
 * 与 AXIS_TYPE 及 HAL 逻辑轴号**同序**（X=0/Y=1/Z=2/主轴=3），无换算。
 * 不复用 AXIS_TYPE 是为了本模块能不引 RT 内部头单独编译。
 */
typedef enum {
    RT_HAL_AXIS_X = 0,
    RT_HAL_AXIS_Y = 1,
    RT_HAL_AXIS_Z = 2,
    RT_HAL_AXIS_SP = 3,
    RT_HAL_AXIS_COUNT = 4
} RtHalAxisSlot;

/**
 * @brief 一条轴的绑定：RT 轴槽 ↔ 物理从站 ↔ HAL 逻辑轴号
 */
typedef struct {
    const char* name;       /* 诊断短名 */
    int         slave_pos;  /* 从站位置 */
    int         axis_index; /* 从站内轴序号，从 0 起 */
    int         hal_axis;   /* HalAxisId；与行号相同，见 rt_hal_config.c */
    int         is_spindle; /* 非零 = 主轴，须同时出现在 spindles[] 里 */
} RtHalBinding;

/* ---- 主轴定向：蓝海华腾厂商专有，本地 profile 配 ----
 * 控制字 0x040F = 保持此字直到状态字 bit10 置起；完成位 0x0400 = 状态字 bit10。
 * 手册依据（VTS 技术手册 附录 C）：发 0x40F 后须延时再查 6041H.bit10，
 * 此时 bit10 才是定位完成标志。 */
#define RT_PLACEHOLDER_ORIENT_CW   0x040Fu
#define RT_PLACEHOLDER_ORIENT_DONE 0x0400u

/* 定向控制字发出后先等这么久再采完成位（ms）。
 * 6041H.bit10 在 CSV 模式下是"速度达到设定转速"，不等就会把到速误当定位完成。
 * **换驱动器只改这个值（0 = 不等），不要在代码里写"如果是某品牌就……"。** */
#define RT_ORIENT_DELAY_MS 10

/* ---- 速度对象单位（每轴）----
 * 本台架四轴都用 rpm（蓝海华腾须把 C2-00 设成 0，出厂默认 pulse/s）。
 * 某轴驱动器只能是 pulse/s 时改成 HAL_SPEED_PULSES_PER_S；换算常数由 pm_pulse 传入。
 * **只改值，不写"如果是某品牌就……"。** */
static const int kSpeedUnit[RT_HAL_AXIS_COUNT] = {
    HAL_SPEED_RPM, HAL_SPEED_RPM, HAL_SPEED_RPM, HAL_SPEED_RPM,
};

/* ---- 未确认参数台账 ----
 * **参数域没给全就挡住运动**，不猜值。非空时 motion_allowed() 返回 0；
 * build() 置初值，set_scaling() 按轴清位。 */
#define RT_HAL_TODO_SCALING  (1u << 0)  /* 命令/反馈当量、方向 */
#define RT_HAL_TODO_ENCODER  (1u << 1)  /* 编码器类型、每转反馈计数、feedback_wrap */
#define RT_HAL_TODO_ENC_OFF  (1u << 2)  /* enc_off 坐标偏置 */
#define RT_HAL_TODO_ESTOP    (1u << 4)  /* 急停动作；用户已定为去使能，未置起 */
/* bit3、bit5 是已删位，号位留空不补：免得旧日志里同一个掩码被当成另一件事。 */

/* ---- 台账查询 ---- */

/**
 * @brief 读当前未确认台账掩码
 *
 * 只读。台账只反映"参数域给全了没有"；`RT_HAL_TODO_ESTOP` 不在列。
 *
 * @return uint32_t `RT_HAL_TODO_*` 的按位或；0 = 全清，可以开放运动
 */
uint32_t rt_hal_config_todo(void);

/**
 * @brief 把一个台账位翻成给人看的文本（启动打印用）
 *
 * @param bit 单个 `RT_HAL_TODO_*` 位，**不是掩码**
 * @return const char* 静态字符串；未知位返回 "未知项"
 */
const char* rt_hal_config_todo_text(uint32_t bit);

/**
 * @brief 未确认台账里置起了几项
 *
 * @return int 置起的位数（0 = 全清）
 */
int rt_hal_config_todo_count(void);

/**
 * @brief 该轴的换算量（当量 + enc_off + 每转计数）是否都已装载
 *
 * 只看本轴有没有装真值，不看别的轴。
 *
 * @param slot RT 轴槽
 * @return int 1 = 已装载；0 = 仍是占位值，或槽号越界
 */
int rt_hal_config_scaling_loaded(RtHalAxisSlot slot);

/**
 * @brief 未确认项是否全清 —— 运动的总闸门
 *
 * 台账非空返回 0，RT 侧据此挡住运动请求（使能、写位置、写转速、写模式）。
 * 定向、急停、故障复位不走这个判据。
 *
 * @return int 1 = 台账全清，可以开放运动；0 = 还有未确认项
 */
int rt_hal_config_motion_allowed(void);

/**
 * @brief RT 轴槽 → HAL 逻辑轴号
 *
 * 当前是恒等映射，但**跨到 HAL 的编号一律走这里**，调用点不要自己写数字。
 *
 * @param slot RT 轴槽
 * @return int HalAxisId；槽号越界返回 -1
 */
int rt_hal_config_hal_axis(RtHalAxisSlot slot);

/**
 * @brief 该轴槽是不是主轴
 *
 * 判据唯一来源是绑定表第 5 列。主轴专用读接口须先过这一关：快照的主轴数组只填主轴槽，
 * 读非主轴会拿到"成功 + 0"，分不出"停着"和"压根没这个数据"。
 *
 * @param slot RT 轴槽
 * @return int 1 = 主轴；0 = 不是主轴，或槽号越界
 */
int rt_hal_config_is_spindle(RtHalAxisSlot slot);

/* ---- 装配 ---- */

/**
 * @brief 用本 profile 填 *out（只装配，不校验）
 *
 * @param out     装配结果
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度；err 非空且 err_len > 0 时保证 NUL 结尾
 * @return int 0 = 成功；RT_HAL_ERR_NULL_ARG（ABI 不符由编译期 #error 拦下）
 */
int rt_hal_config_build(HalCConfig* out, char* err, uint32_t err_len);

/**
 * @brief 打印 profile 标识、HAL ABI、周期、轴↔从站绑定、IO 区间、未确认台账
 *
 * 在启动路径上（rt_hal_create）打一次：没硬件时也能先看配置对不对。
 *
 * @param cfg 已 build 的配置；为空则直接返回
 */
void rt_hal_config_print(const HalCConfig* cfg);

/**
 * @brief 校验 + 打印，用于 build/set_scaling 之后的收口
 *
 * @param cfg     已 build 的配置
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度
 * @return int 0 = 成功；RT_HAL_ERR_NULL_ARG；RT_HAL_ERR_ORDER = 没先 build；
 *             其余为 HAL 的错误码（见 hal_config_api.h）
 */
int rt_hal_config_finish(HalCConfig* cfg, char* err, uint32_t err_len);

/**
 * @brief 用 profile 的**占位值**装配一套配置并校验（参数域为空时的入口）
 *
 * 等价于 build() 之后立刻 finish()：当量、enc_off、每转计数、主轴上限全是占位值，
 * 台账挂满，motion_allowed() 返回 0。
 *
 * **不是 RT 的启动入口**：装机启动走 rt_hal_create()。本函数只验 profile 能否过 HAL 校验。
 *
 * @param out     装配结果
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度
 * @return int 见 rt_hal_config_build() 与 rt_hal_config_finish()
 */
int rt_hal_config_build_and_check(HalCConfig* out, char* err, uint32_t err_len);

/* ---- 从机床参数装载换算量 ----
 * 换算量来自共享内存 par_axis，由 RT 侧取值传。
 * 顺序：build() → set_scaling() × 轴数 → finish()。
 * 没参数的轴不要调用：它保持占位值并挂台账。 */

/**
 * @brief 一根轴的换算量，由 RT 从自己的参数域取好再传进来
 */
typedef struct {
    double units_per_count;          /* 用户单位/计数，必须 > 0 且有限 */
    double enc_off;                  /* 坐标偏置，语义与 RT 的 enc_off 一致：
                                        直接传**不要取反**（HAL 内部存成 −enc_off） */
    int    command_invert;           /* 命令方向取反 */
    int    feedback_invert;          /* 反馈方向取反；外接光栅尺可能只需翻反馈 */
    int    feedback_pulses_per_rev;  /* 模循环展开用，来自 PAR_AX_MT_PPR；
                                        为 0 时调用方退回 pm_pulse，都不要时传 0 */
    int    command_pulses_per_rev;   /* 每转指令脉冲数（PAR_AX_PM_PULSE）；
                                        仅该轴 speed_unit 为 pulse/s 时参与换算 */
} RtHalAxisScaling;

/**
 * @brief 把换算量装进已 build 的配置（按轴）
 *
 * 先全部校验再落字段。装载成功清掉该轴 SCALING / ENC_OFF 台账位并重算掩码。
 *
 * @param cfg  已 build 的配置
 * @param slot RT 轴槽
 * @param s    该轴的换算量
 * @return int 0 = 成功；RT_HAL_ERR_NULL_ARG；RT_HAL_ERR_ORDER = 没先 build；
 *             RT_HAL_ERR_SLOT = 槽号越界或在配置里找不到该轴；
 *             RT_HAL_ERR_PARAM = 字段非法（**配置一个字节都没动**）
 */
int rt_hal_config_set_scaling(HalCConfig* cfg, RtHalAxisSlot slot, const RtHalAxisScaling* s);

/**
 * @brief 装载主轴转速上限（rpm）
 *
 * HAL 拿它做下发前的量程检查：超上限的转速指令被 HAL 直接拒收，RT 侧看不出来。
 * 不装载时保持占位值。
 *
 * @param cfg     已 build 的配置
 * @param max_rpm 上限（rpm），必须有限且 > 0
 * @return int 0 = 成功；RT_HAL_ERR_NULL_ARG；RT_HAL_ERR_ORDER = 没先 build；
 *             RT_HAL_ERR_PARAM = 非法（配置不变）
 */
int rt_hal_config_set_spindle_max(HalCConfig* cfg, double max_rpm);

/* ---- RT 内部错误码（都是负值；HAL 的错误码是正值，别混）---- */
#define RT_HAL_ERR_NULL_ARG   (-1)  /* 传入空指针 */
#define RT_HAL_ERR_TOO_SMALL  (-2)  /* err 缓冲放不下 */
#define RT_HAL_ERR_ABI        (-3)  /* HAL 公共头 ABI 与预期不一致；现由编译期 #error 拦，
                                       本码不由代码返回，留作占位 */
#define RT_HAL_ERR_PARAM      (-4)  /* 参数值非法 */
#define RT_HAL_ERR_ORDER      (-5)  /* 调用顺序错：未先 build */
#define RT_HAL_ERR_SLOT       (-6)  /* 轴槽号越界 */

#ifdef __cplusplus
}
#endif
