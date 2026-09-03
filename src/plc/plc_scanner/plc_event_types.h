#ifndef PLC_EVENT_TYPES_H
#define PLC_EVENT_TYPES_H

#include "regdata.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief CNC 运行模式枚举
 * 映射逻辑：基于 MD1(bit0), MD2(bit1), MD4(bit2), DNCI(bit5), ZRN(bit7) 的组合
 */
typedef enum
{
    CNC_MODE_INVALID = 0, /**< 无效模式 */
    CNC_MODE_EDIT = 1,    /**< 编辑 */
    CNC_MODE_MEM = 2,     /**< 存储器 */
    CNC_MODE_MDI = 3,     /**< MDI */
    CNC_MODE_JOG = 4,     /**< JOG */
    CNC_MODE_MPG = 5,     /**< 手轮 */
    CNC_MODE_ZRN = 6,     /**< 回零 */
    CNC_MODE_DNC = 7,     /**< DNC运行 */
} CNC_MODE_INDEX;

/**
 * @brief PLC事件 ID
 */
typedef enum
{
    PLC_EV_MSG_NONE = 0,
    PLC_EV_MSG_MODE,         /**< 模式切换 (参数1: 模式索引) */
    PLC_EV_MSG_FEEDAXIS_KEY, /**< 进给轴按键事件 (参数1: 轴号, 参数2: 方向) */
    PLC_EV_MSG_SPINDLE,      /**< 主轴控制 (参数1: 方向) */
    PLC_EV_MSG_OVERRIDE,     /**< 倍率调节 (参数1: 倍率类型, 参数2: 数值) */
    PLC_EV_MSG_AUTO_RUN,     /**< 运行控制 (参数1: 启动/中止) */
    PLC_EV_MSG_RESET,        /**< 系统复位 */
    PLC_EV_MSG_ENANLE,
    PLC_EV_MSG_SPD_ORI,        /**< 主轴定向 */
    PLC_EV_MSG_EXTERNAL_DECEL, /**< 外部减速事件 */

    // todo: 后续HMI按键事件得从plc事件分开
    PLC_EV_MSG_HMI, /**< HMI按键事件 */
} PLC_EVENT_ID;

/**
 * @brief 外部减速功能编号
 * 每类功能分别由一组正向和负向 G 信号控制。
 */
typedef enum
{
    PLC_EXTERNAL_DECEL_FUNCTION_1 = 1,
    PLC_EXTERNAL_DECEL_FUNCTION_2,
    PLC_EXTERNAL_DECEL_FUNCTION_3,
    PLC_EXTERNAL_DECEL_FUNCTION_4,
    PLC_EXTERNAL_DECEL_FUNCTION_5,
} PLC_EXTERNAL_DECEL_FUNCTION;

typedef enum
{
    MSBT_LOCK = 0,  // 辅助功能锁住
    DRY_RUN,        // 空运行
    SINGLE_BLOCK,   // 单段
    BLOCK_SKIP = 3, // 程序段跳过（1~9）
    MAC_LOCK = 12,  // 所有轴机床锁住（MLK,MLK1~4）
    MAC_LOCK1,      // 各轴轴机床锁住
    MAC_LOCK2,
    MAC_LOCK3,
    MAC_LOCK4,
} PLC_ENABLE_KEY;

// 各个PLC事件对应的信息负载结构体定义
typedef struct
{
    int mode;
} PLC_EV_MSG_MODE_PAYLOAD;

typedef struct
{
    int axis_num;
    int direction;
} PLC_EV_MSG_FEEDAXIS_KEY_PAYLOAD;

typedef struct
{
    int direction;
} PLC_EV_MSG_SPINDLE_PAYLOAD;

typedef struct
{
    int override_type;
    int val;
} PLC_EV_MSG_OVERRIDE_PAYLOAD;

typedef struct
{
    int run_state;
} PLC_EV_MSG_AUTO_RUN_PAYLOAD;

typedef struct
{
    int reset_state;
    int reset_type;
} PLC_EV_MSG_RESET_PAYLOAD;

typedef struct
{
    int enable_key;
    int enable_state;
} PLC_EV_MSG_ENABLE_PAYLOAD;

typedef struct
{
    int p1;
    int p2;
} PLC_EV_MSG_SPD_ORI_PAYLOAD;

/**
 * @brief 外部减速事件信息负载
 * function 减速功能好
 * axis_num 逻辑轴编号
 * direction 使用 +1/-1 表示正/负方向
 * active 为1表示对应G位为0、减速生效
 */
typedef struct
{
    int function;
    int axis_num;
    int direction;
    int active;
} PLC_EV_MSG_EXTERNAL_DECEL_PAYLOAD;

// todo: 后续HMI按键事件得从plc事件分开
typedef struct
{
    int press_key;
} PLC_EV_MSG_HMI_PAYLOAD;

/**
 * @brief CNC 事件通用结构
 * 用于在内存中统一事件格式，方便解析
 */
typedef struct
{
    PLC_EVENT_ID id; /**< 事件代码 */
    union
    {
        PLC_EV_MSG_MODE_PAYLOAD mode_payload;
        PLC_EV_MSG_FEEDAXIS_KEY_PAYLOAD feedaxis_key_payload;
        PLC_EV_MSG_SPINDLE_PAYLOAD spindle_payload;
        PLC_EV_MSG_OVERRIDE_PAYLOAD override_payload;
        PLC_EV_MSG_AUTO_RUN_PAYLOAD run_payload;
        PLC_EV_MSG_RESET_PAYLOAD reset_payload;
        PLC_EV_MSG_ENABLE_PAYLOAD enable_payload;
        PLC_EV_MSG_SPD_ORI_PAYLOAD spd_ori_payload;
        PLC_EV_MSG_EXTERNAL_DECEL_PAYLOAD external_decel_payload;

        // todo: 后续HMI按键事件得从plc事件分开
        PLC_EV_MSG_HMI_PAYLOAD hmi_payload;
    };
} PLC_EVENT;

#pragma pack(push, 1)
/**
 * @brief G 变量本地快照映射
 * 采用数组方式管理。
 * g_regs 明确指代 PLC 中的 G 寄存器数据 (PLC -> CNC)。
 * 索引值即为 G 寄存器的偏移量，例如 g_regs[12] 对应 G0012。
 * REG_G_NUM: 共享内存支持的最大G寄存器索引大小
 */
typedef struct
{
    char g_regs[REG_G_NUM];
} G_INPUT_MAP;

/**
 * @brief X 变量本地快照映射
 * 采用数组方式管理。
 * x_regs 明确指代 PLC 中的 X 寄存器数据 (机床 -> PLC)。
 * 索引值即为 X 寄存器的偏移量，例如 x_regs[8] 对应 X0008。
 * REG_X_NUM: 共享内存支持的最大 X 寄存器索引大小
 */
typedef struct
{
    char x_regs[REG_X_NUM];
} X_INPUT_MAP;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* PLC_EVENT_TYPES_H */
