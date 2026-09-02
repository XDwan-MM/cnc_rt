#ifndef PMC_IO_MAP_H
#define PMC_IO_MAP_H

#include "regdata.h"

/**
 * @brief CNC 运行模式枚举
 * 映射逻辑：基于 MD1(bit0), MD2(bit1), MD4(bit2), DNCI(bit5), ZRN(bit7) 的组合
 */
typedef enum {
    CNC_MODE_INVALID = 0,  /**< 无效模式 */
    CNC_MODE_EDIT = 1,    /**< 编辑 */
    CNC_MODE_MEM  = 2,    /**< 存储器 */
    CNC_MODE_MDI  = 3,    /**< MDI */
    CNC_MODE_JOG  = 4,    /**< JOG */
    CNC_MODE_MPG  = 5,    /**< 手轮 */
    CNC_MODE_ZRN  = 6,    /**< 回零 */
    CNC_MODE_DNC  = 7,    /**< DNC运行 */
} CNC_MODE_INDEX;

/**
 * @brief PMC事件 ID
 * 对应 EVENT_DATA 中的 code 字段，用于区分不同的动作类型
 */
typedef enum {
    EV_MSG_NONE = 0,
    EV_MSG_MODE,        /**< 模式切换 (参数1: 模式索引) */
    EV_MSG_FEEDAXIS_KEY,/**< 进给轴按键事件 (参数1: 轴号, 参数2: 方向) */
    EV_MSG_SPINDLE,     /**< 主轴控制 (参数1: 方向) */
    EV_MSG_OVERRIDE,    /**< 倍率调节 (参数1: 倍率类型, 参数2: 数值) */
    EV_MSG_AUTO_RUN,    /**< 运行控制 (参数1: 启动/中止) */
    EV_MSG_RESET,       /**< 系统复位 */
    EV_MSG_ENANLE,
    EV_MSG_SPD_ORI,     /**< 主轴定向 */
    EV_MSG_HMI,         /**< HMI按键 */
} PMC_EVENT_ID;

typedef enum{
    MSBT_LOCK = 0,       // 辅助功能锁住
    DRY_RUN,             // 空运行
    SINGLE_BLOCK,        // 单段
    BLOCK_SKIP = 3,      // 程序段跳过（1~9） 
    ALL_AXIS_LK,         // 所有轴互锁   
    MAC_LOCK = 12,       // 所有轴机床锁住（MLK,MLK1~4）
    MAC_LOCK1,           // 各轴轴机床锁住   
    MAC_LOCK2,
    MAC_LOCK3,
    MAC_LOCK4,
}ENABLE_KEY;

// 各个PMC事件对应的参数结构体定义（子结构体）, 六种事件类型对应六个参数结构体
/* 1、*/
typedef struct {
    int mode;
} EV_MSG_MODE_PAYLOAD;

/* 2、*/
typedef struct {
    int axis_num;
    int direction;
} EV_MSG_FEEDAXIS_KEY_PAYLOAD;

/* 3、*/
typedef struct {
    int direction;
} EV_MSG_SPINDLE_PAYLOAD;

/* 4、*/
typedef struct {
    int override_type;
    int val;
} EV_MSG_OVERRIDE_PAYLOAD;

/* 5、*/
typedef struct {
    int run_state;
} EV_MSG_AUTO_RUN_PAYLOAD;

/* 6、*/
typedef struct {
    int reset_state;
    int reset_type;
} EV_MSG_RESET_PAYLOAD;

typedef struct {
    int enable_key;
    int enable_state;
} EV_MSG_ENABLE_PAYLOAD;

typedef struct {
    int p1;
    int p2;
} EV_MSG_SPD_ORI_PAYLOAD;

typedef struct{
    int press_key;
} EV_MSG_HMI_PAYLOAD;

/**
 * @brief CNC 事件通用结构
 * 用于在内存中统一事件格式，方便解析
 */
typedef struct {
    PMC_EVENT_ID id;    /**< 事件代码 */
    union{
        EV_MSG_MODE_PAYLOAD mode_payload;
        EV_MSG_FEEDAXIS_KEY_PAYLOAD feedaxis_key_payload;
        EV_MSG_SPINDLE_PAYLOAD spindle_payload;
        EV_MSG_OVERRIDE_PAYLOAD override_payload;
        EV_MSG_AUTO_RUN_PAYLOAD run_payload;
        EV_MSG_RESET_PAYLOAD reset_payload;
        EV_MSG_ENABLE_PAYLOAD enable_payload;
        EV_MSG_SPD_ORI_PAYLOAD spd_ori_payload;
        EV_MSG_HMI_PAYLOAD hmi_payload;
    };
} PMC_EVENT;

#pragma pack(push, 1)
/**
 * @brief G 变量本地快照映射
 * 采用数组方式管理。
 * g_regs 明确指代 PMC 中的 G 寄存器数据 (PMC -> CNC)。
 * 索引值即为 G 寄存器的偏移量，例如 g_regs[12] 对应 G0012。
 * REG_G_NUM: 共享内存支持的最大G寄存器索引大小
 */
typedef struct {
    char g_regs[REG_G_NUM];
} G_INPUT_MAP;
#pragma pack(pop)

#endif /* PMC_IO_MAP_H */
