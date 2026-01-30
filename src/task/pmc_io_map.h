#ifndef PMC_IO_MAP_H
#define PMC_IO_MAP_H

/**
 * @brief CNC 运行模式枚举
 * 映射逻辑：基于 MD1(bit0), MD2(bit1), MD4(bit2), DNCI(bit5), ZRN(bit7) 的组合
 */
typedef enum {
    CNC_MODE_EDIT = 0,    /**< 编辑 (MD1=0, MD2=0, MD4=0) */
    CNC_MODE_MEM  = 1,    /**< 存储器 (MD1=1, MD2=0, MD4=0) */
    CNC_MODE_MDI  = 2,    /**< MDI (MD1=0, MD2=1, MD4=0) */
    CNC_MODE_JOG  = 3,    /**< JOG (MD1=1, MD2=1, MD4=0) */
    CNC_MODE_MPG  = 4,    /**< 手轮 (MD1=0, MD2=0, MD4=1) */
    CNC_MODE_ZRN  = 5,    /**< 回零 (ZRN=1) */
    CNC_MODE_DNC  = 6,    /**< DNC运行 (DNCI=1) */
    CNC_MODE_INVALID = 7  /**< 无效模式 */
} CNC_MODE_INDEX;

/**
 * @brief PMC事件 ID
 * 对应 EVENT_DATA 中的 code 字段，用于区分不同的动作类型
 */
typedef enum {
    EV_MSG_NONE = 0,
    EV_MSG_MODE,        /**< 模式切换 (参数1: 模式索引) */
    EV_MSG_JOG,         /**< 手动点动 (参数1: 轴号, 参数2: 方向) */
    EV_MSG_SPINDLE,     /**< 主轴控制 (参数1: 方向) */
    EV_MSG_OVERRIDE,    /**< 倍率调节 (参数1: 类型, 参数2: 数值) */
    EV_MSG_AUTO_RUN,    /**< 运行控制 (参数1: 启动/中止) */
    EV_MSG_RESET        /**< 系统复位 */
} PMC_EVENT_ID;

// 各个PMC事件对应的参数结构体定义（子结构体）, 六种事件类型对应六个参数结构体
/* 1、*/
typedef struct {
    int mode;
} EV_MSG_MODE_PAYLOAD;

/* 2、*/
typedef struct {
    int axis_num;
    int direction;
} EV_MSG_JOG_PAYLOAD;

/* 3、*/
typedef struct {
    int direction;
} EV_MSG_SPINDLE_PAYLOAD;

/* 4、*/
typedef struct {
    int override_type;;
    int val;
} EV_MSG_OVERRIDE_PAYLOAD;

/* 5、*/
typedef struct {
    int run_state;;
} EV_MSG_AUTO_RUN_PAYLOAD;

/* 6、*/
typedef struct {
    int reset_state;
} EV_MSG_RESET_PAYLOAD;

/**
 * @brief CNC 事件通用结构
 * 用于在内存中统一事件格式，方便解析
 */
typedef struct {
    PMC_EVENT_ID id;    /**< 事件代码 */
    union{
        EV_MSG_MODE_PAYLOAD mode_payload;
        EV_MSG_JOG_PAYLOAD jog_payload;
        EV_MSG_SPINDLE_PAYLOAD spindle_payload;
        EV_MSG_OVERRIDE_PAYLOAD override_payload;
        EV_MSG_AUTO_RUN_PAYLOAD run_payload;
        EV_MSG_RESET_PAYLOAD reset_payload;
    };
} PMC_EVENT;

#pragma pack(push, 1)
/**
 * @brief G 变量本地快照映射
 * 严格对应 regdata.h 中的 regG 寄存器偏移
 */
typedef struct {
    unsigned char G0007; /**< 循环启动信号 */
    unsigned char G0008; /**< 中止与复位信号 */
    unsigned char G0012; /**< 进给倍率 */
    unsigned char G0014; /**< 快速倍率 */
    unsigned char G0029; /**< 主轴停止 */
    unsigned char G0030; /**< 主轴倍率 */
    unsigned char G0043; /**< 模式选择 */ /**< 模式控制字节: bit0:MD1, bit1:MD2, bit2:MD4, bit5:DNCI, bit7:ZRN */
    unsigned char G0070; /**< 主轴方向控制 */
    unsigned char G0100; /**< 正向轴选 (JOG+) */
    unsigned char G0102; /**< 负向轴选 (JOG-) */
} G_INPUT_MAP;
#pragma pack(pop)

#endif
