#include "plc_x_signal_scanner.h"
#include "global.h"
#include "regapi.h"       /**< 寄存器访问 API */
#include "signal_utils.h" /**< 边沿检测宏定义 */
#include <hmisysapi.h>
#include <string.h>

/**
 * @brief 需要扫描的 X 寄存器列表。
 *
 * 新增 X 信号时，只需将对应的寄存器号加入此列表，并在扫描逻辑中处理
 * 相应的位即可。未注册的寄存器不会从共享内存读取。
 */
static const int SCAN_X_REGS[] = {
    PLC_X_ESTOP_REG,
};

/** 计算注册表中元素个数。 */
#define SCAN_X_REGS_COUNT (sizeof(SCAN_X_REGS) / sizeof(SCAN_X_REGS[0]))

static X_INPUT_MAP last_x; /**< 上一周期快照 */

/* 首次扫描标志。 */
static bool is_first_scan = true;

/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static void plc_x_input_fetch_snapshot(X_INPUT_MAP *local_snap);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断。
 * X8.4 为低电平有效；X、G 信号分别更新系统急停输入。
 * X8.4 释放时不修改系统急停输入，也不触发复位事件。
 */
void plc_x_signal_scan_cycle(void)
{
    if (rt_data == NULL)
        return;

    X_INPUT_MAP curr_x;
    memset(&curr_x, 0, sizeof(X_INPUT_MAP)); /* 初始化清零，防止脏数据 */
    /* step1. 抓取寄存器快照 */
    plc_x_input_fetch_snapshot(&curr_x);

    /* X8.4 紧急停止检测：X8.4 = 0 急停拍下，X8.4 = 1 急停释放。 */
    if (!GET_BIT(curr_x.x_regs[PLC_X_ESTOP_REG], PLC_X_ESTOP_BIT))
    {
        rt_set_sys_state(SYS_ESTOP_INPUT, true);
        if (is_first_scan || GET_BIT(last_x.x_regs[PLC_X_ESTOP_REG], PLC_X_ESTOP_BIT))
        {
            LOG(LOG_LEVEL_INFO, "扫描到急停X8.4信号为0, 急停拍下");
        }
    }
    /* X8.4 释放不修改急停输入、不产生复位事件，G8.4 的逻辑由 G 扫描器独立处理。 */
    if (!is_first_scan && SIGNAL_RISING(curr_x.x_regs[PLC_X_ESTOP_REG], last_x.x_regs[PLC_X_ESTOP_REG], PLC_X_ESTOP_BIT))
    {
        LOG(LOG_LEVEL_INFO, "扫描到急停X8.4信号由0变1, 不修改急停输入且不触发复位");
    }

    /* 更新快照供下一周期使用。 */
    last_x = curr_x;

    /* 首次扫描结束，清除标志位。 */
    is_first_scan = false;
}

/* ==========================================================================
 * 3. 内部函数定义 (Private Implementations)
 * ========================================================================== */

/**
 * @brief 从寄存器系统同步数据到本地结构体。
 */
static void plc_x_input_fetch_snapshot(X_INPUT_MAP *local_snap)
{
    if (local_snap == NULL)
        return;

    /* 遍历注册表中的每一个索引。 */
    for (int i = 0; i < SCAN_X_REGS_COUNT; i++)
    {
        int reg_idx = SCAN_X_REGS[i];
        getRegValue8(REG_X, reg_idx, &local_snap->x_regs[reg_idx]);
    }
}
