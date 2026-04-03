#include "pmc_a_input_scanner.h"
#include <hmisysapi.h>
#include "regapi.h"       /**< 寄存器访问 API */
#include "signal_utils.h" /**< 边沿检测宏定义 */
#include "alarmapi.h"
#include <string.h>

// 静态存储 A0-A63 的上次状态
static char last_a_status[A_BYTE_COUNT] = {0}; // 上次快照数据

/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static void pmc_a_input_fetch_snapshot(char *local_snap);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void pmc_a_input_scan_cycle(void)
{
    char current_a_status[A_BYTE_COUNT]; // 本次抓取的实时数据

    // 1. 同步最新硬件/共享内存数据到数组
    pmc_a_input_fetch_snapshot(current_a_status);

    /* 2. 将PLC报警反馈给HMI*/
    for (int i = 0; i < A_BYTE_COUNT; i++)
    {
        // 性能优化：字节无变化则跳过
        if (current_a_status[i] == last_a_status[i])
            continue;

        // 3. 遍历字节中的 8 个位，使用上升沿/下降沿宏判断
        for (int bit = 0; bit < 8; bit++)
        {
            /* * 报警号计算逻辑：
             * A0.0: 1 + (0*8) + 0 = 1
             * A0.7: 1 + (0*8) + 7 = 8
             * A1.0: 1 + (1*8) + 0 = 9
             * A63.7: 1 + (63*8) + 7 = 512
             */
            int alarm_no = 1 + (i * 8) + bit;

            // 检测上升沿 (0 -> 1): 产生报警
            if (SIGNAL_RISING(current_a_status[i], last_a_status[i], bit))
            {
                setAlarm(ALARM_PC, ALARM_ERROR, alarm_no);
            }
            // 检测下降沿 (1 -> 0): 消除报警
            else if (SIGNAL_FALLING(current_a_status[i], last_a_status[i], bit))
            {
                delAlarm(ALARM_PC, ALARM_ERROR, alarm_no);
            }
        }

        // 4. 【重要修改】在处理完所有字节后，统一更新历史记录
        memcpy(last_a_status, current_a_status, A_BYTE_COUNT);
    }
}

/* ==========================================================================
 * 3. 内部函数实现 (Private Implementations)
 * ========================================================================== */

/** * @brief 从寄存器系统同步 A 变量数据到本地静态数组
 * @param local_a_snap 指向存储 A 变量快照的数组 (例如 static unsigned char last_a_status[A_BYTE_COUNT])
 */
static void pmc_a_input_fetch_snapshot(char *local_a_snap)
{
    if (local_a_snap == NULL)
        return;

    /* 使用循环逐个抓取 A 寄存器字节 (A0 - A63) */
    for (int i = 0; i < A_BYTE_COUNT; i++)
    {
        // 从共享内存寄存器读取 A(i) 的值
        getRegValue8(REG_A, i, &local_a_snap[i]);
    }
}
