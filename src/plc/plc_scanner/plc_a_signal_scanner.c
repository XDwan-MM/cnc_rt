#include "plc_a_signal_scanner.h"
#include "alarmapi.h"
#include "global.h"
#include "regapi.h" /**< 寄存器访问 API */
#include "rt_data.h"
#include "signal_utils.h" /**< 边沿检测宏定义 */
#include <hmisysapi.h>
#include <string.h>

// 静态存储 A0-A127 的上次状态
static char last_a_status[A_BYTE_COUNT] = {0}; // 上次快照数据

// 首次扫描标志
// 首次扫描使用电平检测
static bool is_first_scan = true;

/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static void plc_a_input_fetch_snapshot(char *local_snap);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。
 */
void plc_a_signal_scan_cycle(void)
{
    char current_a_status[A_BYTE_COUNT]; // 本次抓取的实时数据

    // 1. 同步最新硬件/共享内存数据到数组
    plc_a_input_fetch_snapshot(current_a_status);

    // 2. 遍历处理所有字节
    for (int i = 0; i < A_BYTE_COUNT; i++)
    {
        if (is_first_scan)
        {
            // 首次扫描：使用电平检测
            for (int bit = 0; bit < 8; bit++)
            {
                if (GET_BIT(current_a_status[i], bit))
                {
                    if (i < 64)
                    {
                        // A0-A63: 报警 (ALARM_ERROR)
                        int alarm_no = 1 + (i * 8) + bit;
                        setAlarm(ALARM_UP, ALARM_ERROR, alarm_no);
                    }
                    else
                    {
                        // A64-A127: 提示 (ALARM_WARNING)
                        int warning_no = 1 + ((i - 64) * 8) + bit;
                        setAlarm(ALARM_UP, ALARM_WARNING, warning_no);
                    }
                }
            }
        }
        else
        {
            // 后续扫描：正常边沿检测
            // 性能优化：字节无变化则跳过
            if (current_a_status[i] == last_a_status[i])
                continue;

            // 3. 遍历字节中的 8 个位，使用上升沿/下降沿宏判断
            for (int bit = 0; bit < 8; bit++)
            {
                if (i < 64)
                {
                    /* ==========================================
                     * A0 - A63: 对应 报警 (ALARM_ERROR)
                     * 报警号计算公式：alarm_no = 1 + (i * 8) + bit
                     * ========================================== */
                    int alarm_no = 1 + (i * 8) + bit;

                    // 检测上升沿 (0 -> 1): 产生报警
                    if (SIGNAL_RISING(current_a_status[i], last_a_status[i], bit))
                    {
                        setAlarm(ALARM_UP, ALARM_ERROR, alarm_no);
                    }
                    // 检测下降沿 (1 -> 0): 消除报警
                    else if (SIGNAL_FALLING(current_a_status[i], last_a_status[i], bit))
                    {
                        delAlarm(ALARM_UP, ALARM_ERROR, alarm_no);
                    }
                }
                else
                {
                    /* ==========================================
                     * A64 - A127: 对应 提示 (ALARM_WARNING)
                     * 提示号计算公式：warning_no = 1 + ((i - 64) * 8) + bit
                     * A64.0 -> 1 + (0 * 8) + 0 = 1
                     * A64.7 -> 1 + (0 * 8) + 7 = 8
                     * A65.0 -> 1 + (1 * 8) + 0 = 9
                     * A127.7 -> 1 + (63 * 8) + 7 = 512
                     * ========================================== */
                    int warning_no = 1 + ((i - 64) * 8) + bit;

                    // 检测上升沿 (0 -> 1): 产生提示
                    if (SIGNAL_RISING(current_a_status[i], last_a_status[i], bit))
                    {
                        setAlarm(ALARM_UP, ALARM_WARNING, warning_no);
                    }
                    // 检测下降沿 (1 -> 0): 消除提示
                    else if (SIGNAL_FALLING(current_a_status[i], last_a_status[i], bit))
                    {
                        delAlarm(ALARM_UP, ALARM_WARNING, warning_no);
                    }
                }
            }
        }
    }

    // 4.在处理完所有字节后，统一更新历史记录
    memcpy(last_a_status, current_a_status, A_BYTE_COUNT);
    // 5.首次扫描结束，清除标志位
    is_first_scan = false;
}

/* ==========================================================================
 * 3. 内部函数实现 (Private Implementations)
 * ========================================================================== */

/**
 * @brief 从寄存器系统同步 A 变量数据到本地静态数组
 * @param local_a_snap 指向存储 A 变量快照的数组 (例如 static unsigned char
 * last_a_status[A_BYTE_COUNT])
 */
static void plc_a_input_fetch_snapshot(char *local_a_snap)
{
    if (local_a_snap == NULL)
        return;

    /* 使用循环逐个抓取 A 寄存器字节 (A0 - A127) */
    for (int i = 0; i < A_BYTE_COUNT; i++)
    {
        // 从共享内存寄存器读取 A(i) 的值
        getRegValue8(REG_A, i, &local_a_snap[i]);
    }
}
