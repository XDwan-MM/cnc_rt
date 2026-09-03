/**
 * @file        plc_x_signal_scanner.h
 * @brief       PLC X 信号扫描模块
 * @date        2026/09/03
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef PLC_X_SIGNAL_SCANNER_H
#define PLC_X_SIGNAL_SCANNER_H

#include "event_queue.h"
#include "plc_event_types.h"

/** 当前扫描的 X 急停寄存器及位。 */
#define PLC_X_ESTOP_REG (8)
#define PLC_X_ESTOP_BIT (4)

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 周期性扫描 X8.4 急停输入。
 *
 * X8.4 为低电平有效，信号释放时不修改系统急停输入且不触发复位事件；
 * G8.4 保持现有急停复位逻辑。
 * 建议在运动控制器线程中周期调用。
 */
extern void plc_x_signal_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* PLC_X_SIGNAL_SCANNER_H */
