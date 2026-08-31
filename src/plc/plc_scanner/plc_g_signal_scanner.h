/**
 * @file        plc_g_signal_scanner.h
 * @brief       PLC G信号扫描模块
 * @date        2026/08/31
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef PLC_G_SIGNAL_SCANNER_H
#define PLC_G_SIGNAL_SCANNER_H

#include "event_queue.h"
#include "plc_event_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** 外部减速功能每个方向支持的逻辑轴数量。 */
#define PLC_EXTERNAL_DECEL_AXIS_COUNT (8)

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列。
 * 外部减速 G 信号为低有效，状态通过 PLC_EV_MSG_EXTERNAL_DECEL 事件推送。
 */
extern void plc_g_signal_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* PLC_G_SIGNAL_SCANNER_H */
