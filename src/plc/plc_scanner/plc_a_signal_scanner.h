/**
 * @file        plc_a_signal_scanner.h
 * @brief       PLC 报警A信号扫描模块
 * @date        2026/08/31
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef PLC_A_SIGNAL_SCANNER_H
#define PLC_A_SIGNAL_SCANNER_H

#include "plc_event_types.h"

#define A_START_ADDR 0
#define A_BYTE_COUNT 128 // 1、A0-A63共64个PLC报警A变量 2、 A64-A127共64个PLC提示A变量

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断
 */
extern void plc_a_signal_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* PLC_A_SIGNAL_SCANNER_H */
