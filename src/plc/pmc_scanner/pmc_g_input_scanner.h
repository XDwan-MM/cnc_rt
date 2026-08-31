#ifndef PMC_G_INPUT_SCANNER_H
#define PMC_G_INPUT_SCANNER_H

#include "pmc_io_map.h"
#include "event_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 外部减速功能数量及每个方向支持的逻辑轴数量。 */
#define PMC_EXTERNAL_DECEL_FUNCTION_COUNT (5)
#define PMC_EXTERNAL_DECEL_AXIS_COUNT     (8)

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列。
 * 外部减速 G 信号为低有效，状态通过 EV_MSG_EXTERNAL_DECEL 事件推送。
 */
void pmc_g_input_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif
