#ifndef PMC_G_INPUT_SCANNER_H
#define PMC_G_INPUT_SCANNER_H

#include "pmc_io_map.h"
#include "event_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void pmc_g_input_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif
