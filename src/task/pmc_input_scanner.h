#ifndef PMC_INPUT_SCANNER_H
#define PMC_INPUT_SCANNER_H

#include "pmc_io_map.h"
#include "event_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化输入扫描器
 * @param queue 绑定任务控制器分配的事件队列
 */
void pmc_input_init(EVENT_QUEUE* queue);

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void pmc_input_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif
