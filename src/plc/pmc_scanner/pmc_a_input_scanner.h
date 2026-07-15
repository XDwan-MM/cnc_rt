#ifndef PMC_A_INPUT_SCANNER_H
#define PMC_A_INPUT_SCANNER_H

#include "pmc_io_map.h"

#define A_START_ADDR 0
#define A_BYTE_COUNT 128  // 1、A0-A63共64个PLC报警A变量 2、 A64-A127共64个PLC提示A变量

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断
 */
void pmc_a_input_scan_cycle(void);

#ifdef __cplusplus
}
#endif

#endif
