/**
 * @file pmc_output_handler.c
 */
#include "pmc_output_handler.h"
#include "regapi.h"   /* 引入寄存器访问 API */
#include "regdata.h"  /* 引入 RegType 定义 */

/* ==========================================================================
 * 外部函数定义
 * ========================================================================== */

void pmc_output_feedback(int is_running, int has_alarm) {
    /**
     * 映射策略：
     * F0000.0 (第0字节第0位) -> 运行状态反馈
     * F0001.0 (第1字节第0位) -> 报警状态反馈
     */
    unsigned char f0_val = is_running ? 0x01 : 0x00;
    unsigned char f1_val = has_alarm  ? 0x01 : 0x00;

    /* 使用底层 API 写入 F 寄存器区 */
    setRegValue8(REG_F, 0, f0_val); /* REG_F 定义见 regdata.h */
    setRegValue8(REG_F, 1, f1_val);
}
