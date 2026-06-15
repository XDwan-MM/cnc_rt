/**
 * @file pmc_output_handler.h
 * @brief PMC 输出反馈模块接口。
 * 本模块负责将 CNC 内核的运行状态（如是否运行、是否报警）写回 F 寄存器。
 */

#ifndef PMC_OUTPUT_HANDLER_H
#define PMC_OUTPUT_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 反馈内核状态至 PMC 系统。
 * @param is_running 当前系统是否处于自动运行（Cycle Start）状态。
 * @param has_alarm  当前系统是否存在待处理的报警。
 * * @note 此函数应在任务控制器的输出阶段调用，利用 setRegValue8 接口写入 F 寄存器。
 */
void pmc_output_feedback();

#ifdef __cplusplus
}
#endif

#endif // PMC_OUTPUT_HANDLER_H
