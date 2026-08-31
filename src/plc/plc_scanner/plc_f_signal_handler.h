/**
 * @file plc_f_signal_handler.h
 * @brief PLC 输出反馈模块接口。
 * 本模块负责将 CNC 内核的运行状态（如是否运行、是否报警）写回 F 寄存器。
 */

#ifndef PLC_F_SIGNAL_HANDLER_H
#define PLC_F_SIGNAL_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 反馈内核状态至 PLC 系统。
 * @param is_running 当前系统是否处于自动运行（Cycle Start）状态。
 * @param has_alarm  当前系统是否存在待处理的报警。
 * * @note 此函数应在任务控制器的输出阶段调用，利用 setRegValue8 接口写入 F 寄存器。
 */
extern void plc_f_signal_handler_cycle();

#ifdef __cplusplus
}
#endif

#endif // PLC_F_SIGNAL_HANDLER_H
