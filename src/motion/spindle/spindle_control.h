/**
 * @file spindle_control.h
 * @brief 主轴控制模块的公开接口。
 *
 * 外部模块只能通过本文件提交主轴命令或读取业务状态，不得直接访问主轴
 * 状态机、规划器或 HAL 模式切换过程。命令入口由运动线程调用，本接口不提供
 * 跨线程排队。PLC 速度/速度源两个入口目前由 PLC 线程直接调用，尚需同步交接。
 */

#ifndef SPINDLE_CONTROL_H
#define SPINDLE_CONTROL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** 主轴旋转方向。STOP 仅用于显式停止请求。 */
typedef enum
{
    SPINDLE_STOP = 0,
    SPINDLE_CW,
    SPINDLE_CCW
} spindle_state_t;

/** 主轴速度指令来源。G33.7=0 使用 CNC，G33.7=1 使用 PLC。 */
typedef enum
{
    SPINDLE_SPEED_SOURCE_CNC = 0,
    SPINDLE_SPEED_SOURCE_PLC
} spindle_speed_source_t;

/** 初始化主轴控制器及其全部内部闩锁。 */
void spindle_init(void);

/** 执行一个 1 ms 主轴控制周期。 */
void spindle_control_update(void);

/** 提交自动/MDI M3、M4 指令，并更新手动启动使用的记忆转速。 */
bool spindle_request_program_run(spindle_state_t direction, double speed);

/** 提交面板或 PLC 方向信号，使用最近一次有效的程序转速。 */
bool spindle_request_manual_run(spindle_state_t direction);

/** 提交普通 M5/面板停止请求。 */
void spindle_request_stop(void);

/** 程序结束时退出定向，并受控停车后下使能。 */
void spindle_request_program_end_stop(void);

/** 标记一条 M19 正在等待完成反馈。实际定向由 PLC G70.6 上升沿触发。 */
void spindle_begin_m19(void);

/** 提交定向或退出请求；适配设备支持 CSV 零速/CSP 当前位置保持，拒绝时保留定向。 */
void spindle_request_orientation(bool active);

/** 更新 PLC 转速，单位 rpm；当前调用来自 PLC 线程，非线程安全队列。 */
void spindle_set_plc_speed(double speed);

/** 选择 CNC 或 PLC 速度输入；须与 PLC 转速建立一致的跨线程交接。 */
void spindle_set_speed_source(spindle_speed_source_t source);

/** 获取按 G33.7 选择后的当前主轴速度值，单位 rpm。 */
double spindle_get_command_speed(void);

/**
 * @brief 获取最近一次 M3/M4 的 NC 指令 S 速度，单位 rpm。
 *
 * 该接口不受 G33.7 和 M5 影响，始终返回 NC 侧最近一次记录的 S 值。
 * PLC F22~F25 输出的是 NC 指令数据，必须使用此接口，不能使用当前
 * 速度源选择后的目标速度，也不能使用主轴实际反馈速度。
 */
double spindle_get_program_command_speed(void);

/** 普通 M29：定向后切换 CSP。 */
bool spindle_request_m29(void);

/** 不定向 M29：受控停车后在当前位置切换 CSP。 */
bool spindle_request_m29_no_orient(void);

/** M29 是否仍处于有效准备及等待 PLC FIN 的流程。 */
bool spindle_m29_is_active(void);

/** M29 是否已经进入不可继续的错误状态。 */
bool spindle_m29_has_error(void);

/** M29 的 CSP、使能、零速及位置保持条件是否全部满足。 */
bool spindle_m29_is_ready(void);

/** 通用 PLC FIN 结束 M29 时收尾，不下使能、不清运动模块的 CSP 保持目标。 */
void spindle_complete_m29(void);

/** 报告 M29 在入口联锁检查阶段被拒绝。 */
void spindle_report_m29_prepare_rejected(void);

/** CSV 停车执行器；公共复位流程负责调度和最终裁决。 */
bool spindle_csv_stop_pending(void);
void spindle_csv_stop_begin(void);
void spindle_csv_stop_cancel(void);

/** F45.3 的唯一查询入口。 */
bool spindle_velocity_is_ready(void);

/** 普通 M19 定向完成查询；M29 期间始终返回 false。 */
bool spindle_orientation_is_done(void);

#ifdef __cplusplus
}
#endif

#endif /* SPINDLE_CONTROL_H */
