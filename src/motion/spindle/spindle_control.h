/**
 * @file spindle_control.h
 * @brief 主轴控制模块的公开接口。
 *
 * 外部模块只能通过本文件提交主轴命令或读取业务状态，不得直接访问主轴
 * 状态机、规划器或 HAL 模式切换过程。所有命令最终都在 1 ms 运动周期中
 * 统一仲裁，避免任务线程、PLC 扫描线程和运动线程同时修改控制状态。
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

/** 撤销主轴力矩/使能意愿，由复位等上层流程接管下使能时调用。 */
void spindle_release_torque_request(void);

/** 标记一条 M19 正在等待完成反馈。实际定向由 PLC G70.6 上升沿触发。 */
void spindle_begin_m19(void);

/** 提交或释放 PLC 主轴定向请求。active=true 请求，false 释放。 */
void spindle_request_orientation(bool active);

/** 更新 PLC 模拟量换算后的主轴转速，单位 rpm。 */
void spindle_set_plc_speed(double speed);

/** 选择 CNC 或 PLC 速度输入。 */
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
void spindle_request_m29(void);

/** 不定向 M29：受控停车后在当前位置切换 CSP。 */
bool spindle_request_m29_no_orient(void);

/** M29 是否仍处于有效握手流程。 */
bool spindle_m29_is_active(void);

/** M29 是否已经进入不可继续的错误状态。 */
bool spindle_m29_has_error(void);

/** M29 的 CSP、使能、零速及位置保持条件是否全部满足。 */
bool spindle_m29_is_ready(void);

/** PLC FIN 与 NC 就绪均完成后结束本次 M29 握手。 */
void spindle_complete_m29(void);

/** 报告不定向 M29 在入口联锁检查阶段被拒绝。 */
void spindle_report_m29_prepare_rejected(void);

/** 报告 M29 的外部 PLC FIN 握手超时。 */
void spindle_report_m29_handshake_timeout(void);

/** HMI 复位是否仍在等待主轴受控停稳。 */
bool spindle_reset_stop_pending(void);

/** CSP 攻丝复位停车是否需要主轴位置反馈保持有效。 */
bool spindle_reset_stop_uses_csp(void);

/** F45.3 的唯一查询入口。 */
bool spindle_velocity_is_ready(void);

/** 普通 M19 定向完成查询；M29 期间始终返回 false。 */
bool spindle_orientation_is_done(void);

#ifdef __cplusplus
}
#endif

#endif /* SPINDLE_CONTROL_H */
