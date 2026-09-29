#ifndef SPL_CONTROL_H
#define SPL_CONTROL_H

#include "driver_hal.h"
#include "global.h"
#include "spl_vel_planner.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** 主轴逻辑状态。定向过程由 spl_orient 模块维护。 */
typedef enum
{
    SPINDLE_STOP = 0, /**< 逻辑停止；速度规划器目标为零。 */
    SPINDLE_CW,       /**< 主轴正转，目标速度为正。 */
    SPINDLE_CCW,      /**< 主轴反转，目标速度为负。 */
} spindle_state_t;

/** 主轴控制事件。 */
typedef enum
{
    SPINDLE_EVENT_NONE = 0, /**< 无待处理事件。 */
    SPINDLE_EVENT_STOP, /**< 普通停止、复位或安全停机请求。 */
    SPINDLE_EVENT_CW, /**< 正转请求，通常来自 M3/G70.5。 */
    SPINDLE_EVENT_CCW, /**< 反转请求，通常来自 M4/G70.4。 */
    SPINDLE_EVENT_ORIENT, /**< 定向请求，通常对应 G70.6 上升沿。 */
    SPINDLE_EVENT_RELEASE_ORIENT /**< 释放定向，通常对应 G70.6 下降沿。 */
} spindle_event_t;

/** 主轴速度规划状态。 */
typedef enum
{
    SPINDLE_VEL_ZERO = 0, /**< 规划器/反馈速度为零。 */
    SPINDLE_VEL_RAMP, /**< 正在加速或减速，尚未满足到速条件。 */
    SPINDLE_VEL_ARRIVED /**< 已满足到速或停止完成握手条件。 */
} spindle_velocity_state_t;

/** 主轴定向流程状态。 */
typedef enum
{
    SPINDLE_ORIENT_IDLE = 0, /**< 未请求定向，普通 M3/M4 可接管主轴。 */
    SPINDLE_ORIENT_WAIT_ZERO, /**< 兼容的等待零速阶段，当前入口不使用。 */
    SPINDLE_ORIENT_RUNNING, /**< 已请求驱动器定向，等待状态字 bit10。 */
    SPINDLE_ORIENT_DONE, /**< 驱动器报告定向完成，等待释放事件。 */
    SPINDLE_ORIENT_RELEASING /**< 已发送释放，等待驱动器实际下使能。 */
} spindle_orient_phase_t;

/** 主轴速度指令来源。G33.7=0 使用 CNC，G33.7=1 使用 PLC。 */
typedef enum
{
    SPINDLE_SPEED_SOURCE_CNC = 0,
    SPINDLE_SPEED_SOURCE_PLC
} spindle_speed_source_t;

/**
 * 主轴控制数据。
 *
 * 速度单位统一为转/分钟。目标速度由 CNC 或 PLC 提供，
 * 规划器的 current_vel 由速度规划器计算后输出给驱动器。
 */
typedef struct
{
    /* 逻辑状态与待处理事件。 */
    spindle_state_t spl_state; /**< 当前主轴逻辑状态，决定是否允许速度规划。 */
    spindle_state_t command_state; /**< 最近一条 CNC M3/M4 的方向请求。 */
    spindle_velocity_state_t spl_vel_state; /**< 对 PLC 发布的速度握手状态。 */
    spindle_event_t spl_event; /**< 当前待处理事件；每周期处理后清为 NONE。 */

    /* 速度输入，单位：转/分钟。 */
    double command_speed; /**< 当前 CNC M3/M4 指令中的 S 值。 */
    double mcp_target_speed; /**< 最近一次有效 M3/M4 的 S 值，供面板启动使用。 */
    double plc_speed; /**< PLC 侧设定的主轴速度指令。 */
    spindle_speed_source_t speed_source; /**< G33.7 选择 CNC 还是 PLC 速度。 */

    /* 速度规划状态。 */
    spl_vel_planner_t vel_planner; /**< 加减速规划器；current_vel 输出到 60FF。 */

    /* M 代码握手状态。 */
    bool mcode_feedback; /**< 当前 M 代码/速度握手是否已经报告完成。 */
    bool m19_active; /**< 当前是否有待完成的 M19 定向握手。 */
    bool orient_hold_stop; /**< M19/定向完成后锁定停止，等待新的 M3/M4。 */
    bool orient_request_pending; /**< 收到定向请求但尚未提交驱动器定向命令。 */
    bool orient_release_pending; /**< 定向完成前收到释放请求，待完成后释放。 */
    spindle_orient_phase_t orient_phase; /**< CNC/PLC 层定向阶段。 */
    uint32_t orient_elapsed_cycles; /**< 定向运行已经过的控制周期数。 */
    bool reset_requires_start; /**< 复位/急停后禁止旧目标自动恢复。 */
    bool start_request_pending; /**< 已收到新的 M3/M4，允许解除复位启动锁。 */
    bool deferred_start_pending; /**< 定向期间收到的 M3/M4，释放后自动执行。 */
    spindle_state_t deferred_start_state; /**< 延迟启动的方向。 */
    double deferred_start_speed; /**< 延迟启动的速度。 */
} spindle_context_t;

extern spindle_context_t spl_context;

/** 执行一个主轴控制周期。 */
void spindle_control_update(void);

void spl_context_init(spindle_context_t* spl_ctx);

#ifdef __cplusplus
}
#endif

#endif /* SPL_CONTROL_H */
