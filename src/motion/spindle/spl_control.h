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
    SPINDLE_STOP = 0,
    SPINDLE_CW,
    SPINDLE_CCW,
} spindle_state_t;

/** 主轴控制事件。 */
typedef enum
{
    SPINDLE_EVENT_NONE = 0,
    SPINDLE_EVENT_STOP,          /**< 主轴停止。 */
    SPINDLE_EVENT_CW,            /**< 主轴正转。 */
    SPINDLE_EVENT_CCW,           /**< 主轴反转。 */
    SPINDLE_EVENT_ORIENT,        /**< 主轴定向。 */
    SPINDLE_EVENT_RELEASE_ORIENT /**< 解除主轴定向。 */
} spindle_event_t;

/** 主轴速度规划状态。 */
typedef enum
{
    SPINDLE_VEL_ZERO = 0, /**< 当前速度为零。 */
    SPINDLE_VEL_RAMP,     /**< 当前速度正在变化。 */
    SPINDLE_VEL_ARRIVED   /**< 当前速度已到达目标。 */
} spindle_velocity_state_t;

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
    spindle_state_t spl_state;
    spindle_state_t command_state; /**< 最近一条 CNC M3/M4 的方向请求。 */
    spindle_velocity_state_t spl_vel_state;
    spindle_event_t spl_event;

    /* 速度输入，单位：转/分钟。 */
    double command_speed;    /**< 当前 CNC M3/M4 指令中的 S 值。 */
    double mcp_target_speed; /**< 最近一次 M3/M4 的 S 值，供面板启动使用。 */
    double plc_speed;        /**< PLC 侧设定的主轴速度指令。 */
    spindle_speed_source_t speed_source; /**< G33.7 选择的速度指令来源。 */

    /* 速度规划状态。 */
    spl_vel_planner_t vel_planner;

    /* M 代码握手状态。 */
    bool mcode_feedback; /**< 是否已经完成当前 M 代码反馈。 */
    bool reset_requires_start; /**< 复位后必须收到新的 M3/M4 才允许启动。 */
    bool start_request_pending; /**< 自动模式已收到新的 M3/M4 请求。 */
} spindle_context_t;

extern spindle_context_t spl_context;

/** 执行一个主轴控制周期。 */
void spindle_control_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SPL_CONTROL_H */
