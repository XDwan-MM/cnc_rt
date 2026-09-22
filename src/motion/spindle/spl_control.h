#ifndef SPL_CONTROL_H
#define SPL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "spl_vel_planner.h"
#include "driver_hal.h"
#include "global.h"

// 
typedef enum {
    SPINDLE_STOP = 0,
    SPINDLE_CW,
    SPINDLE_CCW,
    SPINDLE_ORIENT,
    SPINDLE_RELEASE_ORIENT
} SpindleState;

/** 主轴控制事件。 */
typedef enum
{
    SPINDLE_EVENT_NONE = 0,
    SPINDLE_EVENT_STOP,           /**< 主轴停止。 */
    SPINDLE_EVENT_CW,             /**< 主轴正转。 */
    SPINDLE_EVENT_CCW,            /**< 主轴反转。 */
    SPINDLE_EVENT_ORIENT,         /**< 主轴定向。 */
    SPINDLE_EVENT_RELEASE_ORIENT  /**< 解除主轴定向。 */
} spindle_event_t;

/** 主轴速度规划状态。 */
typedef enum
{
    SPINDLE_VEL_ZERO = 0,  /**< 当前速度为零。 */
    SPINDLE_VEL_RAMP,      /**< 当前速度正在变化。 */
    SPINDLE_VEL_ARRIVED    /**< 当前速度已到达目标。 */
} spindle_velocity_state_t;

/**
 * 主轴控制数据。
 *
 * 速度单位统一为转/分钟。目标速度由 CNC 或 PLC 提供，
 * 规划器的 current_vel 由速度规划器计算后输出给驱动器。
 */
typedef struct
{
    /* 状态与事件。 */
    SpindleState spl_state;
    spindle_velocity_state_t spl_vel_state;
    spindle_event_t spl_event;

    /* 速度输入。 */
    double command_speed;        /**< CNC 或 M 代码指令速度。 */
    double plc_speed;            /**< PLC 指令速度。 */
    bool speed_mode;             /**< true 使用 PLC 速度，false 使用 CNC 速度。 */

    /* 速度规划状态。 */
    spl_vel_planner_t vel_planner;

    /* M 代码握手状态。 */
    bool mcode_feedback;         /**< 是否已经完成当前 M 代码反馈。 */
} SPINDLE_DATA;

extern SPINDLE_DATA spl_data;
void spindle_set_state(void);
void spindle_cal_speed(void);
bool spindle_is_stopped(void);
bool spindle_is_rotating(void);

#ifdef __cplusplus
}
#endif


#endif /* SPL_CONTROL_H */
