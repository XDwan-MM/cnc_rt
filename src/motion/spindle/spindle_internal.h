/**
 * @file spindle_internal.h
 * @brief 主轴控制模块私有状态与内部协作接口。
 *
 * 本文件只能被 src/motion/spindle 目录中的实现文件包含。外部模块不得依赖
 * 任何阶段枚举或状态字段，否则会重新形成跨线程直接修改状态的问题。
 */

#ifndef SPINDLE_INTERNAL_H
#define SPINDLE_INTERNAL_H

#include "spindle_control.h"
#include "spindle_velocity_planner.h"
#include "global.h"
#include <stdbool.h>
#include <stdint.h>

/** 每周期最多处理一个已仲裁的主轴事件。 */
typedef enum
{
    SPINDLE_EVENT_NONE = 0,
    SPINDLE_EVENT_STOP,
    SPINDLE_EVENT_CW,
    SPINDLE_EVENT_CCW,
    SPINDLE_EVENT_ORIENT,
    SPINDLE_EVENT_RELEASE_ORIENT
} spindle_event_t;

/** 对 PLC 发布的速度完成状态。 */
typedef enum
{
    SPINDLE_VELOCITY_ZERO = 0,
    SPINDLE_VELOCITY_RAMPING,
    SPINDLE_VELOCITY_ARRIVED
} spindle_velocity_state_t;

/** M19 定向状态机。 */
typedef enum
{
    SPINDLE_ORIENT_IDLE = 0,
    SPINDLE_ORIENT_WAIT_ZERO,
    SPINDLE_ORIENT_RUNNING,
    SPINDLE_ORIENT_DONE,
    SPINDLE_ORIENT_RELEASING
} spindle_orient_phase_t;

/**
 * M29 状态机。
 *
 * 前半部分属于普通“先定向再切 CSP”的 M29，NO_ORIENT_* 属于当前位置
 * 停稳后切 CSP 的流程。后续拆分实现时二者分别由独立函数推进，保留同一
 * 枚举只为降低本次行为保持重构的迁移风险。
 */
typedef enum
{
    SPINDLE_M29_IDLE = 0,
    SPINDLE_M29_ORIENTING,
    SPINDLE_M29_STOPPING_TO_CSP,
    SPINDLE_M29_RELEASING,
    SPINDLE_M29_SWITCHING_CSP,
    SPINDLE_M29_READY,
    SPINDLE_M29_ERROR,
    SPINDLE_M29_NO_ORIENT_WAIT_CSV,
    SPINDLE_M29_NO_ORIENT_STOPPING,
    SPINDLE_M29_NO_ORIENT_SWITCHING,
    SPINDLE_M29_NO_ORIENT_ALIGNING
} spindle_m29_phase_t;

/** 主轴模式切换请求的本地事务状态。 */
typedef enum
{
    SPINDLE_MODE_SWITCH_IDLE = 0,
    SPINDLE_MODE_SWITCH_PENDING,
    SPINDLE_MODE_SWITCH_DONE,
    SPINDLE_MODE_SWITCH_ERROR
} spindle_mode_switch_state_t;

/**
 * HAL 未提供直接查询接口的本地输出镜像。
 *
 * target_velocity 和 target_position 只用于显示、完成判定及诊断；驱动器真实
 * 状态始终以本周期 HAL 反馈为准。mode_switch_* 保存异步模式请求的进度。
 */
typedef struct
{
    double target_velocity;
    double target_position;
    int requested_mode;
    spindle_mode_switch_state_t mode_switch_state;
    int32_t mode_switch_result;
} spindle_hal_state_t;

/**
 * 主轴控制器的全部可变状态。
 *
 * 字段按职责分组。该结构只在主轴模块内部可见，其他线程必须通过公开命令
 * 接口提交请求，由运动线程在 spindle_control_update() 中统一消费。
 */
typedef struct
{
    /* 普通旋转命令与单槽内部事件。 */
    spindle_state_t run_state;
    spindle_state_t command_state;
    spindle_velocity_state_t velocity_state;
    spindle_event_t pending_event;

    /* CNC/PLC 速度输入，单位均为 rpm。 */
    double command_speed;
    double remembered_program_speed;
    double plc_speed;
    spindle_speed_source_t speed_source;

    /* CSV 速度斜坡规划器。 */
    spindle_velocity_planner_t velocity_planner;

    /* M 代码和模式切换握手。 */
    bool mcode_complete;
    bool m19_active;
    bool m29_active;
    spindle_m29_phase_t m29_phase;
    uint32_t m29_stop_stable_cycles;
    bool m29_feedback_stable; /**< CSP 零速和位置误差连续稳定后置位。 */
    bool csv_switch_pending;

    /* M19 定向及定向期间的命令交接。 */
    bool orient_hold_stop;
    bool orient_request_pending;
    bool orient_release_pending;
    spindle_orient_phase_t orient_phase;
    uint32_t orient_elapsed_cycles;

    /* 复位/急停后的启动锁和受控停车状态。 */
    bool reset_requires_start;
    bool reset_stop_pending;
    bool reset_stop_csp;
    int reset_stop_feed_axis;
    uint32_t reset_stop_cycles;
    uint32_t reset_stop_stable_cycles;

    /* 定向期间收到的启动请求，在释放完成后再执行。 */
    bool start_request_pending;
    bool deferred_start_pending;
    spindle_state_t deferred_start_state;
    double deferred_start_speed;
} spindle_context_t;

extern spindle_context_t g_spindle;
extern spindle_hal_state_t g_spindle_hal;

/* HAL 反馈访问与输出意愿。 */
void spindle_hal_init(void);
int spindle_hal_is_enabled(void);
int spindle_hal_mode(void);
double spindle_hal_actual_velocity(void);
double spindle_hal_actual_position(void);
uint16_t spindle_hal_status_word(void);
int spindle_hal_orientation_state(void);
bool spindle_hal_orientation_is_inactive(void);
void spindle_hal_request_torque(void);
void spindle_hal_release_torque(void);

/* 异步 CSV/CSP 模式切换事务。 */
bool spindle_hal_mode_switch_is_pending(void);
bool spindle_hal_mode_switch_is_done(void);
int32_t spindle_hal_mode_switch_result(void);
void spindle_hal_mode_switch_reset(void);
int spindle_hal_request_mode(int mode);

/* M19 定向子模块及其 HAL 命令。 */
bool spindle_orientation_submit(void);
bool spindle_orientation_hal_is_done(void);
bool spindle_orientation_release(void);
void spindle_orientation_abort(void);
void spindle_internal_update_orientation(void);

/* 定向模块回调控制核心执行排队启动和完成反馈。 */
void spindle_internal_handle_rotation(spindle_context_t *context,
                                      spindle_state_t state);

/* 安全仲裁和 HMI 复位受控停车。 */
bool spindle_internal_stop_is_required(const spindle_context_t *context);
bool spindle_internal_reset_or_estop_active(void);
bool spindle_internal_reset_stop_abort_required(void);
void spindle_internal_update_reset_stop(void);

/* CSV 速度规划与完成判定。 */
void spindle_internal_update_velocity(void);
bool spindle_internal_validate_start_request(const spindle_context_t *context);
double spindle_internal_get_selected_speed(const spindle_context_t *context);
void spindle_internal_sync_planner_to_actual(void);
void spindle_internal_finish_stop_if_ready(void);
void spindle_internal_publish_status(void);
void spindle_internal_report_mcode_complete(void);

/* 控制核心提供给 M19/M29/安全停车模块的公共内部操作。 */

/* M29 子状态机，由 1 ms 控制入口按固定顺序推进。 */
bool spindle_internal_m29_no_orient_preparing(void);
void spindle_internal_update_m29_no_orient(void);
void spindle_internal_update_m29(void);
void spindle_internal_update_m29_feedback(void);
void spindle_internal_publish_rigid_state(void);

/* 只供主轴实现文件协作的内部函数。 */
void spindle_internal_clear_velocity_arrival(void);

#endif /* SPINDLE_INTERNAL_H */
