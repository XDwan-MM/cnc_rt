/**
 * @file spindle_orientation.c
 * @brief 主轴 M19 定向状态机与 HAL 定向命令封装。
 *
 * HAL 只提供 NONE、RUNNING、DONE 三态。本模块额外维护 RELEASING 阶段，
 * 用于确认厂商定向控制字已经交回普通 DS402 状态机。定向期间收到的最后
 * 一条手动启动命令会被保存，只有释放完成且所有安全条件满足后才执行。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include <alarmapi.h>
#include <hmisysapi.h>

#define SPINDLE_ALARM_ORIENT_TIMEOUT  (2100)
#define SPINDLE_ORIENT_TIMEOUT_CYCLES (10000U)

/* 内部函数声明 */

static void spindle_orientation_apply_deferred_start(void);

/* 外部函数定义 */

bool spindle_orientation_submit(void)
{
    const int result = rt_hal_spindle_request_orient(RT_HAL_AXIS_SP);

    if (result == HAL_OK)
    {
        return true;
    }
    if (result == HAL_ERROR_UNSUPPORTED)
    {
        LOG(LOG_LEVEL_ERR, "主轴定向请求失败：驱动器未配置定向控制字");
    }
    return false;
}

bool spindle_orientation_hal_is_done(void)
{
    int32_t state = HAL_ORIENT_NONE;

    if (rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &state) != HAL_OK)
    {
        return false;
    }
    return state == HAL_ORIENT_DONE;
}

bool spindle_orientation_release(void)
{
    if (rt_hal_spindle_release_orient(RT_HAL_AXIS_SP) == HAL_OK)
    {
        return true;
    }

    LOG(LOG_LEVEL_ERR, "解除主轴定向失败");
    return false;
}

void spindle_orientation_abort(void)
{
    /* 急停或复位时，物理安全动作由 HAL 覆盖；这里释放定向请求，防止
     * 下一条 M19 继承上次未清理的 DONE/RUNNING 状态。 */
    (void)rt_hal_spindle_release_orient(RT_HAL_AXIS_SP);
}

void spindle_begin_m19(void)
{
    spindle_internal_clear_velocity_arrival();
    g_spindle.m19_active = true;
    g_spindle.orient_hold_stop = true;
    g_spindle.command_speed = 0.0;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;
}

void spindle_request_orientation(bool active)
{
    spindle_internal_clear_velocity_arrival();
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;
    g_spindle.pending_event = active ?
        SPINDLE_EVENT_ORIENT : SPINDLE_EVENT_RELEASE_ORIENT;
}

bool spindle_orientation_is_done(void)
{
    if (g_spindle.m29_active)
    {
        return false;
    }
    return spindle_orientation_hal_is_done();
}

void spindle_internal_update_orientation(void)
{
    static bool release_status_log_active;
    static uint16_t last_release_status_word;
    static int last_release_enabled;
    static int last_release_orient_state;

    /*
     * 把 HAL 的定向进展映射到本文件的阶段状态，并在 M19 完成时产生代码握手。
     * HAL 只有 NONE / RUNNING / DONE 三态；RELEASING 是本地加的"等控制字真的交回"阶段。
     */

    /* 是否被接受由 HAL 给出，不直接看原始 bit10 */
    if (g_spindle.orient_request_pending &&
        g_spindle.orient_phase != SPINDLE_ORIENT_RELEASING)
    {
        const int state = spindle_hal_orientation_state();
        bool accepted;

        accepted = (state == HAL_ORIENT_RUNNING ||
                    state == HAL_ORIENT_DONE);
        if (accepted)
        {
            g_spindle.orient_request_pending = false;
            g_spindle.orient_phase = SPINDLE_ORIENT_RUNNING;
            g_spindle.orient_elapsed_cycles = 0U;
        }
        else
        {
            if (spindle_hal_is_enabled())
            {
                (void)spindle_orientation_submit();
            }
            if (++g_spindle.orient_elapsed_cycles >=
                 SPINDLE_ORIENT_TIMEOUT_CYCLES)
            {
                LOG(LOG_LEVEL_ERR,
                    "主轴定向请求一直不被受理：status_word=0x%04X enabled=%d",
                    spindle_hal_status_word(), spindle_hal_is_enabled());
                setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
                g_spindle.orient_request_pending = false;
                g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
                g_spindle.m19_active = false;
                g_spindle.orient_hold_stop = true;
                g_spindle.reset_requires_start = true;
                spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
                g_spindle.start_request_pending = false;
                g_spindle.command_speed = 0.0;
            }
        }
    }

    /* 2) 已请求，等 HAL 把完成位置起来。 */
    if (g_spindle.orient_phase == SPINDLE_ORIENT_RUNNING)
    {
        if (spindle_orientation_hal_is_done())
        {
            g_spindle.orient_phase = SPINDLE_ORIENT_DONE;
            g_spindle.orient_elapsed_cycles = 0U;
        }
        else if (++g_spindle.orient_elapsed_cycles >=
                 SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR, "主轴定向超时，自动请求释放");
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            g_spindle.m19_active = false;
            g_spindle.orient_phase = SPINDLE_ORIENT_RELEASING;
            g_spindle.orient_release_pending = true;
            g_spindle.orient_elapsed_cycles = 0U;
        }
    }

    /* 3) 待处理的释放请求（定向完成前的释放、复位、超时都走这里）。 */
    if (g_spindle.orient_release_pending && spindle_orientation_release())
    {
        g_spindle.orient_release_pending = false;
        if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE)
        {
            g_spindle.orient_phase = SPINDLE_ORIENT_RELEASING;
        }
        g_spindle.orient_elapsed_cycles = 0U;
    }

    /* 4) 释放收尾。HAL 的释放是"控制字交回状态机并重新使能"，不是退回
     *    Switch On Disabled，所以看的是定向状态已经退回 NONE。 */
    if (g_spindle.orient_phase == SPINDLE_ORIENT_RELEASING &&
        spindle_hal_orientation_is_inactive())
    {
        g_spindle.orient_elapsed_cycles = 0U;
        release_status_log_active = false;
        if (g_spindle.orient_request_pending)
        {
            /* 等释放信号已经发送到总线上之后，在下一个周期再重新发出请求。 */
            g_spindle.orient_phase = SPINDLE_ORIENT_WAIT_ZERO;
        }
        else
        {
            g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
            g_spindle.orient_hold_stop = false;
            spindle_internal_sync_planner_to_actual();
            spindle_orientation_apply_deferred_start();
        }
    }

    /* 5) 释放迟迟收不了尾就报警。这里不再退回去使能——HAL 那边已经重新使能了。 */
    if (g_spindle.orient_phase == SPINDLE_ORIENT_RELEASING)
    {
        bool release_status_changed;

        release_status_changed =
            (!release_status_log_active ||
             last_release_status_word != spindle_hal_status_word() ||
             last_release_enabled != spindle_hal_is_enabled() ||
             last_release_orient_state != spindle_hal_orientation_state());

        if (release_status_changed)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴定向释放状态：status_word=0x%04X enabled=%d "
                "orient_state=%d actual_vel=%.0f",
                spindle_hal_status_word(), spindle_hal_is_enabled(),
                spindle_hal_orientation_state(), spindle_hal_actual_velocity());
            release_status_log_active = true;
            last_release_status_word = spindle_hal_status_word();
            last_release_enabled = spindle_hal_is_enabled();
            last_release_orient_state = spindle_hal_orientation_state();
        }

        if (++g_spindle.orient_elapsed_cycles >= SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴定向释放超时：status_word=0x%04X enabled=%d "
                "orient_state=%d actual_vel=%.0f",
                spindle_hal_status_word(), spindle_hal_is_enabled(),
                spindle_hal_orientation_state(), spindle_hal_actual_velocity());
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
            g_spindle.orient_hold_stop = true;
            g_spindle.reset_requires_start = true;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            g_spindle.start_request_pending = false;
            g_spindle.orient_elapsed_cycles = 0U;
        }
    }

    /* 6) M19 完成握手。 */
    if (g_spindle.orient_phase == SPINDLE_ORIENT_DONE &&
        g_spindle.m19_active)
    {
        g_spindle.m19_active = false;
        spindle_internal_report_mcode_complete();
    }
}

/* 内部函数定义 */

static void spindle_orientation_apply_deferred_start(void)
{
    if (spindle_internal_stop_is_required(&g_spindle) || g_spindle.reset_requires_start)
    {
        g_spindle.deferred_start_pending = false;
        g_spindle.start_request_pending = false;
        return;
    }
    if (g_spindle.m29_active)
    {
        return;
    }

    /* 后到的 CNC/PLC 启动命令优先于先前排队的延迟请求。 */
    if (g_spindle.deferred_start_pending && !g_spindle.start_request_pending)
    {
        g_spindle.command_state = g_spindle.deferred_start_state;
        g_spindle.command_speed = g_spindle.deferred_start_speed;
        g_spindle.start_request_pending = true;
    }
    g_spindle.deferred_start_pending = false;

    if (!g_spindle.start_request_pending)
    {
        return;
    }
    if (g_spindle.command_state == SPINDLE_STOP)
    {
        /* 最新命令是停止：不能拿排队的老请求把它顶回去。 */
        g_spindle.start_request_pending = false;
        return;
    }
    spindle_internal_handle_rotation(&g_spindle, g_spindle.command_state);
}
