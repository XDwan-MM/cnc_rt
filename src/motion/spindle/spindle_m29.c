/**
 * @file spindle_m29.c
 * @brief 主轴 M29 与 CSP 准备状态机。
 *
 * 本文件维护两条互斥流程：普通 M29 先执行驱动器定向，再释放定向并
 * 切换到 CSP；不定向 M29 则在 CSV 中受控停车，在当前机械位置建立
 * 软件零点后切换 CSP。外部只观察 active/error/ready，不得修改阶段。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

#define SPINDLE_CONTROL_PERIOD_SECONDS       (0.001)
#define SPINDLE_SPEED_EPSILON                (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON          (1.0)
#define SPINDLE_CSP_POSITION_EPSILON         (10.0)
#define SPINDLE_M29_STABLE_CYCLES            (10U)
#define SPINDLE_M29_MODE_TIMEOUT_CYCLES      (10000U)
#define SPINDLE_ALARM_ORIENT_TIMEOUT         (2100)
#define SPINDLE_ALARM_M29_FIN_TIMEOUT        (2101)
#define SPINDLE_ALARM_M29_PREP_REJECT        (2102)

/* 内部函数声明 */

static bool spindle_m29_read_feedback(HalCAxisStatus *axis,
                                      HalCSpindleStatus *spindle);
static void spindle_m29_no_orient_fail(void);
static int spindle_velocity_direction(double velocity);

/* 外部函数定义 */

bool spindle_m29_is_active(void)
{
    return g_spindle.m29_active;
}

bool spindle_m29_has_error(void)
{
    return g_spindle.m29_phase == SPINDLE_M29_ERROR;
}

void spindle_report_m29_prepare_rejected(void)
{
    setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_M29_PREP_REJECT);
}

void spindle_report_m29_handshake_timeout(void)
{
    setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_M29_FIN_TIMEOUT);
}

void spindle_request_m29(void)
{
    /* M29 是一条全新的主轴操作：清除上一条到速反馈和排队启动，
     * 然后把控制权交给“定向 -> 释放 -> CSP”状态机。 */
    spindle_internal_clear_velocity_arrival();
    /* M29 本身是复位后的新主轴操作请求，允许重新取得驱动器使能。 */
    g_spindle.reset_requires_start = false;
    g_spindle.start_request_pending = true;
    g_spindle.m29_active = true;
    g_spindle.m29_feedback_stable = false;
    g_spindle.m29_phase = SPINDLE_M29_ORIENTING;
    g_spindle.m19_active = false;
    g_spindle.orient_hold_stop = true;
    /* M29 是新的主轴操作请求，取消排队中的 M3/M4。 */
    g_spindle.deferred_start_pending = false;
    g_spindle.pending_event = SPINDLE_EVENT_ORIENT;
    g_spindle.command_speed = 0.0;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.start_request_pending = false;
}

bool spindle_request_m29_no_orient(void)
{
    HalCAxisStatus axis;
    HalCSpindleStatus spindle;
    int32_t orient;
    int orient_result;
    bool stop_required;
    bool mode_switching;
    bool wait_for_csv;

    stop_required = spindle_internal_stop_is_required(&g_spindle);
    mode_switching = spindle_hal_mode_switch_is_pending();
    if (stop_required || rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->axis_data[AXIS_SP].status.is_disable ||
        rt_data->axis_data[AXIS_SP].status.is_error ||
        g_spindle.m29_active ||
        g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_request_pending)
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：stop=%d estop=%d disable=%d error=%d "
            "active=%d switching=%d csv_pending=%d orient_phase=%d orient_pending=%d",
            stop_required, rt_data->sys_state[SYS_ESTOP_INPUT],
            rt_data->axis_data[AXIS_SP].status.is_disable,
            rt_data->axis_data[AXIS_SP].status.is_error, g_spindle.m29_active,
            mode_switching, g_spindle.csv_switch_pending,
            g_spindle.orient_phase, g_spindle.orient_request_pending);
        return false;
    }

    orient_result = rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient);
    if (orient_result != HAL_OK || orient != HAL_ORIENT_NONE)
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：HAL定向状态 rc=%d state=%d",
            orient_result, orient_result == HAL_OK ? orient : -1);
        return false;
    }
    if (!spindle_m29_read_feedback(&axis, &spindle))
    {
        return false;
    }
    if ((axis.raw_status & 0x0008U) != 0U ||
        (axis.error_code_valid && axis.error_code != 0U))
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：主轴故障 status=0x%04X error=0x%04X",
            (unsigned)axis.raw_status, (unsigned)axis.error_code);
        return false;
    }
    if (!isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) ||
        rt_data->axis_data[AXIS_SP].max_feed_acc <= 0.0)
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：主轴进给加速度无效 %.3f",
            rt_data->axis_data[AXIS_SP].max_feed_acc);
        return false;
    }
    if (spindle.mode != HAL_SPINDLE_CSV &&
        spindle.mode != HAL_SPINDLE_CSP &&
        fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON)
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：未知模式%d下主轴仍在转 %.1f rpm",
            spindle.mode, axis.actual_speed_rpm);
        return false;
    }

    /* 复位可能保留中断前的 CSP 目标。只有静止且位置已经跟随时，
     * 才能在不定向流程中接管该目标，避免突然回到旧位置。 */
    if (spindle.mode == HAL_SPINDLE_CSP &&
        (fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON ||
         (!g_spindle.reset_requires_start &&
          fabs(axis.command_pos - axis.actual_pos) >
              SPINDLE_CSP_POSITION_EPSILON)))
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：CSP未停稳或位置未跟随 "
            "rpm=%.3f 目标=%.3f 实际=%.3f",
            axis.actual_speed_rpm, axis.command_pos, axis.actual_pos);
        return false;
    }
    if (spindle.mode == HAL_SPINDLE_CSP && !mode_switching &&
        !g_spindle.csv_switch_pending &&
        rt_hal_request_axis_pos(RT_HAL_AXIS_SP, axis.actual_pos) != HAL_OK)
    {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：CSP位置保持请求失败");
        return false;
    }

    wait_for_csv =
        (mode_switching || g_spindle.csv_switch_pending ||
         (spindle.mode != HAL_SPINDLE_CSV &&
          spindle.mode != HAL_SPINDLE_CSP));
    if (wait_for_csv &&
        spindle_hal_request_mode(HAL_SPINDLE_CSV) != HAL_OK)
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：CSV模式请求失败 rc=%d",
            spindle_hal_mode_switch_result());
        return false;
    }

    /* 从这里开始，普通 M3/M4/M5 和 M19 不再拥有控制权。规划器以当前
     * 实际速度为起点减速，避免模式交接时出现目标速度阶跃。 */
    g_spindle.m29_active = true;
    g_spindle.m29_feedback_stable = false;
    g_spindle.m29_phase = wait_for_csv ?
        SPINDLE_M29_NO_ORIENT_WAIT_CSV : SPINDLE_M29_NO_ORIENT_STOPPING;
    g_spindle.csv_switch_pending = false;
    g_spindle.m29_stop_stable_cycles = 0U;
    g_spindle.orient_elapsed_cycles = 0U;
    g_spindle.reset_requires_start = false;
    g_spindle.m19_active = false;
    g_spindle.orient_hold_stop = true;
    g_spindle.orient_release_pending = false;
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;
    g_spindle.pending_event = SPINDLE_EVENT_NONE;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.run_state = SPINDLE_STOP;
    g_spindle.command_speed = 0.0;
    g_spindle.velocity_planner.current_vel = axis.actual_speed_rpm;
    g_spindle.velocity_planner.target_vel = 0.0;
    g_spindle.velocity_planner.enable = false;
    g_spindle_hal.target_position = axis.actual_pos;
    spindle_internal_clear_velocity_arrival();
    if (!wait_for_csv)
    {
        spindle_hal_mode_switch_reset();
    }

    LOG(LOG_LEVEL_INFO,
        "M29无预定向：初始模式=%d 切换中=%d 当前位置=%.3f "
        "旧目标=%.3f，准备停稳后切CSP",
        spindle.mode, mode_switching, axis.actual_pos, axis.command_pos);
    return true;
}

void spindle_internal_update_m29_no_orient(void)
{
    HalCAxisStatus axis;
    HalCSpindleStatus spindle;
    int32_t orient;
    int velocity_direction;
    int set_position_result;
    bool abort_required;

    abort_required =
        (spindle_internal_stop_is_required(&g_spindle) ||
         rt_data->sys_state[SYS_ESTOP_INPUT] ||
         rt_data->axis_data[AXIS_SP].status.is_disable ||
         rt_data->axis_data[AXIS_SP].status.is_error ||
         !spindle_m29_read_feedback(&axis, &spindle) ||
         !isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) ||
         rt_data->axis_data[AXIS_SP].max_feed_acc <= 0.0 ||
         (axis.raw_status & 0x0008U) != 0U ||
         (axis.error_code_valid && axis.error_code != 0U) ||
         rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient) != HAL_OK ||
         orient != HAL_ORIENT_NONE ||
         g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
         g_spindle.pending_event == SPINDLE_EVENT_RELEASE_ORIENT ||
         ++g_spindle.orient_elapsed_cycles >=
             SPINDLE_M29_MODE_TIMEOUT_CYCLES);
    if (abort_required)
    {
        spindle_m29_no_orient_fail();
        return;
    }

    /* PLC 的方向边沿不能接管准备流程；不产生 M19 完成或 F45.7。 */
    g_spindle.pending_event = SPINDLE_EVENT_NONE;
    spindle_hal_request_torque();

    if (g_spindle.m29_phase == SPINDLE_M29_NO_ORIENT_WAIT_CSV)
    {
        if (g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_IDLE &&
            spindle_hal_request_mode(HAL_SPINDLE_CSV) != HAL_OK)
        {
            spindle_m29_no_orient_fail();
            return;
        }
        if (!spindle_hal_mode_switch_is_done() ||
            spindle.mode != HAL_SPINDLE_CSV)
        {
            return;
        }
        g_spindle.m29_phase = SPINDLE_M29_NO_ORIENT_STOPPING;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_NO_ORIENT_STOPPING)
    {
        if (spindle.mode == HAL_SPINDLE_CSV)
        {
            g_spindle.velocity_planner.max_acc =
                rt_data->axis_data[AXIS_SP].max_feed_acc;
            (void)spindle_velocity_planner_update(
                &g_spindle.velocity_planner, SPINDLE_CONTROL_PERIOD_SECONDS);
            g_spindle_hal.target_velocity =
                g_spindle.velocity_planner.current_vel;
            velocity_direction = spindle_velocity_direction(
                g_spindle_hal.target_velocity);
            if (rt_hal_request_spindle_speed(
                    RT_HAL_AXIS_SP,
                    fabs(g_spindle_hal.target_velocity),
                    velocity_direction) != HAL_OK)
            {
                spindle_m29_no_orient_fail();
                return;
            }
        }

        if (fabs(axis.actual_speed_rpm) <= SPINDLE_ACTUAL_ZERO_EPSILON &&
            (spindle.mode == HAL_SPINDLE_CSP ||
             fabs(g_spindle.velocity_planner.current_vel) <=
                 SPINDLE_SPEED_EPSILON))
        {
            ++g_spindle.m29_stop_stable_cycles;
        }
        else
        {
            g_spindle.m29_stop_stable_cycles = 0U;
        }

        if (g_spindle.m29_stop_stable_cycles >=
            SPINDLE_M29_STABLE_CYCLES)
        {
            if (spindle_hal_request_mode(HAL_SPINDLE_CSP) != HAL_OK)
            {
                spindle_m29_no_orient_fail();
                return;
            }
            g_spindle.m29_phase = SPINDLE_M29_NO_ORIENT_SWITCHING;
            g_spindle.orient_elapsed_cycles = 0U;
        }
        return;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_NO_ORIENT_SWITCHING)
    {
        if (!spindle_hal_mode_switch_is_done() ||
            spindle.mode != HAL_SPINDLE_CSP || !axis.enabled ||
            (axis.raw_status & 0x006FU) != 0x0027U ||
            fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON)
        {
            return;
        }

        /* HAL 切 CSP 时预置实际位置。这里只平移软件坐标，不命令主轴
         * 回机械零度；确认设零成功后才写零位置保持目标。 */
        if (rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0) != HAL_OK)
        {
            spindle_m29_no_orient_fail();
            return;
        }
        g_spindle.m29_phase = SPINDLE_M29_NO_ORIENT_ALIGNING;
        g_spindle.orient_elapsed_cycles = 0U;
        g_spindle.m29_stop_stable_cycles = 0U;
        return;
    }

    set_position_result = rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP);
    if (set_position_result == HAL_ERROR_BUSY)
    {
        return;
    }
    if (set_position_result != HAL_OK)
    {
        spindle_m29_no_orient_fail();
        return;
    }
    if (spindle.mode != HAL_SPINDLE_CSP || !axis.enabled ||
        (axis.raw_status & 0x006FU) != 0x0027U ||
        fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON ||
        fabs(axis.actual_pos) > SPINDLE_CSP_POSITION_EPSILON ||
        fabs(axis.command_pos) > SPINDLE_CSP_POSITION_EPSILON)
    {
        g_spindle.m29_stop_stable_cycles = 0U;
        return;
    }
    if (++g_spindle.m29_stop_stable_cycles < SPINDLE_M29_STABLE_CYCLES)
    {
        return;
    }
    if (rt_hal_request_axis_pos(RT_HAL_AXIS_SP, 0.0) != HAL_OK)
    {
        spindle_m29_no_orient_fail();
        return;
    }

    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur = 0.0;
    g_spindle_hal.target_position = 0.0;
    g_spindle.m29_phase = SPINDLE_M29_READY;
    g_spindle.m29_feedback_stable = true;
    LOG(LOG_LEVEL_INFO,
        "M29无预定向：CSP使能与起点对齐已确认，等待PLC握手");
}


bool spindle_m29_is_ready(void)
{
    /* 只看 READY 枚举不够（那只是本地状态机到位）：必须再核对实际模式切换结果、
     * 实际 CSP、使能状态与 PDO 状态字。 */
    if (spindle_internal_stop_is_required(&g_spindle))
        return false;
    if (!g_spindle.m29_active || g_spindle.reset_requires_start)
        return false;
    if (g_spindle.m29_phase != SPINDLE_M29_READY ||
        !g_spindle.m29_feedback_stable)
        return false;
    if (!spindle_hal_mode_switch_is_done())
        return false;
    if (spindle_hal_mode() != HAL_SPINDLE_CSP)
        return false;
    if (!spindle_hal_is_enabled())
        return false;
    return (spindle_hal_status_word() & 0x006FU) == 0x0027U;
}

void spindle_complete_m29(void)
{
    if (g_spindle.m29_phase != SPINDLE_M29_READY)
    {
        return;
    }

    /* PLC 已返回 FIN。握手结束后继续保持 CSP、Operation Enabled 和当前位置；
     * 后续 M3/M4 会显式退出 READY，并执行安全的 CSP -> CSV 控制权交接。 */
    g_spindle.m29_active = false;
    g_spindle.m29_phase = SPINDLE_M29_READY;
    g_spindle.orient_hold_stop = true;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.command_speed = 0.0;
    spindle_internal_clear_velocity_arrival();   /* 清旧到速反馈 + F45.3 */
}

void spindle_internal_publish_rigid_state(void)
{
    rt_data->sys_state[SYS_IS_RIGID] =
        (g_spindle.reset_stop_pending && g_spindle.reset_stop_csp) ||
        (g_spindle.m29_phase != SPINDLE_M29_IDLE &&
         g_spindle.m29_phase != SPINDLE_M29_ERROR);
}

bool spindle_internal_m29_no_orient_preparing(void)
{
    return g_spindle.m29_phase >= SPINDLE_M29_NO_ORIENT_WAIT_CSV &&
           g_spindle.m29_phase <= SPINDLE_M29_NO_ORIENT_ALIGNING;
}

void spindle_internal_update_m29(void)
{
    if (!g_spindle.m29_active)
    {
        return;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_ORIENTING &&
        (g_spindle.orient_phase == SPINDLE_ORIENT_RELEASING ||
         g_spindle.orient_phase == SPINDLE_ORIENT_IDLE))
    {
        LOG(LOG_LEVEL_ERR,
            "M29主轴定向失败：orient_phase=%d status_word=0x%04X "
            "actual_vel=%.0f",
            g_spindle.orient_phase, spindle_hal_status_word(),
            spindle_hal_actual_velocity());
        g_spindle.m29_phase = SPINDLE_M29_ERROR;
        g_spindle.m29_feedback_stable = false;
        g_spindle.reset_requires_start = true;
        spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        g_spindle.start_request_pending = false;
        g_spindle.orient_hold_stop = true;
        g_spindle.command_speed = 0.0;
        g_spindle.command_state = SPINDLE_STOP;
        return;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_ORIENTING &&
        g_spindle.orient_phase == SPINDLE_ORIENT_DONE)
    {
        if (spindle_orientation_release())
        {
            g_spindle.m29_phase = SPINDLE_M29_RELEASING;
            g_spindle.orient_phase = SPINDLE_ORIENT_RELEASING;
        }
        else
        {
            g_spindle.m29_phase = SPINDLE_M29_ERROR;
            g_spindle.m29_feedback_stable = false;
            g_spindle.reset_requires_start = true;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            g_spindle.orient_hold_stop = true;
        }
    }

    /* 释放后 HAL 会重新使能，故判据是"定向已交回 + 主轴真的停了"，不是 Switch On Disabled；
     * 位置目标等 CSP 到位后再写（CSV 下的位置写会被 HAL 以 MODE_MISMATCH 拒收）。 */
    if (g_spindle.m29_phase == SPINDLE_M29_RELEASING &&
        spindle_hal_orientation_is_inactive() &&
        fabs((double)spindle_hal_actual_velocity()) <= SPINDLE_ACTUAL_ZERO_EPSILON)
    {
        spindle_internal_sync_planner_to_actual();
        g_spindle_hal.target_position = spindle_hal_actual_position();
        LOG(LOG_LEVEL_INFO,
            "M29定向停止确认，切换CSP保持：actual_pos=%.0f actual_vel=%.0f "
            "status_word=0x%04X",
            spindle_hal_actual_position(), spindle_hal_actual_velocity(),
            spindle_hal_status_word());
        if (spindle_hal_request_mode(HAL_SPINDLE_CSP) == HAL_OK)
        {
            g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
            g_spindle.m29_phase = SPINDLE_M29_SWITCHING_CSP;
            g_spindle.orient_elapsed_cycles = 0U;
        }
        else
        {
            g_spindle.m29_phase = SPINDLE_M29_ERROR;
            g_spindle.m29_feedback_stable = false;
            g_spindle.reset_requires_start = true;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            g_spindle.orient_hold_stop = true;
        }
    }

    if (g_spindle.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        if (spindle_hal_mode() == HAL_SPINDLE_CSP && spindle_hal_is_enabled())
        {
            g_spindle.m29_phase = SPINDLE_M29_READY;
            /* 攻丝起点：把当前角度定为 0 度（插补算的是从本段起点起的绝对累计角
             * acc/螺距*360，必须与这里的零点对齐，否则第一个目标角会先跳回旧零点）。
             * 先设零、再写 0 度保持（适配层同拍按此顺序消费），此后角度由插补每拍下发。 */
            rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur = 0.0;
            g_spindle_hal.target_position = 0.0;
            (void)rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0);
            (void)rt_hal_request_axis_pos(RT_HAL_AXIS_SP, 0.0);
        }
        else if (spindle_hal_mode_switch_result() != HAL_OK)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP失败：status_word=0x%04X enabled=%d",
                spindle_hal_status_word(), spindle_hal_is_enabled());
            g_spindle.m29_phase = SPINDLE_M29_ERROR;
            g_spindle.m29_feedback_stable = false;
            g_spindle.reset_requires_start = true;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            g_spindle.start_request_pending = false;
            g_spindle.orient_hold_stop = true;
            g_spindle.command_speed = 0.0;
            g_spindle.command_state = SPINDLE_STOP;
        }
        else if (++g_spindle.orient_elapsed_cycles >=
                 SPINDLE_M29_MODE_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP超时：status_word=0x%04X enabled=%d act_mode=%d",
                spindle_hal_status_word(), spindle_hal_is_enabled(), spindle_hal_mode());
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            g_spindle.m29_phase = SPINDLE_M29_ERROR;
            g_spindle.m29_feedback_stable = false;
            g_spindle.reset_requires_start = true;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            g_spindle.start_request_pending = false;
            g_spindle.orient_hold_stop = true;
            g_spindle.command_speed = 0.0;
            g_spindle.command_state = SPINDLE_STOP;
            g_spindle.orient_elapsed_cycles = 0U;
        }
    }
}

void spindle_internal_update_m29_feedback(void)
{
    static bool feedback_active;
    static uint32_t unsettled_cycles;
    static uint32_t stable_cycles;
    bool complete;
    double position_error;

    if (!g_spindle.m29_active ||
        g_spindle.m29_phase != SPINDLE_M29_READY)
    {
        stable_cycles = 0U;
        unsettled_cycles = 0U;
        g_spindle.m29_feedback_stable = false;
    }

    position_error = fabs((double)g_spindle_hal.target_position -
                          (double)spindle_hal_actual_position());
    complete = g_spindle.m29_active &&
               g_spindle.m29_phase == SPINDLE_M29_READY &&
               spindle_hal_mode() == HAL_SPINDLE_CSP &&
               spindle_hal_is_enabled() &&
               fabs((double)spindle_hal_actual_velocity()) <=
                   SPINDLE_ACTUAL_ZERO_EPSILON &&
               position_error <= SPINDLE_CSP_POSITION_EPSILON;
    if (complete)
    {
        unsettled_cycles = 0U;
        stable_cycles++;
        if (stable_cycles < SPINDLE_M29_STABLE_CYCLES)
        {
            return;
        }
        g_spindle.m29_feedback_stable = true;
        if (!feedback_active)
        {
            LOG(LOG_LEVEL_INFO,
                "M29定向及CSP切换完成：status_word=0x%04X "
                "act_mode=%d enabled=%d target_pos=%.0f actual_pos=%.0f "
                "actual_vel=%.0f position_error=%.0f",
                spindle_hal_status_word(), spindle_hal_mode(), spindle_hal_is_enabled(),
                g_spindle_hal.target_position, spindle_hal_actual_position(),
                spindle_hal_actual_velocity(), position_error);
            feedback_active = true;
        }
    }
    else
    {
        g_spindle.m29_feedback_stable = false;
        feedback_active = false;
        stable_cycles = 0U;
        if (g_spindle.m29_active &&
            g_spindle.m29_phase == SPINDLE_M29_READY &&
            (++unsettled_cycles % 1000U) == 0U)
        {
            LOG(LOG_LEVEL_WARN,
                "M29 CSP尚未稳定：target_pos=%.0f "
                "actual_pos=%.0f actual_vel=%.0f position_error=%.0f "
                "act_mode=%d enabled=%d status_word=0x%04X",
                g_spindle_hal.target_position, spindle_hal_actual_position(),
                spindle_hal_actual_velocity(), position_error,
                spindle_hal_mode(), spindle_hal_is_enabled(),
                spindle_hal_status_word());
        }
    }
}

/* 内部函数定义 */

static bool spindle_m29_read_feedback(HalCAxisStatus *axis, HalCSpindleStatus *spindle)
{
    const int axis_rc = rt_hal_axis_status(RT_HAL_AXIS_SP, axis);
    const int spindle_rc = rt_hal_spindle_status(RT_HAL_AXIS_SP, spindle);
    if (axis_rc != HAL_OK || spindle_rc != HAL_OK)
    {
        LOG(LOG_LEVEL_ERR, "M29无预定向反馈不可读：axis_rc=%d spindle_rc=%d",
            axis_rc, spindle_rc);
        return false;
    }
    if (!axis->speed_valid || !axis->position_valid ||
        !isfinite(axis->actual_speed_rpm) || !isfinite(axis->actual_pos) ||
        !isfinite(axis->command_pos) || !isfinite(spindle->position_deg))
    {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向反馈无效：speed_valid=%d position_valid=%d "
            "rpm=%.3f actual_pos=%.3f command_pos=%.3f spindle_pos=%.3f",
            axis->speed_valid, axis->position_valid, axis->actual_speed_rpm,
            axis->actual_pos, axis->command_pos, spindle->position_deg);
        return false;
    }
    return true;
}

static int spindle_velocity_direction(double velocity)
{
    if (velocity > 0.0)
    {
        return 1;
    }
    if (velocity < 0.0)
    {
        return -1;
    }
    return 0;
}

static void spindle_m29_no_orient_fail(void)
{
    g_spindle.m29_phase = SPINDLE_M29_ERROR;
    g_spindle.m29_feedback_stable = false;
    g_spindle.m29_active = false;
    g_spindle.reset_requires_start = true;
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.run_state = SPINDLE_STOP;
    g_spindle.command_speed = 0.0;
    g_spindle.velocity_planner.enable = false;
    g_spindle.velocity_planner.target_vel = 0.0;
    g_spindle_hal.target_velocity = 0.0;
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
    spindle_internal_clear_velocity_arrival();
    spindle_hal_mode_switch_reset();
    spindle_hal_release_torque();
    setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
    LOG(LOG_LEVEL_ERR, "M29无预定向准备中止：停止/联锁、反馈或请求异常、等待超时");
}

