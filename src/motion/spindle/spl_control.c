#include "spl_control.h"
#include "driver.h"
#include "rt_data.h"
#include "spl_vel_planner.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <parapi.h>

SPINDLE_DATA spl_data;

static const double spindle_control_period = 0.001;

/* 内部函数声明 */
static bool spindle_validate_command_speed(const SPINDLE_DATA *data);
static double spindle_calculate_target_velocity(const SPINDLE_DATA *data);
static void spindle_update_planner_state(SPINDLE_DATA *data);
static void spindle_report_mcode_complete(void);

/* 外部函数定义 */
void spindle_set_state(void)
{
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        rt_data->nc_event == SYS_EVENT_HMI_RESET ||
        rt_data->sys_state[SYS_IS_ALARMING] ||
        spl_data.spl_event == SPINDLE_EVENT_STOP)
    {
        spl_data.spl_event = SPINDLE_EVENT_STOP;
    }

    switch (spl_data.spl_event)
    {
    case SPINDLE_EVENT_STOP:
        spl_data.command_speed = 0.0;
        spl_data.spl_state = SPINDLE_STOP;
        /* 停止过程仍保持使能，直到速度规划器降到零。 */
        Driver_Enable(&spindle);
        LOG(LOG_LEVEL_INFO, "主轴收到停止指令，当前速度=%f", spl_data.vel_planner.current_vel);
        break;
    case SPINDLE_EVENT_CW:
        if (spindle_validate_command_speed(&spl_data))
        {
            spl_data.spl_state = SPINDLE_CW;
            spindle.act_mode = DRV_MODE_CSV;
            Driver_Enable(&spindle);
            LOG(LOG_LEVEL_INFO, "主轴收到正转指令，目标速度=%f", spindle_calculate_target_velocity(&spl_data));
        }
        break;
    case SPINDLE_EVENT_CCW:
        if (spindle_validate_command_speed(&spl_data))
        {
            spl_data.spl_state = SPINDLE_CCW;
            spindle.act_mode = DRV_MODE_CSV;
            Driver_Enable(&spindle);
            LOG(LOG_LEVEL_INFO, "主轴收到反转指令，目标速度=%f", spindle_calculate_target_velocity(&spl_data));
        }
        break;
    case SPINDLE_EVENT_NONE:
    case SPINDLE_EVENT_ORIENT:
        /* 主轴定向功能已移除，保留事件入口以兼容上层调用。 */
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
    default:
        break;
    }
    spl_data.spl_event = SPINDLE_EVENT_NONE;
}

void spindle_cal_speed(void)
{
    spl_vel_planner_t *planner = &spl_data.vel_planner;

    planner->max_acc = rt_data->axis_data[AXIS_SP].max_feed_acc;
    planner->enable = (spl_data.spl_state == SPINDLE_CW ||
                       spl_data.spl_state == SPINDLE_CCW);
    planner->target_vel = planner->enable ? spindle_calculate_target_velocity(&spl_data) : 0.0;
    spindle_update_planner_state(&spl_data);

    Hal_Driver_Set_Target_Vel(&spindle, (int32_t)planner->current_vel);

    /* 只有驱动器反馈的实际速度已经为零，才允许切回下使能状态。 */
    if (spl_data.spl_state == SPINDLE_STOP && spindle.actual_vel == 0)
    {
        spl_data.spl_vel_state = SPINDLE_VEL_ZERO;

        spindle.act_mode = DRV_MODE_NONE;
        Driver_Disable(&spindle);
        spindle_report_mcode_complete();
    }
}

bool spindle_is_stopped(void)
{
    return spl_data.spl_state == SPINDLE_STOP;
}

bool spindle_is_rotating(void)
{
    if (spindle_is_stopped())
    {
        return false;
    }
    return true;
}

/* 内部函数定义 */
static bool spindle_validate_command_speed(const SPINDLE_DATA *data)
{
    double command_speed;

    if (data->speed_mode)
    {
        command_speed = data->plc_speed;
    }
    else
    {
        command_speed = data->command_speed;
    }

    if (fabs(command_speed) > 0.0)
    {
        return true;
    }

    if (setAlarm(ALARM_AX, ALARM_ERROR, 20) == -8)
    {
        return false;
    }
    LOG(LOG_LEVEL_ERR, "主轴速度值缺省");
    return false;
}

static double spindle_calculate_target_velocity(const SPINDLE_DATA *data)
{
    double max_speed = fabs(rt_data->axis_data[AXIS_SP].spdl_max_speed);
    double override = var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100.0;
    double speed;

    if (data->speed_mode)
    {
        speed = fabs(data->plc_speed);
    }
    else
    {
        speed = fabs(data->command_speed) * override;
    }
    speed = fmin(max_speed, speed);
    if (data->spl_state == SPINDLE_CCW)
    {
        speed = -speed;
    }
    return speed;
}

static void spindle_update_planner_state(SPINDLE_DATA *data)
{
    bool arrived;

    arrived = spl_vel_planner_update(&data->vel_planner, spindle_control_period);
    if (!data->vel_planner.active)
    {
        data->spl_vel_state = SPINDLE_VEL_ZERO;
    }
    else if (arrived)
    {
        data->spl_vel_state = SPINDLE_VEL_ARRIVED;
        spindle_report_mcode_complete();
    }
    else
    {
        data->spl_vel_state = SPINDLE_VEL_RAMP;
    }
}

static void spindle_report_mcode_complete(void)
{
    if (!spl_data.mcode_feedback)
    {
        spl_data.mcode_feedback = true;
        LOG(LOG_LEVEL_DEBUG, "主轴速度到达 %f", spl_data.vel_planner.current_vel);
    }
}
