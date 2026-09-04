#include "ferror_control.h"
#include "global.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

ferror_control_t ferror_control[SYS_AXES_NUM]; // 跟随误差控制器

static const char *axis_names[] = {"X", "Y", "Z"};

enum
{
    ALARM_RESULT_ALREADY_EXISTS = -8
};

/* ---------- 内部函数声明 ---------- */
static void check_axis_ferror(ferror_control_t *ferror_ctrl, const RT_AXIS *axis_para, double pos_cmd, double pos_fb, double vel_cmd);
static void report_axis_ferror(int axis_index, const ferror_control_t *ferror_ctrl, double vel_cmd, double pos_cmd, double pos_fb);

/* ---------- 外部函数定义 ---------- */

/*
 * 全轴跟随误差故障检查
 * 在轨迹规划周期中每周期调用一次，遍历所有轴进行跟随误差检测和报警
 */
void check_ferror_fault(void)
{
    /* 报警状态、急停拍下情况、(空闲、编辑状态)时不检查跟随误差，避免上电无运动误报 */
    if (rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] || rt_data->current_mode == MODE_IDLE || rt_data->current_mode == MODE_EDIT)
    {
        return;
    }

    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        // 控制轴拆卸或者机床锁住不进行检查
        if (rt_data->sys_state[SYS_IS_MAC_LOCK_READY] || rt_data->axis_data[i].axis_status.is_disable)
        {
            continue;
        }

        double vel_cmd = rt_data->axis_data[i].cmd_vel;
        double pos_cmd = rt_data->axis_data[i].motor_cmd_pos;
        double pos_fb = rt_data->axis_data[i].motor_act_pos;

        ferror_control_t *ferror_ctrl = &ferror_control[i];

        check_axis_ferror(ferror_ctrl, &rt_data->axis_data[i], pos_cmd, pos_fb, vel_cmd);

        if (ferror_ctrl->error_flag)
        {
            rt_data->axis_data[i].axis_status.is_lag_error = true;
            report_axis_ferror(i, ferror_ctrl, vel_cmd, pos_cmd, pos_fb);
        }
    }
}

void reset_ferror_control(void)
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        ferror_control[i].error_flag = false;
        ferror_control[i].is_moving = false;
        rt_data->axis_data[i].axis_status.is_lag_error = false;
    }
}

/* ---------- 内部函数定义 ---------- */

/*
 * 单轴跟随误差检测
 * 采用 FANUC PRM 1828/1829 双阈值模型：
 *   - 运动中 (|vel_cmd| > FERROR_SPEED_THRESHOLD) 使用 PAR_AX_MAX_FERROR（PRM 1828）
 *   - 停止时 (|vel_cmd| <= FERROR_SPEED_THRESHOLD) 使用 PAR_AX_MIN_FERROR（PRM 1829）
 */
static void check_axis_ferror(ferror_control_t *ferror_ctrl, const RT_AXIS *axis_para, double pos_cmd, double pos_fb, double vel_cmd)
{
    if (ferror_ctrl == NULL || axis_para == NULL)
        return;

    double ferror = fabs(pos_cmd - pos_fb);

    if (fabs(vel_cmd) > FERROR_SPEED_THRESHOLD)
    {
        /* 运动中：使用动态最大跟随误差 */
        ferror_ctrl->ferror_limit = axis_para->max_dynamic_lag;
        ferror_ctrl->is_moving = true;
    }
    else
    {
        /* 停止中：使用静态最大跟随误差 */
        ferror_ctrl->ferror_limit = axis_para->max_static_lag;
        ferror_ctrl->is_moving = false;
    }

    ferror_ctrl->ferror = ferror;

    if (ferror > ferror_ctrl->ferror_limit)
    {
        ferror_ctrl->error_flag = true;
    }
}

static void report_axis_ferror(int axis_index, const ferror_control_t *ferror_ctrl, double vel_cmd, double pos_cmd, double pos_fb)
{
    int alarm_code;
    int alarm_result;
    const char *fault_type;
    bool alarm_already_exists;

    if (ferror_ctrl->is_moving)
    {
        alarm_code = axis_index * 10 + 100; /* 移动时: X=100, Y=110, Z=120 */
        fault_type = "移动时";
    }
    else
    {
        alarm_code = axis_index * 10 + 101; /* 静止时: X=101, Y=111, Z=121 */
        fault_type = "静止时";
    }

    alarm_result = setAlarm(ALARM_AX, ALARM_ERROR, alarm_code);
    alarm_already_exists = (alarm_result == ALARM_RESULT_ALREADY_EXISTS);
    if (alarm_already_exists)
    {
        return;
    }

    LOG(LOG_LEVEL_ERR, "%s轴%s跟随误差过大: 指令速度=%.6f, 指令位置=%.6f, 反馈位置=%.6f, 跟随误差=%.6f", axis_names[axis_index], fault_type, vel_cmd, pos_cmd, pos_fb, ferror_ctrl->ferror);
}
