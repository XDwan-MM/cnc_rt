/********************************************************************
 * @file        ferror_control.c
 * @brief       跟随误差控制功能（FANUC PRM 1828/1829 双阈值模型）
 *             - PRM 1828: 移动中各轴的位置偏差极限值
 *             - PRM 1829: 停止时各轴的位置偏差极限值
 * @version     V2.00
 * @date        2026/06/10
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "ferror_control.h"
#include "global.h"
#include "jog.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

ferror_control_t ferror_control[SYS_AXES_NUM]; // 跟随误差控制器

static const char *axis_names[] = {"X", "Y", "Z"};

/* ---------- 内部函数声明 ---------- */
static void check_axis_ferror(ferror_control_t *ferror_ctrl,
                              const ParamUnit *axis_para,
                              double pos_cmd,
                              double pos_fb,
                              double vel_cmd);

/* ---------- 外部函数定义 ---------- */

/*
 * 全轴跟随误差故障检查
 * 在轨迹规划周期中每周期调用一次，遍历所有轴进行跟随误差检测和报警
 */
void check_ferror_fault(void)
{
    /* 报警状态、急停拍下情况、(空闲、编辑状态)时不检查跟随误差，避免上电无运动误报 */
    if (rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        rt_data->current_mode == MODE_IDLE || rt_data->current_mode == MODE_EDIT)
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

        double vel_cmd = var_axis[i][VAR_AXIS_CMD_VEL].value.f;
        double pos_cmd = rt_data->axis_data[i].motor_cmd_pos;
        double pos_fb = rt_data->axis_data[i].motor_act_pos;

        check_axis_ferror(&ferror_control[i], par_axis[i], pos_cmd, pos_fb, vel_cmd);

        if (ferror_control[i].error_flag)
        {
            int alarm_code;
            const char *fault_type;
            if (ferror_control[i].is_moving)
            {
                alarm_code = i * 10 + 100; /* 报警号 移动时: X=100, Y=110, Z=120 */
                fault_type = "移动时";
            }
            else
            {
                alarm_code = i * 10 + 101; /* 报警号 静止时: X=101, Y=111, Z=121 */
                fault_type = "静止时";
            }
            int res = setAlarm(ALARM_AX, ALARM_ERROR, alarm_code);
            if (res == -8)
                continue;
            /* 上述代码逻辑是让下面LOG只打印一次；返回值-8：表示该报警已存在。*/
            LOG(LOG_LEVEL_ERR, "%s轴%s跟随误差过大", axis_names[i], fault_type);
        }
    }
}

void reset_ferror_control(void)
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        ferror_control[i].error_flag = false;
        ferror_control[i].is_moving = false;
    }
}

/* ---------- 内部函数定义 ---------- */

/*
 * 单轴跟随误差检测
 * 采用 FANUC PRM 1828/1829 双阈值模型：
 *   - 运动中 (|vel_cmd| > SPEED_THRESHOLD) 使用 PAR_AX_MAX_FERROR（PRM 1828）
 *   - 停止时 (|vel_cmd| <= SPEED_THRESHOLD) 使用 PAR_AX_MIN_FERROR（PRM 1829）
 */
static void check_axis_ferror(ferror_control_t *ferror_ctrl,
                              const ParamUnit *axis_para,
                              double pos_cmd,
                              double pos_fb,
                              double vel_cmd)
{
    if (ferror_ctrl == NULL || axis_para == NULL)
        return;

    double ferror = fabs(pos_cmd - pos_fb);

    if (fabs(vel_cmd) > SPEED_THRESHOLD)
    {
        /* 运动中：使用动态最大跟随误差 */
        ferror_ctrl->ferror_limit = axis_para[PAR_AX_MAX_DYNAMIC_LAG].value.val_real;
        ferror_ctrl->is_moving = true;
    }
    else
    {
        /* 停止中：使用静态最大跟随误差 */
        ferror_ctrl->ferror_limit = axis_para[PAR_AX_MAX_STATIC_LAG].value.val_real;
        ferror_ctrl->is_moving = false;
    }

    ferror_ctrl->ferror = ferror;
    ferror_ctrl->error_flag = (ferror > ferror_ctrl->ferror_limit);
}
