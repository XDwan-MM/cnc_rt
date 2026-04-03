#include "stored_stroke_check.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>

AxisLimitRuntime runtime[SYS_AXES_NUM];

//// 全局限位管理器
//typedef struct
//{
//    AxisLimitRuntime* axes[3];
//    uint8_t axis_count;
//    uint8_t global_enable;
//} LimitManager;

uint8_t Axis_Limit_Manager_Init(void)
{
    runtime[AXIS_X].axis = AXIS_X;
    runtime[AXIS_Y].axis = AXIS_Y;
    runtime[AXIS_Z].axis = AXIS_Z;

    //level 1
    runtime[AXIS_X].config.level1.is_enabled = 1;
    runtime[AXIS_X].config.level1.positive_limit = par_axis[AXIS_X][PAR_AX_PLMT].value.val_real;
    runtime[AXIS_X].config.level1.negative_limit = par_axis[AXIS_X][PAR_AX_NLMT].value.val_real;
    runtime[AXIS_Y].config.level1.is_enabled = 1;
    runtime[AXIS_Y].config.level1.positive_limit = par_axis[AXIS_Y][PAR_AX_PLMT].value.val_real;
    runtime[AXIS_Y].config.level1.negative_limit = par_axis[AXIS_Y][PAR_AX_NLMT].value.val_real;
    runtime[AXIS_Z].config.level1.is_enabled = 1;
    runtime[AXIS_Z].config.level1.positive_limit = par_axis[AXIS_Z][PAR_AX_PLMT].value.val_real;
    runtime[AXIS_Z].config.level1.negative_limit = par_axis[AXIS_Z][PAR_AX_NLMT].value.val_real;
    //level 2
    runtime[AXIS_X].config.level2.is_enabled = 0;
    runtime[AXIS_X].config.level2_mode = 0;
    runtime[AXIS_X].config.level2.positive_limit = par_axis[AXIS_X][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_X].config.level2.negative_limit = par_axis[AXIS_X][PAR_AX_NLMT2].value.val_real;
    runtime[AXIS_Y].config.level2.is_enabled = 0;
    runtime[AXIS_Y].config.level2_mode = 0;
    runtime[AXIS_Y].config.level2.positive_limit = par_axis[AXIS_Y][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_Y].config.level2.negative_limit = par_axis[AXIS_Y][PAR_AX_NLMT2].value.val_real;
    runtime[AXIS_Z].config.level2.is_enabled = 0;
    runtime[AXIS_Z].config.level2_mode = 0;
    runtime[AXIS_Z].config.level2.positive_limit = par_axis[AXIS_Z][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_Z].config.level2.negative_limit = par_axis[AXIS_Z][PAR_AX_NLMT2].value.val_real;
    //level 3
    runtime[AXIS_X].config.level3.is_enabled = 0;
    runtime[AXIS_X].config.level3.positive_limit = 3000;
    runtime[AXIS_X].config.level3.negative_limit = 2000;
    runtime[AXIS_Y].config.level3.is_enabled = 0;
    runtime[AXIS_Y].config.level3.positive_limit = 3000;
    runtime[AXIS_Y].config.level3.negative_limit = 2000;
    runtime[AXIS_Z].config.level3.is_enabled = 0;
    runtime[AXIS_Z].config.level3.positive_limit = 3000;
    runtime[AXIS_Z].config.level3.negative_limit = 2000;

}
// 检查位置是否在限位区域内部
uint8_t Is_Inside_Limit(float position, LimitLevel limit)
{
    return (position >= limit.negative_limit) && (position <= limit.positive_limit);
}

// 检查位置是否在限位区域外部
uint8_t Is_Outside_Limit(float position, LimitLevel limit)
{
    return (position < limit.negative_limit) || (position > limit.positive_limit);
}

// 单轴限位检查主函数
uint8_t Check_Axis_Limit_Violation(AxisLimitRuntime runtime, bool moving)
{
    double position = 0;
    uint8_t violation = 0;
    if(moving)
    {
        position = runtime.current_position;
    }
    else
    {
        position = runtime.parser_position;
    }


    // 第一级检查：必须在区域内部运动
    if (runtime.config.level1.is_enabled && Is_Outside_Limit(position, runtime.config.level1))
    {
        violation = 1; // 一级限位违规
        runtime.violation_flag |= 0x01;
        if(position > runtime.config.level1.positive_limit)
        {
            runtime.violation_flag |= 0x08;
            if(moving)
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 0);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "朝正方向移动超过了存储行程检查1");
            }
            else
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
            }
        }
        else
        {
            if(moving)
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 1);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "朝负方向移动超过了存储行程检查1");
            }
            else
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 11);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在负侧行程极限禁止区内");
            }
        }
    }

    // 第三级检查：必须在区域外部运动
    if (runtime.config.level3.is_enabled && Is_Inside_Limit(position, runtime.config.level3))
    {
        violation = 1; // 三级限位违规
        runtime.violation_flag |= 0x04;
        if(position >= runtime.config.level3.negative_limit)
        {
            runtime.violation_flag |= 0x20;
            if(moving)
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 4);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "朝正方向移动超过了存储行程检查3");
            }
            else
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
            }
        }
        else
        {
            if(moving)
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 5);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "朝负方向移动超过了存储行程检查3");
            }
            else
            {
                int res = setAlarm(ALARM_AX, ALARM_ERROR, 11);
                if(res == -8) return violation;
                LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在负侧行程极限禁止区内");
            }
        }
    }

    // 第二级检查：根据模式判断
    if(runtime.config.level2.is_enabled && runtime.config.level2_mode == 0)
    {
        // 内部检查模式
        if(Is_Outside_Limit(position, runtime.config.level2))
        {
            violation = 1;
            runtime.violation_flag |= 0x02;
            if(position > runtime.config.level2.positive_limit)
            {
                runtime.violation_flag |= 0x18;
                if(moving)
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 2);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
                }
            }
            else
            {
                if(moving)
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 3);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
                }
            }
        }
    }
    else if(runtime.config.level2.is_enabled && runtime.config.level2_mode == 1)
    {
        // 外部检查模式
        if (Is_Inside_Limit(position, runtime.config.level2))
        {
            violation = 1;
            runtime.violation_flag |= 0x02;
            if(position >= runtime.config.level2.negative_limit)
            {
                runtime.violation_flag |= 0x18;
                if(moving)
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 2);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
                }
            }
            else
            {
                if(moving)
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 3);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 10);
                    if(res == -8) return violation;
                    LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
                }
            }
        }
    }
    return violation;
}

void lookahead_check_soft_limit(POS *target_pos){
    runtime[0].parser_position = target_pos->x;
    runtime[1].parser_position = target_pos->y;
    runtime[2].parser_position = target_pos->z;
    runtime[0].active_levels = Check_Axis_Limit_Violation(runtime[0], false);
    runtime[1].active_levels = Check_Axis_Limit_Violation(runtime[1], false);
    runtime[2].active_levels = Check_Axis_Limit_Violation(runtime[2], false);
}

//void Axis_Status_Init()
//{
//    for (int var = 0; var < SYS_AXES_NUM; ++var)
//    {
//        rt_data->axis_data[var].axis_status.is_moving = 0;
//        rt_data->axis_data[var].axis_status.direction = 0;
//        rt_data->axis_data[var].axis_status.is_init = 0;
//    }
//}

void Axis_Status_Update()
{
    for (int var = 0; var < SYS_AXES_NUM; ++var)
    {
        rt_data->axis_data[var].axis_status.current_pos = rt_data->axis_data[var].motor_cmd_pos;
        double delta = rt_data->axis_data[var].axis_status.current_pos - rt_data->axis_data[var].axis_status.last_pos;
        rt_data->axis_data[var].axis_status.last_pos = rt_data->axis_data[var].axis_status.current_pos;
        if(!rt_data->axis_data[var].axis_status.is_init)
        {
            rt_data->axis_data[var].axis_status.is_init = 1;
            return ;
        }
        if(delta == 0)
        {
            rt_data->axis_data[var].axis_status.is_moving = 0;
        }
        else if(delta > 0)
        {
            rt_data->axis_data[var].axis_status.is_moving = 1;
            rt_data->axis_data[var].axis_status.direction = 1;
        }
        else if(delta < 0)
        {
            rt_data->axis_data[var].axis_status.is_moving = 1;
            rt_data->axis_data[var].axis_status.direction = 0;
        }
    }
}
