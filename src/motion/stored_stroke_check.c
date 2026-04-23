#include "stored_stroke_check.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>

AxisLimitRuntime runtime[SYS_AXES_NUM];
static char* axis_names[] = {"X","Y","Z"};

static void Pre_Log_Alarm(bool direction);
static void Data_Log_Alarm(AXIS_TYPE axis ,Level level);
// 检查位置是否在限位区域内部
static uint8_t Is_Inside_Limit(float position, LimitLevel limit);
// 检查位置是否在限位区域外部
static uint8_t Is_Outside_Limit(float position, LimitLevel limit);
static void Outside_Alarm(float position, LimitLevel limit , int premove_id, Level level, AxisLimitRuntime *runtime);
static void Inside_Alarm(AxisLimitRuntime *runtime, int premove_id ,Level level);



void Axis_Limit_Manager_Init(void)
{
       for (int i = 0; i < 3; i++)
    {
        //level 1
        runtime[i].config.level1.is_enabled = 1;
        runtime[i].config.level1.positive_limit = par_axis[i][PAR_AX_PLMT].value.val_real;
        runtime[i].config.level1.negative_limit = par_axis[i][PAR_AX_NLMT].value.val_real;
        //level 2
        runtime[i].config.level2.is_enabled = par_axis[i][503].value.val_int;
        runtime[i].config.level2_mode = par_axis[i][502].value.val_int;
        runtime[i].config.level2.positive_limit = par_axis[i][PAR_AX_PLMT2].value.val_real;
        runtime[i].config.level2.negative_limit = par_axis[i][PAR_AX_NLMT2].value.val_real;
        //level 3
        runtime[AXIS_X].config.level3.is_enabled = par_axis[i][504].value.val_int;
        runtime[AXIS_X].config.level3.positive_limit = par_axis[i][500].value.val_real;
        runtime[AXIS_X].config.level3.negative_limit = par_axis[i][501].value.val_real;

        rt_data->axis_data[i].axis_status.is_fwd_limit = 0;
        rt_data->axis_data[i].axis_status.is_rev_limit = 0;
        rt_data->axis_data[i].axis_status.is_decel_done = 0;
        runtime[i].is_first_run = 1;
        runtime[i].axis = i;
    }
}

// 单轴限位检查主函数
uint8_t Check_Axis_Limit_Violation(AxisLimitRuntime *runtime, double pos,AxisLimitUsedType type)
{
    uint8_t violation = 0;
    AxisLimitConfig config;
    switch (type)
    {
    case ALARM_CHEAK_USED:
        config = runtime->temp_config;
        break;
    case LOOKAHEAD_CHECK_USED:
    case SOFT_CHEAK_USED:
    case RESET_CHEAK_USED:
        config = runtime->config;
        break;
    default:
        break;
    }
    runtime->direction = rt_data->axis_data[runtime->axis].axis_status.direction;
//    printf("config: %f %f %f %f %f %f %d %d %d\n",config.level1.negative_limit,config.level1.positive_limit
//             ,config.level2.negative_limit,config.level2.positive_limit
//             ,config.level3.negative_limit,config.level3.positive_limit
//             ,rt_data->axis_data[runtime->axis].axis_status.is_fwd_limit
//             ,rt_data->axis_data[runtime->axis].axis_status.is_rev_limit,type);
    // 第一级检查：必须在区域内部运动
    if (runtime->config.level1.is_enabled && Is_Outside_Limit(pos, config.level1))
    {
        violation = 1; // 一级限位违规
        runtime->violation_flag |= 0x01;
        switch (type)
        {
        case LOOKAHEAD_CHECK_USED:
            Pre_Log_Alarm(runtime->direction);
            break;    
        case RESET_CHEAK_USED:     
        case ALARM_CHEAK_USED:
        case SOFT_CHEAK_USED:
            Outside_Alarm(pos, config.level1, 0, LEVEL1, runtime);
            break;
        default:
            break;
        }
    }

    // 第三级检查：必须在区域外部运动.
    if (runtime->config.level3.is_enabled && Is_Inside_Limit(pos, config.level3))
    {
        violation = 1; // 三级限位违规
        runtime->violation_flag |= 0x04;
        switch (type)
        {
        case LOOKAHEAD_CHECK_USED:
            Pre_Log_Alarm(runtime->direction);
            break;    
        case RESET_CHEAK_USED:     
            Data_Log_Alarm(runtime->axis, LEVEL3);
            break;
        case ALARM_CHEAK_USED:
        case SOFT_CHEAK_USED:
            Inside_Alarm(runtime, 4 , LEVEL3);
            break;
        default:
            break;
        }
    }
    // 第二级检查：根据模式判断
    if(runtime->config.level2.is_enabled)
    {
        if(runtime->config.level2_mode == 0)
        {
            // 内部检查模式
            if (Is_Outside_Limit(pos, runtime->config.level2))
            {
                violation = 1; // 二级限位违规
                runtime->violation_flag |= 0x02;
                switch (type)
                {
                case LOOKAHEAD_CHECK_USED:
                    Pre_Log_Alarm(runtime->direction);
                    break;    
                case RESET_CHEAK_USED:     
                case ALARM_CHEAK_USED:
                case SOFT_CHEAK_USED:
                    Outside_Alarm(pos, config.level2, 2, LEVEL2, runtime);
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            // 外部检查模式
            if (Is_Inside_Limit(pos, config.level2))
            {
                violation = 1; // 二级限位违规
                runtime->violation_flag |= 0x02;
                switch (type)
                {
                case LOOKAHEAD_CHECK_USED:
                    Pre_Log_Alarm(runtime->direction);
                    break;    
                case RESET_CHEAK_USED:     
                    Data_Log_Alarm(runtime->axis, LEVEL2);
                    break;
                case ALARM_CHEAK_USED:
                case SOFT_CHEAK_USED:
                    Inside_Alarm(runtime, 2 , LEVEL2);
                    break;
                default:
                    break;
                }
            }
        }
    }
    return violation;
}

void lookahead_check_soft_limit(POS *target_pos){
    runtime[0].active_levels = Check_Axis_Limit_Violation(&runtime[0], target_pos->x,LOOKAHEAD_CHECK_USED);
    runtime[1].active_levels = Check_Axis_Limit_Violation(&runtime[1], target_pos->y,LOOKAHEAD_CHECK_USED);
    runtime[2].active_levels = Check_Axis_Limit_Violation(&runtime[2], target_pos->z,LOOKAHEAD_CHECK_USED);
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
static void Outside_Alarm(float position, LimitLevel limit , int premove_id, Level level,AxisLimitRuntime *runtime)
{
    // printf("axis%d \n",runtime->axis);
    if (position < limit.negative_limit)
    {
        rt_data->axis_data[runtime->axis].axis_status.is_rev_limit = 1;
        if(level == 1)
        {
            runtime->limit_type = LEVEL1_REV_LIMIT;
        }
        else
        {
            runtime->limit_type = LEVEL2_REV_LIMIT;
        }
        int res = setAlarm(ALARM_AX, ALARM_ERROR, premove_id + 6 * runtime->axis + 1);
        if(res == -8) return 1;
        LOG(LOG_LEVEL_ERR, "%s 轴朝负方向移动超过了存储行程检查%d", axis_names[runtime->axis],runtime->axis);
        return 1;
    }
    else if(position > limit.positive_limit)
    {
        rt_data->axis_data[runtime->axis].axis_status.is_fwd_limit = 1;
        if(level == 1)
        {
            runtime->limit_type = LEVEL1_FWD_LIMIT;
        }
        else
        {
            runtime->limit_type = LEVEL2_FWD_LIMIT;
        }
        int res = setAlarm(ALARM_AX, ALARM_ERROR, premove_id + 6 * runtime->axis);
        if(res == -8) return 1;
        LOG(LOG_LEVEL_ERR, "%s 轴朝正方向移动超过了存储行程检查%d", axis_names[runtime->axis],runtime->axis);
        return 1;
    }
    else
    {
        return 0;
    }
}

static void Inside_Alarm(AxisLimitRuntime *runtime, int premove_id , Level level)
{
    if(runtime->direction == 1)
    {
        rt_data->axis_data[runtime->axis].axis_status.is_fwd_limit = 1;
        int res = setAlarm(ALARM_AX, ALARM_ERROR, premove_id + 6 * runtime->axis);
        if(res == -8) return ;
        LOG(LOG_LEVEL_ERR, "%s 轴朝正方向移动超过了存储行程检查%d", axis_names[runtime->axis],level);
    }
    else
    {
        rt_data->axis_data[runtime->axis].axis_status.is_rev_limit = 1;
        int res = setAlarm(ALARM_AX, ALARM_ERROR, premove_id + 6 * runtime->axis + 1);
        if(res == -8) return ;
        LOG(LOG_LEVEL_ERR, "%s 轴朝负方向移动超过了存储行程检查%d", axis_names[runtime->axis],level);
    }
    switch (level)
    {
    case 1:
        if(rt_data->axis_data[runtime->axis].axis_status.direction == 0)
        {
            runtime->limit_type = LEVEL1_REV_LIMIT;
        }
        else
        {
            runtime->limit_type = LEVEL1_FWD_LIMIT;
        }
        break;
    case 2:
        if(rt_data->axis_data[runtime->axis].axis_status.direction == 0)
        {
            runtime->limit_type = LEVEL2_REV_LIMIT;
        }
        else
        {
            runtime->limit_type = LEVEL2_FWD_LIMIT;
        }
        break;
    case 3:
        if(rt_data->axis_data[runtime->axis].axis_status.direction == 0)
        {
            runtime->limit_type = LEVEL3_REV_LIMIT;
        }
        else
        {
            runtime->limit_type = LEVEL3_FWD_LIMIT;
        }
        break;
    default:
        break;
    }
}
static void Pre_Log_Alarm(bool direction)
{
    if(direction == 1)
    {
        int res = setAlarm(ALARM_AX, ALARM_ERROR, 18);
        if(res == -8) return ;
        LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在正侧行程极限禁止区内");
    }
    else
    {
        int res = setAlarm(ALARM_AX, ALARM_ERROR, 19);
        if(res == -8) return ;
        LOG(LOG_LEVEL_ERR, "移动前行程检查中程序段终点位置在负侧行程极限禁止区内");
    }
}
static void Data_Log_Alarm(AXIS_TYPE axis ,Level level)
{
    rt_data->axis_data[axis].axis_status.is_fwd_limit = 1;
    rt_data->axis_data[axis].axis_status.is_rev_limit = 1;
    int res = setAlarm(ALARM_AX, ALARM_ERROR, 21 + axis * 2 + level - 2);
    if(res == -8) return ;
    LOG(LOG_LEVEL_ERR, "%s 轴处于存储行程检查%d外，请检查限位参数", axis_names[axis], level);
}
