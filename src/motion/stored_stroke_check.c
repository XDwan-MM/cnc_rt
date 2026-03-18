#include "stored_stroke_check.h"
#include "global.h"
#include "log.h"
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
    runtime[AXIS_X].config.level2.is_enabled = 1;
    runtime[AXIS_X].config.level2_mode = 0;
    runtime[AXIS_X].config.level2.positive_limit = par_axis[AXIS_X][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_X].config.level2.negative_limit = par_axis[AXIS_X][PAR_AX_NLMT2].value.val_real;
    runtime[AXIS_Y].config.level2.is_enabled = 1;
    runtime[AXIS_Y].config.level2_mode = 0;
    runtime[AXIS_Y].config.level2.positive_limit = par_axis[AXIS_Y][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_Y].config.level2.negative_limit = par_axis[AXIS_Y][PAR_AX_NLMT2].value.val_real;
    runtime[AXIS_Z].config.level2.is_enabled = 1;
    runtime[AXIS_Z].config.level2_mode = 0;
    runtime[AXIS_Z].config.level2.positive_limit = par_axis[AXIS_Z][PAR_AX_PLMT2].value.val_real;
    runtime[AXIS_Z].config.level2.negative_limit = par_axis[AXIS_Z][PAR_AX_NLMT2].value.val_real;
    //level 3
    runtime[AXIS_X].config.level3.is_enabled = 1;
    runtime[AXIS_X].config.level3.positive_limit = 300;
    runtime[AXIS_X].config.level3.negative_limit = 200;
    runtime[AXIS_Y].config.level3.is_enabled = 1;
    runtime[AXIS_Y].config.level3.positive_limit = 300;
    runtime[AXIS_Y].config.level3.negative_limit = 200;
    runtime[AXIS_Z].config.level3.is_enabled = 1;
    runtime[AXIS_Z].config.level3.positive_limit = 300;
    runtime[AXIS_Z].config.level3.negative_limit = 200;

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
uint8_t Check_Axis_Limit_Violation(AxisLimitRuntime runtime)
{
    //    AxisLimitConfig cfg = runtime.config;
    float pos = runtime.current_position;
    uint8_t violation = 0;

    // 第一级检查：必须在区域内部运动
    if (runtime.config.level1.is_enabled && Is_Outside_Limit(pos, runtime.config.level1))
    {
        violation = 1; // 一级限位违规
        runtime.violation_flag |= 0x01;
        if(pos > runtime.config.level1.positive_limit)
        {
            runtime.violation_flag |= 0x08;
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 0);
            if(res == -8) return;
            LOG_OUTPUT(ERROR, "朝正方向移动超过了存储行程检查1");
        }
        else
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 1);
            if(res == -8) return;
            LOG_OUTPUT(ERROR, "朝负方向移动超过了存储行程检查1");
        }
    }

    // 第三级检查：必须在区域外部运动
    if (runtime.config.level3.is_enabled && Is_Inside_Limit(pos, runtime.config.level3))
    {
        violation = 1; // 三级限位违规
        runtime.violation_flag |= 0x04;
        if(pos >= runtime.config.level3.negative_limit)
        {
            runtime.violation_flag |= 0x20;
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 4);
            if(res == -8) return;
            LOG_OUTPUT(ERROR, "朝正方向移动超过了存储行程检查3");
        }
        else
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 5);
            if(res == -8) return;
            LOG_OUTPUT(ERROR, "朝负方向移动超过了存储行程检查3");
        }
    }

    // 第二级检查：根据模式判断
    if (runtime.config.level2.is_enabled)
    {
        if (runtime.config.level2_mode == 0)
        { // 内部检查模式
            if (Is_Outside_Limit(pos, runtime.config.level2))
            {
                violation = 1;
                runtime.violation_flag |= 0x02;
                if(pos > runtime.config.level2.positive_limit)
                {
                    runtime.violation_flag |= 0x18;
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 2);
                    if(res == -8) return;
                    LOG_OUTPUT(ERROR, "朝正方向移动超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 3);
                    if(res == -8) return;
                    LOG_OUTPUT(ERROR, "朝负方向移动超过了存储行程检查2");
                }
            }
        }
        else
        { // 外部检查模式
            if (Is_Inside_Limit(pos, runtime.config.level2))
            {
                violation = 1;
                runtime.violation_flag |= 0x02;
                if(pos >= runtime.config.level2.negative_limit)
                {
                    runtime.violation_flag |= 0x18;
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 2);
                    if(res == -8) return;
                    LOG_OUTPUT(ERROR, "朝正方向移动超过了存储行程检查2");
                }
                else
                {
                    int res = setAlarm(ALARM_AX, ALARM_ERROR, 3);
                    if(res == -8) return;
                    LOG_OUTPUT(ERROR, "朝负方向移动超过了存储行程检查2");
                }
            }
        }
    }
    return violation;
}

