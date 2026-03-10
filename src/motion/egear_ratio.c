#include "egear_ratio.h"

static double convert_pulses_to_position(int32_t feedback_pulses, const ParamUnit* axis_para);
static double convert_pulses_to_position_with_offset(int32_t feedback_pulses, const ParamUnit* axis_para);

static int32_t convert_position_to_pulses(double postion, const ParamUnit* axis_para);
static int32_t convert_position_to_pulses_with_offset(double postion, const ParamUnit* axis_para);


double pulses_to_position(int32_t pulses_fb, const ParamUnit *axis)
{
    // 验证参数有效性
    if (axis == NULL || axis[PAR_AX_PM_PULSE].value.val_int == 0)
    {
        return 0.0;
    }
    double pos = 0.0;
    if(axis[PAR_AX_IS_SET_HOME_OFFSET].value.val_bool)
    {
        pos = convert_pulses_to_position(pulses_fb, axis);
    }
    else
    {
        pos = convert_pulses_to_position_with_offset(pulses_fb, axis);
    }
    return pos;
}

int32_t position_to_pulses(double postion, const ParamUnit *axis)
{
    // 验证参数有效性
    if (axis == NULL || axis[PAR_AX_PM_MUNIT].value.val_int == 0)
    {
        return 0;
    }
    int32_t pulses = 0;
    if(axis[PAR_AX_IS_SET_HOME_OFFSET].value.val_bool)
    {
        pulses = convert_position_to_pulses(postion, axis);
    }
    else
    {
        pulses = convert_position_to_pulses_with_offset(postion, axis);
    }
    return pulses;
}

// 脉冲位置转机械坐标值（无编码器反馈偏置）
static double convert_pulses_to_position(int32_t feedback_pulses, const ParamUnit *axis_para)
{
    double pos_convert;

    // 1.预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)axis_para[PAR_AX_PM_MUNIT].value.val_int) / (double)axis_para[PAR_AX_PM_PULSE].value.val_int;

    // 2.转换为机械位置（先转double避免溢出）
    pos_convert = (double)feedback_pulses * mm_per_pulse;

    // 3.返回
    return pos_convert;
}
// 脉冲位置转机械坐标值（带编码器反馈偏置）
static double convert_pulses_to_position_with_offset(int32_t feedback_pulses, const ParamUnit* axis_para)
{
    double pos_convert;
    double offset = axis_para[PAR_AX_ENC_OFF].value.val_real; // 编码器反馈偏置[unit: mm]

    // 1.预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)axis_para[PAR_AX_PM_MUNIT].value.val_int) / (double)axis_para[PAR_AX_PM_PULSE].value.val_int;

    // 2.转换为机械位置（先转double避免溢出）
    pos_convert = ((double)feedback_pulses) * mm_per_pulse;

    // 3.应用编码器反馈偏置（零点偏移）
    return (pos_convert - offset);
}


// 机械坐标值转脉冲位置（无编码器反馈偏置）
static int32_t convert_position_to_pulses(double postion, const ParamUnit* axis_para)
{
    double pulses_convert; 

    // 1.预计算pulse_per_mm，提高运行效率
    const double pulse_per_mm = (double)axis_para[PAR_AX_PM_PULSE].value.val_int / ((double)axis_para[PAR_AX_PM_MUNIT].value.val_int);

    pulses_convert = postion * pulse_per_mm;

    // 四舍五入至最接近整数脉冲
    if (pulses_convert >= 0) {
        return (int32_t)(pulses_convert + 0.5);
    } else {
        return (int32_t)(pulses_convert - 0.5);
    }
}

// 机械坐标值转脉冲位置（带编码器反馈偏置）
static int32_t convert_position_to_pulses_with_offset(double postion, const ParamUnit* axis_para)
{
    double pulses_convert;
    double offset = axis_para[PAR_AX_ENC_OFF].value.val_real; // 编码器反馈偏置[unit: mm]

    // 1.预计算pulse_per_mm，提高运行效率
    const double pulse_per_mm = (double)axis_para[PAR_AX_PM_PULSE].value.val_int / ((double)axis_para[PAR_AX_PM_MUNIT].value.val_int);

    pulses_convert = (postion + offset) * pulse_per_mm;

    // 四舍五入至最接近整数脉冲
    if (pulses_convert >= 0) {
        return (int32_t)(pulses_convert + 0.5);
    } else {
        return (int32_t)(pulses_convert - 0.5);
    }
}
