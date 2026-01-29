/********************************************************************
 * @file        egear_ratio.c
 * @brief       1、数控系统电子齿轮比功能
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include <stdlib.h>
#include "egear_ratio.h"
#include <stddef.h>

static double convert_pulses_to_position(int32_t feedback_pulses, const PAR_AXIS* axis_para);
static double convert_pulses_to_position_with_offset(int32_t feedback_pulses, const PAR_AXIS* axis_para);

static int32_t convert_position_to_pulses(double postion, const PAR_AXIS* axis_para);
static int32_t convert_position_to_pulses_with_offset(double postion, const PAR_AXIS* axis_para);

/*
 * @brief 脉冲转位置值
* axis传入的pm_munit电子齿轮比分子单位为: um
* axis传入的pm_pulse电子齿轮比分母单位为: 脉冲
*/
double pulses_to_position(int32_t pulses_fb, const PAR_AXIS *axis)
{
    // 验证参数有效性
    if (axis == NULL || axis->pm_pulse  == 0)
    {
        return 0.0;
    }
    double pos = 0.0;
    if(axis->is_set_home_offset)
    {
        pos = convert_pulses_to_position(pulses_fb, axis);
    }
    else
    {
        pos = convert_pulses_to_position_with_offset(pulses_fb, axis);
    }
//    printf("pos: %f\n", pos);
    return pos;
}

/*
 * @brief 位置值转脉冲
* axis传入的pm_munit电子齿轮比分子单位为: um
* axis传入的pm_pulse电子齿轮比分母单位为: 脉冲
*/
int32_t position_to_pulses(double postion, const PAR_AXIS *axis)
{
    // 验证参数有效性
    if (axis == NULL || axis->pm_munit  == 0)
    {
        return 0;
    }
    int32_t pulses = 0;
    if(axis->is_set_home_offset)
    {
        pulses = convert_position_to_pulses(postion, axis);
    }
    else
    {
        pulses = convert_position_to_pulses_with_offset(postion, axis);
    }
//    printf("pulses: %f\n", pulses);
    return pulses;
}

// 脉冲位置转机械坐标值（无编码器反馈偏置）
static double convert_pulses_to_position(int32_t feedback_pulses, const PAR_AXIS *axis_para) {

    double pos_convert;
    // pm_munit电子齿轮比分子[每转位移量um] 约分前就是导程
    // pm_pulse电子齿轮比分母[每转脉冲数]   约分前驱动器每转脉冲数
    // 1.预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)axis_para->pm_munit * 1e-3) / (double)axis_para->pm_pulse;

    // 2.转换为机械位置（先转double避免溢出）
    pos_convert = (double)feedback_pulses * mm_per_pulse;

    // 3.返回
    return pos_convert;
}
// 脉冲位置转机械坐标值（带编码器反馈偏置）
static double convert_pulses_to_position_with_offset(int32_t feedback_pulses, const PAR_AXIS* axis_para) {
    double pos_convert;
    double offset = axis_para->enc_off;             // 编码器反馈偏置[unit: mm]

    // pm_munit电子齿轮比分子[每转位移量um] 约分前就是导程
    // pm_pulse电子齿轮比分母[每转脉冲数]   约分前驱动器每转脉冲数
    // 1.预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)axis_para->pm_munit * 1e-3) / (double)axis_para->pm_pulse;

    // 2.转换为机械位置（先转double避免溢出）
    pos_convert = ((double)feedback_pulses) * mm_per_pulse;

    // printf("bbbbb: %f %f %f\n", offset, pos_convert, pos_convert - offset);

    // 3.应用编码器反馈偏置（零点偏移）
    return (pos_convert - offset);
}


// 机械坐标值转脉冲位置（无编码器反馈偏置）
static int32_t convert_position_to_pulses(double postion, const PAR_AXIS* axis_para)
{
    double pulses_convert; 
    // pm_munit电子齿轮比分子[每转位移量um] 约分前就是导程
    // pm_pulse电子齿轮比分母[每转脉冲数]   约分前驱动器每转脉冲数
    // 1.预计算pulse_per_mm，提高运行效率
    const double pulse_per_mm = (double)axis_para->pm_pulse / ((double)axis_para->pm_munit * 1e-3);

    pulses_convert = postion * pulse_per_mm;

    // 四舍五入至最接近整数脉冲
    if (pulses_convert >= 0) {
        return (int32_t)(pulses_convert + 0.5);
    } else {
        return (int32_t)(pulses_convert - 0.5);
    }
}

// 机械坐标值转脉冲位置（带编码器反馈偏置）
static int32_t convert_position_to_pulses_with_offset(double postion, const PAR_AXIS* axis_para)
{
    double pulses_convert;
    double offset = axis_para->enc_off; // 偏置量

    // pm_munit电子齿轮比分子[每转位移量um] 约分前就是导程
    // pm_pulse电子齿轮比分母[每转脉冲数]   约分前驱动器每转脉冲数
    // 1.预计算pulse_per_mm，提高运行效率

    const double pulse_per_mm = (double)axis_para->pm_pulse / ((double)axis_para->pm_munit * 1e-3);

    pulses_convert = (postion + offset) * pulse_per_mm;

    // 四舍五入至最接近整数脉冲
    if (pulses_convert >= 0) {
        return (int32_t)(pulses_convert + 0.5);
    } else {
        return (int32_t)(pulses_convert - 0.5);
    }
}
