#include "egear_ratio.h"
#include <stddef.h>

// 将机械坐标位置转换为驱动器目标脉冲数。
int32_t position_to_pulses(const RT_AXIS* rt_axis, double target_pos_mm)
{
    // 验证参数有效性
    if (rt_axis == NULL || rt_axis->pm_munit == 0)
    {
        return 0;
    }

    // 预计算pulse_per_mm，提高运行效率
    const double pulse_per_mm = (double)rt_axis->pm_pulse /
                                ((double)rt_axis->pm_munit);

    // 编码器反馈偏置量
    double enc_fb_offset = rt_axis->enc_off; // 编码器反馈偏置[unit: mm]

    // 下发给驱动器的位置指令（MCS下的目标位置 + 偏置量）
    double motor_pos_mm = target_pos_mm + enc_fb_offset;

    // 转换为理想脉冲数
    double ideal_pulses = motor_pos_mm * pulse_per_mm;

    // 四舍五入至最接近整数脉冲
    if (ideal_pulses >= 0)
    {
        return (int32_t)(ideal_pulses + 0.5);
    }
    else
    {
        return (int32_t)(ideal_pulses - 0.5);
    }
}

// 将驱动器反馈脉冲数转换为机械坐标位置。
double pulses_to_position(const RT_AXIS* rt_axis, int32_t feedback_pulses)
{
    // 验证参数有效性
    if (rt_axis == NULL || rt_axis->pm_munit == 0)
    {
        return 0.0;
    }

    // 预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)rt_axis->pm_munit) /
                                (double)rt_axis->pm_pulse;

    // 电机端绝对物理位移
    double motor_pos_mm = ((double)feedback_pulses) * mm_per_pulse;

    // 编码器反馈偏置量
    double enc_fb_offset = rt_axis->enc_off; // 编码器反馈偏置[unit: mm]

    // 机械坐标位置MCS
    return (motor_pos_mm - enc_fb_offset);
}

// 计算编码器反馈偏置量
double calcul_enc_off(const RT_AXIS* rt_axis, int32_t raw_encoder_pulses, double target_pos_mm)
{
    if (rt_axis == NULL || rt_axis->pm_munit == 0)
    {
        return 0.0;
    }

    // 预计算每脉冲位移量（mm/pulse），提高运行效率
    const double mm_per_pulse = ((double)rt_axis->pm_munit) /
                                (double)rt_axis->pm_pulse;

    // 计算当前编码器原始脉冲对应的电机未补偿物理位置 (mm)
    double current_raw_pos_mm = (double)raw_encoder_pulses * mm_per_pulse;

    // 编码器偏置量 = 电机原始物理位置 - 当前期望的机床坐标
    double calculated_enc_offset_mm = current_raw_pos_mm - target_pos_mm;

    // 返回
    return calculated_enc_offset_mm;
}