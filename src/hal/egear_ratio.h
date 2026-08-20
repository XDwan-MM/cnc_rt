/**
 * @file        egear_ratio.h
 * @brief       数控系统电子齿轮比转换功能
 * @date        2026/08/06
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef EGEAR_RATIO_H
#define EGEAR_RATIO_H

#include "pardata.h"
#include "rt_data.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 将机械坐标目标位置转换为驱动器目标脉冲数。
 *
 * 根据电子齿轮比分子 PAR_AX_PM_MUNIT 和分母 PAR_AX_PM_PULSE 计算脉冲数。转换前将
 * PAR_AX_ENC_OFF 编码器反馈偏置加入目标位置，结果四舍五入到最近的整数脉冲。
 *
 * @param axis_para 轴参数数组，包含每转位移量、每转指令脉冲数和编码器反馈偏置。
 * @param target_pos_mm 机械坐标系下的目标位置，单位：mm。
 * @return 驱动器目标脉冲数；axis_para 为空或电子齿轮比分子为 0 时返回 0。
 */
extern int32_t position_to_pulses(const RT_AXIS* rt_axis, double target_pos_mm);

/**
 * @brief 将驱动器反馈脉冲数转换为机械坐标位置。
 *
 * 根据电子齿轮比分子 PAR_AX_PM_MUNIT 和分母 PAR_AX_PM_PULSE 计算电机端位置，再减去
 * PAR_AX_ENC_OFF 编码器反馈偏置，得到机械坐标系下的位置。
 *
 * @param axis_para 轴参数数组，包含每转位移量、每转指令脉冲数和编码器反馈偏置。
 * @param feedback_pulses 驱动器反馈脉冲数，单位：pulse。
 * @return 机械坐标系下的位置，单位：mm；axis_para 为空或电子齿轮比分母为 0 时返回 0.0。
 */
extern double pulses_to_position(const RT_AXIS* rt_axis, int32_t feedback_pulses);

/**
 * @brief 根据编码器原始脉冲和目标机械位置计算反馈偏置。
 *
 * 将编码器原始脉冲换算为未补偿物理位置，再减去目标机械坐标位置。
 *
 * @param axis_para 轴参数数组，包含每转位移量和每转指令脉冲数。
 * @param raw_encoder_pulses 编码器原始反馈脉冲数，单位：pulse。
 * @param target_pos_mm 目标机械坐标位置，单位：mm。
 * @return 编码器反馈偏置，单位：mm；axis_para 为空或电子齿轮比分母为 0 时返回 0.0。
 */
extern double calcul_enc_off(
    const RT_AXIS* rt_axis,
    int32_t raw_encoder_pulses,
    double target_pos_mm
);

#ifdef __cplusplus
}
#endif

#endif // EGEAR_RATIO_H
