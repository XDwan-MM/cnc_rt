/**
 * @file        egear_ratio.h
 * @brief       1、数控系统电子齿轮比功能
 * @version     V1.00
 * @date        2026/02/05
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 */

#ifndef EGEAR_RATIO_H
#define EGEAR_RATIO_H

#include <stdlib.h>
#include "rt_par.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 脉冲转位置值
 * * 将伺服反馈的脉冲数根据电子齿轮比转换为物理位置值。
 * @param pulses_fb 传入的反馈脉冲数 (单位: pulse)
 * @param axis      轴参数指针，函数内部将使用：
 * - axis->pm_munit: 电子齿轮比分子，实时统一单位化 (单位: mm)。约分前就是丝杆导程
 * - axis->pm_pulse: 电子齿轮比分母，实时统一单位化 (单位: 脉冲)。约分前驱动器每转脉冲数
 * @return double   转换后的物理位置值 (单位为 mm)
 */
extern double pulses_to_position(int32_t pulses_fb, const PAR_AXIS *axis);

/**
 * @brief 位置值转脉冲
 * * 将指令的物理位置值转换为伺服需要的脉冲数。
 * @param postion   输入的物理位置值
 * @param axis      轴参数指针，函数内部将使用：
 * - axis->pm_munit: 电子齿轮比分子，实时统一单位化 (单位: mm)。约分前就是丝杆导程
 * - axis->pm_pulse: 电子齿轮比分母，实时统一单位化 (单位: 脉冲)。约分前驱动器每转脉冲数
 * @return int32_t  转换后的目标脉冲数 (单位: pulse)
 */
extern int32_t position_to_pulses(double postion, const PAR_AXIS *axis);

#ifdef __cplusplus
}
#endif

#endif // EGEAR_RATIO_H
