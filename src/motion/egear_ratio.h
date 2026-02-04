/********************************************************************
 * @file        egear_ratio.h
 * @brief       1、数控系统电子齿轮比功能
 * @version     V1.00
 * @date        2026/01/12
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef EGEAR_RATIO_H
#define EGEAR_RATIO_H

#include <stdlib.h>
#include "rt_par.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gear_num; // 电子齿轮分子
    int gear_den; // 电子齿轮分母
} EGear_Config_t;

/*
* axis传入的pm_munit电子齿轮比分子单位为: um
* axis传入的pm_pulse电子齿轮比分母单位为: 脉冲
*/
extern double pulses_to_position(int32_t pulses_fb, const PAR_AXIS *axis);
extern int32_t position_to_pulses(double postion, const PAR_AXIS *axis);

#ifdef __cplusplus
}
#endif

#endif // EGEAR_RATIO_H
