/********************************************************************
 * @file        jog_control.h
 * @brief       JOG控制模块
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/
#ifndef JOG_CONTROL_H
#define JOG_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rt_par.h"
#include "rt_var.h"


/* JOG方向定义 */
#define JOG_DIR_NONE    0
#define JOG_DIR_POSITIVE 1
#define JOG_DIR_NEGATIVE -1

/* 启动连续JOG
 * axis: 轴指针
 * direction: JOG方向（1=正向, -1=负向）
 * axis_para: 速度的单位传入的是: mm/min
 */
void start_jog_cont(VAR_AXIS *runing_axis, const PAR_AXIS* axis_para, int direction, double run_period, const double feed_override);
void start_jog_rapid_cont(VAR_AXIS *runing_axis, const PAR_AXIS *axis_para, int direction, double run_period, const double rapid_override);

/* 停止JOG */
void stop_jog_cont(VAR_AXIS *runing_axis, double run_period, bool immediate);

/* 检查JOG是否激活 */
//int jog_cont_is_active(VAR_AXIS *axis);

#ifdef __cplusplus
}
#endif

#endif /* JOG_CONTROL_H */

