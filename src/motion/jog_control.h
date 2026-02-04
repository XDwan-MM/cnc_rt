/********************************************************************
 * @file        jog_control.h
 * @brief       JOG控制模块
 * @version     V1.00
 * @date        2026/02/04
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/
#ifndef JOG_CONTROL_H
#define JOG_CONTROL_H

#include "rt_par.h"
#include "simple_tp.h"

/* JOG方向定义 */
#define JOG_DIR_NONE    0
#define JOG_DIR_POSITIVE 1
#define JOG_DIR_NEGATIVE -1

#ifdef __cplusplus
extern "C" {
#endif

/* 启动连续JOG
 * axis: 轴指针
 * direction: JOG方向（1=正向, -1=负向）
 * axis_para: 实时这边 速度的单位传入的是: mm/s
 *
 */
void start_jog_cont(const PAR_AXIS* axis_para, simple_tp_t *jog_tp, int direction, const double feed_override, const double run_period);
void start_jog_rapid_cont(const PAR_AXIS* axis_para, simple_tp_t *rapid_jog_tp, int direction, const double feed_override, const double run_period);

/* 停止JOG */
void stop_jog_cont(simple_tp_t *jog_tp, const double run_period, bool immediate);

#ifdef __cplusplus
}
#endif

#endif /* JOG_CONTROL_H */

