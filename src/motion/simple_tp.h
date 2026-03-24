/********************************************************************
 * @file        simple_tp.h
 * @brief       1、简单轨迹规划器
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef SIMPLE_TP_H
#define SIMPLE_TP_H

#include <stdbool.h>

#define TINY_DP(max_acc, period) (max_acc * period * period * 0.001)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct simple_tp_t 
{
    double pos_cmd;     /* 目标位置（JOG模式下通常设为远端限位） */
    double max_vel;     /* 当前速度限制（已包含倍率） */
    double max_acc;     /* 加速度限制 */
    int enable;         /* 使能信号：1-运动，0-减速停止 */
    double curr_pos;    /* 当前位置反馈 */
    double curr_vel;    /* 当前实时速度 */
    int active;         /* 运动指示：非零表示正在运动或减速中 */
} simple_tp_t;

/* 更新轨迹规划器：由 1ms 伺服中断调用 */
extern void simple_tp_update(simple_tp_t *tp, double period);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_TP_H */

