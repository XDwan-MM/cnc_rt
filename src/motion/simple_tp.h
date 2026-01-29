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

// 停止判据：微小位置容差
#define TINY_DP(max_acc, period) (max_acc * period * period * 0.001)

#ifdef __cplusplus
extern "C" {
#endif

/* 简单的单轴轨迹规划器结构体 */
typedef struct simple_tp_t {
    double pos_cmd;     /* 位置命令（目标位置） */
    double max_vel;     /* 速度限制 */
    double max_acc;     /* 加速度限制 */
    int enable;         /* 如果为零，运动会尽快停止 */
    double curr_pos;    /* 当前位置 */
    double curr_vel;    /* 当前速度 */
    int active;         /* 如果运动进行中，则非零 */
} simple_tp_t;

/* 更新轨迹规划器
 * period: 伺服周期（秒）
 */

extern void simple_tp_update(simple_tp_t *tp, double period);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_TP_H */

