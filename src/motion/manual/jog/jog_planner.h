/**
 * @file        jog_planner.h
 * @brief       JOG规划器（速度控制规划）
 * @date        2026/09/04
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef JOG_PLANNER_H
#define JOG_PLANNER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /* 输入参数 */
    double target_vel; /* 目标指令速度 (单位/s, 带方向, 包含倍率) */
    double max_acc;    /* 最大加速度 (单位/s^2) */
    int enable;        /* 运动使能：1-追踪速度, 0-减速停止 */

    /* 输出状态 */
    double curr_pos; /* 规划位置输出 (用于给伺服发送指令) */
    double curr_vel; /* 当前实际速度输出 */
    int active;      /* 运动指示：1-正在运动或减速中, 0-完全静止 */
} jog_planner_t;

/**
 * @brief JOG 规划器周期性更新函数
 * @param tp 规划器结构体指针
 * @param period 执行周期 (单位：s)
 */
extern void jog_planner_update(jog_planner_t *tp, double period);

#ifdef __cplusplus
}
#endif

#endif /* JOG_PLANNER_H */
