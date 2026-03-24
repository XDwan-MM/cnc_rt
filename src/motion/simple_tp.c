/********************************************************************
 * @file        simple_tp.h
 * @brief       1、简单轨迹规划器
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "simple_tp.h"
#include <stddef.h>
#include <math.h>

void simple_tp_update(simple_tp_t *tp, double period)
{
    if (tp == NULL || period <= 0.0) return;

    // 1. 计算当前周期允许的最大速度变化量 (delta V)
    double max_dv = tp->max_acc * period;
    double target_vel = 0.0;

    // 2. 确定目标速度
    if (tp->enable) {
        // 计算方向：根据当前位置与目标位置（限位）的关系决定方向
        double pos_err = tp->pos_cmd - tp->curr_pos;
        
        if (fabs(pos_err) > TINY_DP(tp->max_acc, period)) {
            // JOG模式下，目标速度就是当前的限制速度（带倍率）
            target_vel = (pos_err > 0) ? tp->max_vel : -tp->max_vel;
        } else {
            target_vel = 0.0;
        }
    } else {
        // 使能断开，目标速度设为0，触发减速过程
        target_vel = 0.0;
    }

    // 3. 速度斜坡规划 (Ramp Control)
    // 让当前速度以 max_acc 为约束，线性追赶目标速度
    if (tp->curr_vel < target_vel) {
        tp->curr_vel += max_dv;
        if (tp->curr_vel > target_vel) tp->curr_vel = target_vel;
    } else if (tp->curr_vel > target_vel) {
        tp->curr_vel -= max_dv;
        if (tp->curr_vel < target_vel) tp->curr_vel = target_vel;
    }

    // 4. 更新运动状态指示
    // 只要速度没降到极小值，就认为 active 为 1
    if (fabs(tp->curr_vel) > 1e-9) {
        tp->active = 1;
    } else {
        tp->curr_vel = 0.0; // 彻底停稳
        tp->active = 0;
        // 停止后同步目标点，防止下次使能瞬间产生位置跳变
        if (!tp->enable) tp->pos_cmd = tp->curr_pos;
    }

    // 5. 位置积分：s = s + v * t
    tp->curr_pos += tp->curr_vel * period;
}

