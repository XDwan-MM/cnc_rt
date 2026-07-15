#include "jog_planner.h"
#include <math.h>
#include <stddef.h>

void jog_planner_update(jog_planner_t *tp, double period)
{
    if (tp == NULL || period <= 0.0)
        return;

    // 计算当前周期允许的最大速度变化量 (delta V)
    double max_dv = tp->max_acc * period;

    // 1. 确定当前瞬时目标速度 (使能断开时，目标速度强制为0)
    // 如果 enable 为 0，则目标速度为 0 (触发线性减速)
    double target_v = tp->enable ? tp->target_vel : 0.0;

    // 2. 速度斜坡控制 (Velocity Ramp Control)
    if (tp->curr_vel < target_v)
    {
        tp->curr_vel += max_dv;
        if (tp->curr_vel > target_v)
            tp->curr_vel = target_v;
    }
    else if (tp->curr_vel > target_v)
    {
        tp->curr_vel -= max_dv;
        if (tp->curr_vel < target_v)
            tp->curr_vel = target_v;
    }

    // 3. 更新运动状态指示 (使用工业通用阈值消除浮点数微小抖动)
    if (fabs(tp->curr_vel) > 1e-9)
    {
        tp->active = 1;
    }
    else
    {
        tp->curr_vel = 0.0; // 强制归零，防止漂移
        tp->active = 0;
    }

    // 4. 位置积分：位移 = 原位置 + 速度 * 时间
    // 纯速度模式下的位置积分：s = s + v * t
    tp->curr_pos += tp->curr_vel * period;
}