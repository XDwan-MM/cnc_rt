#include "spl_vel_planner.h"
#include <math.h>
#include <stddef.h>

/* 速度归零阈值，用于消除浮点计算产生的微小残余速度。 */
#define SPINDLE_VELOCITY_EPSILON 1e-9

/*
 * 按一个控制周期执行一次速度规划。
 *
 * 规划器只做速度斜坡，不涉及位置积分：每个周期最多改变
 * max_acc * period 的速度，并在接近目标时直接落到目标值，避免
 * 在目标速度附近来回振荡。enable 为 false 时，目标速度强制为零，
 * 因此同一个函数也可以用于主轴减速停止。
 */
bool spl_vel_planner_update(spl_vel_planner_t *planner, double period)
{
    double target_vel;
    double max_delta;
    double delta;
    bool active;

    /* 无效参数不能进行规划，调用方保留上一周期的输出。 */
    if (planner == NULL || period <= 0.0)
    {
        return false;
    }

    /* 未使能时忽略输入目标，沿加速度限制减速到零。 */
    target_vel = planner->enable ? planner->target_vel : 0.0;
    /* 当前周期允许的最大速度变化量。 */
    max_delta = fabs(planner->max_acc) * period;
    /* 带方向的速度误差决定加速还是减速。 */
    delta = target_vel - planner->current_vel;

    /* 到达目标或没有有效加速度参数时，直接设置目标速度。 */
    if (max_delta <= 0.0 || fabs(delta) <= max_delta)
    {
        planner->current_vel = target_vel;
    }
    else
    {
        /* 目标速度更大时加速，目标速度更小时减速。 */
        if (delta > 0.0)
        {
            planner->current_vel += max_delta;
        }
        else
        {
            planner->current_vel -= max_delta;
        }
    }

    /* 停止后强制归零，防止后续周期继续传播浮点残差。 */
    if (fabs(planner->current_vel) <= SPINDLE_VELOCITY_EPSILON)
    {
        planner->current_vel = 0.0;
    }

    /* active 表示当前仍有速度输出；停止状态下为 false。 */
    active = (fabs(planner->current_vel) > SPINDLE_VELOCITY_EPSILON);

    planner->active = active;
    
    /* 返回本周期更新后是否已经到达有效目标速度。 */
    return fabs(target_vel - planner->current_vel) <= SPINDLE_VELOCITY_EPSILON;
}
