#include "mpg_planner.h"
#include "global.h"
#include <math.h>

/* 内部函数声明 */
static bool mpg_planner_axis_is_valid(AXIS_TYPE axis);
static double mpg_planner_limit_value(double value);

/* 外部函数定义 */

void mpg_planner_update_target(mpg_planner_t *planner)
{
    int32_t count_delta;
    double distance;
    double target;
    double stop_distance;
    double velocity_limit;
    double acceleration_limit;

    if (planner == NULL || rt_data == NULL || !mpg_planner_axis_is_valid(planner->axis))
        return;

    /* 无符号减法按模 2^32 计算，再解释为有符号增量，以兼容硬件计数器回绕。 */
    count_delta = (int32_t)(planner->count - planner->previous_count);
    planner->previous_count = planner->count;

    /* 首周期只建立计数基准，避免选择轴/倍率前累积的计数变成位置跳变。 */
    if (planner->first_cycle)
    {
        planner->first_cycle = false;
        return;
    }
    if (count_delta == 0)
        return;

    /* 正向计数遇正限位、负向计数遇负限位时，丢弃本次输入；反方向仍可移动。 */
    if ((count_delta > 0 && rt_data->axis_data[planner->axis].status.is_fwd_limit) ||
        (count_delta < 0 && rt_data->axis_data[planner->axis].status.is_rev_limit))
        return;

    /* 原实现每 4 个计数形成 1 个倍率单位，位置增量再乘 0.001。 */
    distance = ((double)count_delta / 4.0) * (double)planner->multiplier * 0.001;
    target = planner->target_position + distance;
    velocity_limit = mpg_planner_limit_value(planner->max_velocity);
    acceleration_limit = mpg_planner_limit_value(planner->max_acceleration);

    /* 用 v^2/(2a) 限制命令位置超前量；无效/零加速度时不做除法。 */
    if (acceleration_limit > 0.0)
    {
        stop_distance = velocity_limit * velocity_limit / (2.0 * acceleration_limit);
        if (target > planner->reference_position + stop_distance)
            target = planner->reference_position + stop_distance;
        else if (target < planner->reference_position - stop_distance)
            target = planner->reference_position - stop_distance;
    }
    planner->target_position = target;
}

void mpg_planner_step(mpg_planner_t *planner, double period)
{
    double max_delta_velocity;
    double position_error;
    double requested_velocity;
    double acceleration;
    double velocity_limit;

    if (planner == NULL || !mpg_planner_axis_is_valid(planner->axis))
        return;
    if (!isfinite(period) || period <= 0.0)
    {
        planner->current_velocity = 0.0;
        planner->active = false;
        return;
    }

    acceleration = mpg_planner_limit_value(planner->max_acceleration);
    velocity_limit = mpg_planner_limit_value(planner->max_velocity);
    if (acceleration <= 0.0 || !isfinite(planner->current_position) ||
        !isfinite(planner->current_velocity) || !isfinite(planner->target_position))
    {
        planner->current_velocity = 0.0;
        planner->active = false;
        return;
    }

    /* 每周期的速度变化上限构成加速度斜坡；平方根项按剩余位置误差请求速度。 */
    max_delta_velocity = acceleration * period;
    position_error = planner->target_position - planner->current_position;
    if (position_error > 1.0e-8)
        requested_velocity = -max_delta_velocity +
                             sqrt(2.0 * acceleration * position_error +
                                  max_delta_velocity * max_delta_velocity);
    else if (position_error < -1.0e-8)
        requested_velocity = max_delta_velocity -
                             sqrt(-2.0 * acceleration * position_error +
                                  max_delta_velocity * max_delta_velocity);
    else
    {
        if (rt_data != NULL &&
            (rt_data->axis_data[planner->axis].status.is_fwd_limit ||
             rt_data->axis_data[planner->axis].status.is_rev_limit))
            rt_data->axis_data[planner->axis].status.is_decel_done = 1;
        requested_velocity = 0.0;
    }

    if (!isfinite(requested_velocity))
        requested_velocity = 0.0;
    if (requested_velocity > velocity_limit)
        requested_velocity = velocity_limit;
    else if (requested_velocity < -velocity_limit)
        requested_velocity = -velocity_limit;

    /* 将当前速度按每周期最大变化量靠近请求速度，减速信号解除时也平滑恢复。 */
    if (requested_velocity > planner->current_velocity + max_delta_velocity)
        planner->current_velocity += max_delta_velocity;
    else if (requested_velocity < planner->current_velocity - max_delta_velocity)
        planner->current_velocity -= max_delta_velocity;
    else
        planner->current_velocity = requested_velocity;

    /* 速度积分得到规划位置；保留原周期积分形式。 */
    planner->current_position += planner->current_velocity * period;
    if (!isfinite(planner->current_position) || !isfinite(planner->current_velocity))
    {
        planner->current_position = planner->reference_position;
        planner->current_velocity = 0.0;
    }
    planner->active = (fabs(planner->current_velocity) > 1.0e-9);
}

/* 内部函数定义 */

static bool mpg_planner_axis_is_valid(AXIS_TYPE axis)
{
    if (axis >= AXIS_X && axis <= AXIS_Z)
        return true;
    return false;
}

static double mpg_planner_limit_value(double value)
{
    if (!isfinite(value) || value < 0.0)
        return 0.0;
    return value;
}
