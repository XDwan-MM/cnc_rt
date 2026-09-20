#include "spl_pos_planner.h"
#include <math.h>
#include <stddef.h>

#define POSITION_PLANNER_EPSILON 1e-9

bool spl_pos_planner_update(spl_pos_planner_t *planner, double period)
{
    double distance;
    double direction;
    double max_acc;
    double desired_speed;
    double desired_vel;
    double max_delta_vel;

    if (planner == NULL || period <= 0.0)
    {
        return false;
    }

    distance = planner->target_pos - planner->current_pos;
    max_acc = fabs(planner->max_acc);
    if (fabs(distance) <= POSITION_PLANNER_EPSILON)
    {
        planner->current_pos = planner->target_pos;
        planner->current_vel = 0.0;
        planner->active = false;
        return true;
    }

    if (max_acc <= 0.0 || fabs(planner->max_vel) <= 0.0)
    {
        planner->current_pos = planner->target_pos;
        planner->current_vel = 0.0;
        planner->active = false;
        return true;
    }

    direction = distance > 0.0 ? 1.0 : -1.0;
    /* v=sqrt(2*a*s) 保证到达目标前可以减速到零。 */
    desired_speed = sqrt(2.0 * max_acc * fabs(distance));
    if (desired_speed > fabs(planner->max_vel))
    {
        desired_speed = fabs(planner->max_vel);
    }
    desired_vel = direction * desired_speed;
    max_delta_vel = max_acc * period;
    if (desired_vel > planner->current_vel + max_delta_vel)
    {
        planner->current_vel += max_delta_vel;
    }
    else if (desired_vel < planner->current_vel - max_delta_vel)
    {
        planner->current_vel -= max_delta_vel;
    }
    else
    {
        planner->current_vel = desired_vel;
    }

    planner->current_pos += planner->current_vel * period;
    if ((direction > 0.0 && planner->current_pos >= planner->target_pos) ||
        (direction < 0.0 && planner->current_pos <= planner->target_pos))
    {
        planner->current_pos = planner->target_pos;
        planner->current_vel = 0.0;
        planner->active = false;
        return true;
    }
    planner->active = true;
    return false;
}
