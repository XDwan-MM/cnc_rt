#include "interpolation.h"
#include "global.h"
#include <math.h>

int pmErrno = 0;

int tcLine(const COORD_TP *const tp, double length , POS *const point)
{
    int r1 = 0, r2 = 0;
    r1 = pmCartScalMult(&tp->plan_block->end_vec, length, point);
    r2 = pmCartCartAdd(&tp->plan_block->start_pos, point, point);
    return pmErrno = (r1 || r2) ? PM_NORM_ERR : 0;
}
int pmCartScalMult(const POS *const v1, double d, POS *const vout)
{
    if(v1 != vout)
    {
        *vout = *v1;
    }
    return pmCartScalMultEq(vout, d);
}
int pmCartScalMultEq(POS *const v, double d)
{
    v->x *= d;
    v->y *= d;
    v->z *= d;
    return pmErrno = 0;
}

int pmCartCartAdd(const POS *const v1, const POS *const v2, POS *const vout)
{
    vout->x = v1->x + v2->x;
    vout->y = v1->y + v2->y;
    vout->z = v1->z + v2->z;
    return pmErrno = 0;
}
int tcCircle(const COORD_TP *const circle, double progress, POS *pos)
{
    double r = 0.0;
    double delta_angle;
    double orgin_angle = 0.0;
    double end_angle = 0.0;
    double dot = 0.0;
    POS XYZ;
    if(circle->plan_block->circular_block.plane == PLANE_XY)
    {
        r = pmSqrt(pmSq(circle->plan_block->start_pos.x - circle->plan_block->circular_block.center_pos.x) + pmSq(circle->plan_block->start_pos.y - circle->plan_block->circular_block.center_pos.y));
    }
    else if(circle->plan_block->circular_block.plane == PLANE_XZ)
    {
        r = pmSqrt(pmSq(circle->plan_block->start_pos.x - circle->plan_block->circular_block.center_pos.x) + pmSq(circle->plan_block->start_pos.z - circle->plan_block->circular_block.center_pos.z));
    }
    else
    {
        r = pmSqrt(pmSq(circle->plan_block->start_pos.y - circle->plan_block->circular_block.center_pos.y) + pmSq(circle->plan_block->start_pos.z - circle->plan_block->circular_block.center_pos.z));
    }
    if(r > 0.0)
    {
        delta_angle = progress / r;
        if(circle->plan_block->circular_block.plane == PLANE_XY)
        {
            orgin_angle = acos(saturate((circle->plan_block->start_pos.x - circle->plan_block->circular_block.center_pos.x) / r,1.0));
            dot = circle->plan_block->start_pos.y - circle->plan_block->circular_block.center_pos.y;
            if(dot < 0.0)
            {
                orgin_angle = 2 * PM_PI - orgin_angle;
            }
            if(circle->plan_block->type == CIRCLE_CCW)
            {
                end_angle = orgin_angle - delta_angle;
            }
            else
            {
                end_angle = orgin_angle + delta_angle;
            }
            XYZ.x = circle->plan_block->circular_block.center_pos.x + r * cos(end_angle);
            XYZ.y = circle->plan_block->circular_block.center_pos.y + r * sin(end_angle);
            XYZ.z = circle->plan_block->end_pos.z;
        }
        else if(circle->plan_block->circular_block.plane == PLANE_XZ)
        {
            orgin_angle = acos(saturate((circle->plan_block->start_pos.z - circle->plan_block->circular_block.center_pos.z) / r,1.0));
            dot = circle->plan_block->start_pos.x - circle->plan_block->circular_block.center_pos.x;
            if(dot < 0.0)
            {
                orgin_angle = 2 * PM_PI - orgin_angle;
            }
            if(circle->plan_block->type == CIRCLE_CCW)
            {
                end_angle = orgin_angle - delta_angle;
            }
            else
            {
                end_angle = orgin_angle + delta_angle;
            }
            XYZ.z = circle->plan_block->circular_block.center_pos.z + r * cos(end_angle);
            XYZ.x = circle->plan_block->circular_block.center_pos.x + r * sin(end_angle);
            XYZ.y = circle->plan_block->end_pos.y;
        }
        else
        {
            orgin_angle = acos(saturate((circle->plan_block->start_pos.y - circle->plan_block->circular_block.center_pos.y) / r,1.0));
            dot = circle->plan_block->start_pos.z - circle->plan_block->circular_block.center_pos.z;
            if(dot < 0.0)
            {
                orgin_angle = 2 * PM_PI - orgin_angle;
            }
            if(circle->plan_block->type == CIRCLE_CCW)
            {
                end_angle = orgin_angle - delta_angle;
            }
            else
            {
                end_angle = orgin_angle + delta_angle;
            }
            XYZ.y = circle->plan_block->circular_block.center_pos.y + r * cos(end_angle);
            XYZ.x = circle->plan_block->end_pos.x;
            XYZ.z = circle->plan_block->circular_block.center_pos.z + r * sin(end_angle);
        }
    }
    else
    {
        XYZ = circle->plan_block->end_pos;
    }
    *pos = XYZ;
    return 1;
}
double saturate(double x, double max)
{
    if(x > max)
    {
        return max;
    }
    else if(x < (-max))
    {
        return -max;
    }
    else
    {
        return x;
    }
}
double pmSqrt(double x)
{
    if(x > 0.0)
    {
        return sqrt(x);
    }
    return 0.0;
}

void interpolationExec(double interp_pin)
{
    if(interp_pin <= 0.0) return;

    POS pos = {0};
    switch (cycle_tp->plan_block->type) {
    case RAPID:
    case LINE:
        tcLine(cycle_tp, interp_pin, &pos);
        break;
    case CIRCLE_CCW:
        tcCircle(cycle_tp, interp_pin, &pos);
        break;
    case CIRCLE_CW:
        tcCircle(cycle_tp, interp_pin, &pos);
        break;
    case HELIX:

        break;
    case SPLINE:

        break;
    case RIGID_TAP:

        break;
    case PROBE:

        break;
    case UNKNOWN:
        return;
    default:
        return;


    }
    var_axis[AXIS_X][VAR_AXIS_CMD_POS_EX].value.f = pos.x;
    var_axis[AXIS_Y][VAR_AXIS_CMD_POS_EX].value.f = pos.y;
    var_axis[AXIS_Z][VAR_AXIS_CMD_POS_EX].value.f = pos.z;
   // printf("pos: %f %f %f\n", pos.x, pos.y, pos.z);
}

