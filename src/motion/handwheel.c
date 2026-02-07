#include "handwheel.h"
#include "log.h"
#include <math.h>

simple_tp_t_handwheel simple_tp[5];

//int32_t ratio = 10000;
int Get_Multiplier_Switch(int output)
{
    switch (output) {
    case HANDWHEEL_RATE_X1:
        return 1;
    case HANDWHEEL_RATE_X10:
        return 10;
    case HANDWHEEL_RATE_X100:
        return 100;
    default:
        return -1;
    }
}
int Get_Axis_Switch(int output)
{
    switch (output) {
    case HANDWHEEL_AXIS_OFF:
        return -1;
    case HANDWHEEL_AXIS_X:
        return 0;
    case HANDWHEEL_AXIS_Y:
        return 1;
    case HANDWHEEL_AXIS_Z:
        return 2;
    case HANDWHEEL_AXIS_4:
        return 3;
    case HANDWHEEL_AXIS_5:
        return 4;
    default:
        return -1;
    }
}
void Handlewheels_Motion(simple_tp_t_handwheel *tp)
{
    int new_hw_counts, delta;
    double distance, pos, stop_dist;
    /* 从点动手轮获取计数 */
    new_hw_counts = tp->joint.count;
    delta = new_hw_counts - tp->joint.old_counts;
    /* 保存值供下次使用 */
    tp->joint.old_counts = new_hw_counts;
    /* 初始化完成 */
    if (tp->first_pass)
    {
        tp->first_pass = 0;
        return;
    }
    if ( delta == 0 )
    {
        return;
    }
    /* 计算点动距离 */
    distance = (delta / 4.0) * tp->joint.rate * 0.001 ;
    /* 计算点动目标位置 */
    pos = tp->pos_cmd + distance;
    double v = tp->joint.vel_limit;
    double jaccel_limit = tp->joint.jaccel_limit;
    /* 计算最大速度下的停止距离 */
    stop_dist = v * v / ( 2 * jaccel_limit);
    /* 如果命令位置领先实际位置超过停止距离，丢弃多余的命令 */
    if ( pos > tp->joint.pos_cmd + stop_dist )
    {
        pos = tp->joint.pos_cmd + stop_dist;
    }
    else if ( pos < tp->joint.pos_cmd - stop_dist )
    {
        pos = tp->joint.pos_cmd - stop_dist;
    }
    /* 设置目标位置并使用最大速度和加速度 */
    tp->pos_cmd = pos;
    tp->max_vel = tp->joint.vel_limit;
    tp->max_acc = jaccel_limit;
}

void Simple_Tp_Update(simple_tp_t_handwheel *tp)
{
    double max_dv, tiny_dp, pos_err, vel_req;
    /* 计算每个伺服周期内允许的最大速度变化量 */
    max_dv = tp->max_acc * par_ncu.cycle;
    /* 计算一个极小的位置范围，在此范围内视为零 */
    tiny_dp = 0.00000001;
    pos_err = tp->pos_cmd - tp->curr_pos;
    /* 正负误差需要符号处理，以避免对负数开平方 */
    /* 计算期望速度 */
    if (pos_err > tiny_dp)
    {
        vel_req = -max_dv + sqrt(2.0 * tp->max_acc * pos_err + max_dv * max_dv);
    } else if (pos_err < -tiny_dp)
    {
        vel_req =  max_dv - sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
    } else
    {
        vel_req = 0.0;
    }
    /* 限制速度请求 */
    if (vel_req > tp->max_vel)
    {
        vel_req = tp->max_vel;
    }
    else if (vel_req < -tp->max_vel)
    {
        vel_req = -tp->max_vel;
    }
    /* 在加速度限制下，将当前速度向请求速度逼近 */
    if (vel_req > tp->curr_vel + max_dv)
    {
        tp->curr_vel += max_dv;
    }
    else if (vel_req < tp->curr_vel - max_dv)
    {
        tp->curr_vel -= max_dv;
    }
    else
    {
        tp->curr_vel = vel_req;
    }
    /* 对速度积分以获得新位置 */
    tp->curr_pos += tp->curr_vel * par_ncu.cycle;
}
