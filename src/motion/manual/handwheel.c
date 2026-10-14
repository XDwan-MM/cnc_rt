#include "handwheel.h"
#include <hmisysapi.h>
#include <math.h>

simple_tp_t_handwheel simple_tp[5];
static uint32_t axisandrate = 0;
static uint32_t current_count = 0; // 手轮计数器
static int rate = 0;               // 倍率
static int axis = 0;               // 轴选
static int valid_axis = -1;        // 最后一次有效轴选
static int valid_rate = -1;        // 最后一次有效倍率
static bool mpg_enable = false;    // 当前手轮使能状态
static bool mpg_initialized = false;

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

void handwheel_handle_estop()
{
    for (int i = 0; i < 5; i++)
    {
        memset(&simple_tp[i], 0, sizeof(simple_tp_t_handwheel));
    }
    valid_axis = -1;
    valid_rate = -1;
    mpg_enable = false;
    mpg_initialized = false;
}

double handwheel_get_axis_velocity(void)
{
    /*
     * 只查询当前轴选对应的规划器：换轴后旧轴的规划器不会再被更新，
     * 其残留速度不代表手轮当前还在输出运动。
     */
    if (!mpg_initialized || valid_axis < 0 || valid_axis >= 5)
        return 0.0;
    return simple_tp[valid_axis].curr_vel;
}

void handwheel_resync_position(void)
{
    /*
     * 清掉轴选/倍率的历史状态，使下一次 handle_mpg_cmd() 走“选择变化”分支：
     * 以实时数据中的 wcs_cmd_pos_cur 重建目标位置和规划位置、速度清零，
     * 并把 first_pass 置 1，从而丢弃未选择期间的累计计数。
     */
    valid_axis = -1;
    valid_rate = -1;
    mpg_enable = false;
    mpg_initialized = false;
}

void Handlewheels_Motion(simple_tp_t_handwheel *tp)
{
    uint32_t new_hw_counts;
    int32_t delta;
    double distance, pos, stop_dist;
    /* 从点动手轮获取计数 */
    new_hw_counts = tp->joint.count;
    /*
     * The hardware counter is unsigned and is allowed to wrap.  Converting
     * the modulo-2^32 difference to int32_t gives the expected signed step
     * for normal handwheel increments (including wrap-around).
     */
    delta = (int32_t)(new_hw_counts - tp->joint.old_counts);
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

    if((delta > 0 && rt_data->axis_data[tp->axis].status.is_fwd_limit) || 
        (delta < 0 && rt_data->axis_data[tp->axis].status.is_rev_limit))
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
    double ncu_cycle = rt_data->ncu_data.cycle;
    double max_dv, tiny_dp, pos_err, vel_req;
    /* 计算每个伺服周期内允许的最大速度变化量 */
    max_dv = tp->max_acc * ncu_cycle;
    /* 计算一个极小的位置范围，在此范围内视为零 */
    tiny_dp = 0.00000001;
    pos_err = tp->pos_cmd - tp->curr_pos;
    /* 正负误差需要符号处理，以避免对负数开平方 */
    /* 计算期望速度 */
    if (pos_err > tiny_dp)
    {
        vel_req = -max_dv + sqrt(2.0 * tp->max_acc * pos_err + max_dv * max_dv);
        // rt_data->axis_data[tp->axis].status.is_decel_done = 0;

    } 
    else if (pos_err < -tiny_dp)
    {
        vel_req =  max_dv - sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
    } 
    else
    {
        if(rt_data->axis_data[tp->axis].status.is_fwd_limit || rt_data->axis_data[tp->axis].status.is_rev_limit)
        {
            rt_data->axis_data[tp->axis].status.is_decel_done = 1;
        }
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
    tp->curr_pos += tp->curr_vel * ncu_cycle;
}

/*
 * 机械锁住请求与实际生效状态不一致：正在锁住或正在解除。
 * 与 auto_gcode.c 中自动模式的处理保持一致——两者不一致期间不接收新的手轮位移，
 * 先把轴停下来，等停稳后 emcmot 才切换生效状态。
 */
static bool handwheel_is_mac_lock_switching(void)
{
    return rt_data->sys_state[SYS_IS_MAC_LOCK_ON] != rt_data->sys_state[SYS_IS_MAC_LOCK_READY];
}

/* 锁住状态切换期间停止追赶已有目标：目标位置对齐当前规划位置，由规划器按斜坡停稳。 */
static void handwheel_stop_chasing(simple_tp_t_handwheel *tp)
{
    tp->pos_cmd = tp->curr_pos;
}

void handle_mpg_cmd() {
    bool valid_selection;
    bool selection_changed;
    bool lock_switching;
    int previous_axis;

    // 急停不执行
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE]) return;
    lock_switching = handwheel_is_mac_lock_switching();
    HW_Get_Count(&current_count);
    HW_Get_Axis_And_Rate(&axisandrate);
    rate = axisandrate & 0b111;
    rate = Get_Multiplier_Switch(rate);
    axis = (axisandrate >> 3) & 0b11111;
    axis = Get_Axis_Switch(axis);

    valid_selection = (axis >= 0 && axis < 5 && rate > 0);
    if (!valid_selection)
    {
        mpg_enable = false;
        if (mpg_initialized && valid_axis >= 0 && valid_axis < 5)
        {
            simple_tp[valid_axis].joint.old_counts = current_count;
            if (lock_switching)
                handwheel_stop_chasing(&simple_tp[valid_axis]);
            Simple_Tp_Update(&simple_tp[valid_axis]);
            simple_tp[valid_axis].joint.pos_cmd = simple_tp[valid_axis].curr_pos;
            rt_data->axis_data[valid_axis].wcs_cmd_pos_cur = simple_tp[valid_axis].joint.pos_cmd;
            rt_data->axis_data[valid_axis].cmd_vel = fabs(simple_tp[valid_axis].curr_vel);
            rt_data->chan_data[0].cmd_vel = fabs(simple_tp[valid_axis].curr_vel);
        }
        return;
    } 

    selection_changed = (!mpg_initialized || !mpg_enable || axis != valid_axis || rate != valid_rate);
    if (selection_changed && mpg_initialized && valid_axis >= 0 && valid_axis < 5 && valid_axis != axis)
    {
        previous_axis = valid_axis;
        Simple_Tp_Update(&simple_tp[previous_axis]);
        simple_tp[previous_axis].joint.pos_cmd = simple_tp[previous_axis].curr_pos;
        rt_data->axis_data[previous_axis].wcs_cmd_pos_cur = simple_tp[previous_axis].joint.pos_cmd;
        rt_data->axis_data[previous_axis].cmd_vel = fabs(simple_tp[previous_axis].curr_vel);
    }

    if (selection_changed)
    {
        simple_tp[axis].joint.old_counts = current_count;
        simple_tp[axis].first_pass = 1;
        simple_tp[axis].joint.pos_cmd = rt_data->axis_data[axis].wcs_cmd_pos_cur;
        simple_tp[axis].pos_cmd = rt_data->axis_data[axis].wcs_cmd_pos_cur;
        simple_tp[axis].curr_pos = rt_data->axis_data[axis].wcs_cmd_pos_cur;
        simple_tp[axis].curr_vel = 0;
    }

    valid_axis = axis;
    valid_rate = rate;
    mpg_enable = true;
    mpg_initialized = true;
    simple_tp[axis].joint.count = current_count;
    simple_tp[axis].joint.rate = rate;
    simple_tp[axis].axis = axis;

    simple_tp[axis].joint.vel_limit = rt_data->axis_data[axis].mpg_spd;
    simple_tp[axis].joint.jaccel_limit = rt_data->axis_data[axis].max_feed_acc;
    simple_tp[axis].max_vel = simple_tp[axis].joint.vel_limit;
    simple_tp[axis].max_acc = simple_tp[axis].joint.jaccel_limit;

    if (lock_switching)
    {
        /* 锁住状态切换期间不接收新位移，并丢弃本周期计数，避免切换结束后补发 */
        simple_tp[axis].joint.old_counts = current_count;
        handwheel_stop_chasing(&simple_tp[axis]);
    }
    else
    {
        Handlewheels_Motion(&simple_tp[axis]);
    }
    Simple_Tp_Update(&simple_tp[axis]);
    simple_tp[axis].joint.pos_cmd = simple_tp[axis].curr_pos;
    rt_data->axis_data[axis].wcs_cmd_pos_cur = simple_tp[axis].joint.pos_cmd;
    rt_data->axis_data[axis].cmd_vel = fabs(simple_tp[axis].curr_vel);
    rt_data->chan_data[0].cmd_vel = fabs(simple_tp[axis].curr_vel);
}
