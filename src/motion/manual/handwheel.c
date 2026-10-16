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

void handwheel_resync_position(void)
{
    /*
     * 清掉轴选/倍率的历史状态，使下一次 handle_mpg_cmd() 走“选择变化”分支：
     * 以实时数据中的 wcs_cmd_pos_cur 重建目标位置和规划位置、速度清零，
     * 并把 first_pass 置 1，从而丢弃未选择期间的累计计数。
     * 由 emcmot 在重新进入手轮模式时调用：离开手轮后其他模式可能已经移动过机床，
     * 不能沿用上一次手轮会话的规划位置（否则回到手轮的第一个周期会产生位置跳变）。
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
 * 此时 handle_mpg_cmd() 不再接收新的手轮位移、并把目标追平当前位置让规划器停稳，
 * 停稳后 handwheel_update_mac_lock() 才让 READY 跟随 ON。
 */
static bool handwheel_is_mac_lock_switching(void)
{
    return rt_data->sys_state[SYS_IS_MAC_LOCK_ON] != rt_data->sys_state[SYS_IS_MAC_LOCK_READY];
}

/* 锁住状态切换期间停止追赶已有目标：目标位置对齐当前规划位置，由规划器按斜坡停稳 */
static void handwheel_stop_chasing(simple_tp_t_handwheel *tp)
{
    tp->pos_cmd = tp->curr_pos;
}

/* 当前轴选的规划器是否已停稳（锁住状态切换的判定条件）；无有效轴选视为已停稳 */
static bool handwheel_motion_is_stopped(void)
{
    if (!mpg_initialized || valid_axis < 0 || valid_axis >= 5)
        return true;
    return fabs(simple_tp[valid_axis].curr_vel) < JOG_SPEED_THRESHOLD;
}

/*
 * 机械锁住：处理开关事件，并维护“请求(ON)”与“生效(READY)”两个状态。
 * 与自动模式一致，两者分开：
 *   ON=0,READY=0 未锁住 / ON=1,READY=0 已请求、先让手轮运动停稳
 *   ON=1,READY=1 锁住生效（机床指令位置不再累加，手轮照常规划账面）
 *   松开时对称：ON=0,READY=1 先停稳 → ON=0,READY=0 解除
 * 停稳后 READY 才跟随 ON；手轮的计数与规划状态全程保留，所以锁住期间继续摇
 * 只走账面（虚拟位移），解除后机床从冻结值继续累加、差值保留。
 */
static void handwheel_update_mac_lock(void)
{
    bool lock_request;
    bool lock_effective;

    /*
     * 先按“上一周期结束时的请求/生效状态”判断能不能切换，再处理本周期的开关事件
     * （与自动模式一致），这样 ON=1,READY=0 / ON=0,READY=1 过渡态至少存在一个周期。
     */
    lock_request = rt_data->sys_state[SYS_IS_MAC_LOCK_ON];
    lock_effective = rt_data->sys_state[SYS_IS_MAC_LOCK_READY];

    /* 请求与实际不一致（正在锁住或正在解除）：先等手轮运动停稳，再切换生效状态 */
    if (lock_request != lock_effective && handwheel_motion_is_stopped())
    {
        rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = lock_request;
        LOG(LOG_LEVEL_INFO, "手动模式机械锁住%s", lock_request ? "生效" : "解除");
    }

    /* 本周期开关事件：只改“请求” */
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_MAC_LOCK_ON:
        LOG(LOG_LEVEL_INFO, "手动模式处理开启机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = true;
        break;
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_INFO, "手动模式处理关闭机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = false;
        break;
    default:
        break;
    }
}

void handle_mpg_cmd() {
    bool valid_selection;
    bool selection_changed;
    int previous_axis;

    // 急停不执行
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE]) return;
    handwheel_update_mac_lock();
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
            /* 锁住状态切换期间同时停止追赶已有目标，等规划器停稳后再切换 */
            if (handwheel_is_mac_lock_switching())
                handwheel_stop_chasing(&simple_tp[valid_axis]);
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
    
    if (handwheel_is_mac_lock_switching())
    {
        /* 切换期间不接收新位移，并丢弃本周期计数，让规划器追平当前位置后停稳 */
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
