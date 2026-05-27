#include "handwheel.h"
#include <hmisysapi.h>
#include <math.h>

simple_tp_t_handwheel simple_tp[5];
static uint32_t axisandrate = 0;
static uint32_t current_count = 0; // 手轮计数器
static bool mpg_statis = true;     // true为复位状态
static int rate = 0;               // 倍率
static int axis = 0;               // 轴选
static int last_axis = 0;          // 上一个周期轴选
static int last_rate = 0;          // 上一个周期倍率

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

    if((delta > 0 && rt_data->axis_data[tp->axis].axis_status.is_fwd_limit) || 
        (delta < 0 && rt_data->axis_data[tp->axis].axis_status.is_rev_limit))
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
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
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
        // rt_data->axis_data[tp->axis].axis_status.is_decel_done = 0;

    } 
    else if (pos_err < -tiny_dp)
    {
        vel_req =  max_dv - sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
    } 
    else
    {
        if(rt_data->axis_data[tp->axis].axis_status.is_fwd_limit || rt_data->axis_data[tp->axis].axis_status.is_rev_limit)
        {
            rt_data->axis_data[tp->axis].axis_status.is_decel_done = 1;
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


void handle_mpg_cmd() {
    HW_Get_Count(&current_count);
    HW_Get_Axis_And_Rate(&axisandrate);
    rate = axisandrate & 0b111;
    rate = Get_Multiplier_Switch(rate);
    axis = (axisandrate >> 3) & 0b11111;
    axis = Get_Axis_Switch(axis);

    if (axis == -1 || (last_axis != axis) || (last_rate != rate)) 
    {
        for (int i = 0; i < 5; ++i) 
        {
            simple_tp[i].first_pass = 1;
        }
        mpg_statis = true;
    } 
    else if (axis >= 0 && axis < 5) 
    {
        simple_tp[axis].joint.count = current_count;
        simple_tp[axis].joint.rate = rate;
        simple_tp[axis].axis = axis;

        simple_tp[axis].joint.vel_limit = par_axis[axis][PAR_AX_MPG_SPD].value.val_real;
        simple_tp[axis].joint.jaccel_limit = par_axis[axis][PAR_AX_RAPID_ACC].value.val_real;

        //初始化
        if (mpg_statis) {
            // simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = var_axis[axis][VAR_AXIS_CMD_POS_EX].value.f;
            simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = rt_data->axis_data[axis].wcs_cmd_pos_cur;
            simple_tp[axis].curr_vel = 0;
            mpg_statis = false;
        }

        Handlewheels_Motion(&simple_tp[axis]);
        Simple_Tp_Update(&simple_tp[axis]);
        
        simple_tp[axis].joint.pos_cmd = simple_tp[axis].curr_pos;
    }
    if (!mpg_statis) {
        setVarAxisDouble(axis, VAR_AXIS_CMD_POS_EX, simple_tp[axis].joint.pos_cmd, 0);
        setVarAxisDouble(axis, VAR_AXIS_CMD_VEL_EX, abs(simple_tp[axis].curr_vel), 0);
        setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, abs(simple_tp[axis].curr_vel), 0);
    }
    last_axis = axis;
    last_rate = rate;
}
