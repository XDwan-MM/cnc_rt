#include "ref.h"
#include "datatype.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <stdlib.h>
#include <string.h>
#include "stored_stroke_check.h"
#include "rt_data.h"

static void abs_ref_do_home(AXIS_TYPE axis_type);
static void relative_ref_do_home(AXIS_TYPE axis_type);
static void abs_ref_update_state(AXIS_TYPE axis_type);
static void abs_ref_check_event(AXIS_TYPE axis_type);
static void abs_ref_update_cmd(AXIS_TYPE axis_type);
static void abs_ref_execute_cmd(AXIS_TYPE axis_type);
static void abs_ref_tp_execute_move_cmd( AXIS_TYPE axis_type);
static void abs_ref_tp_update_traj(AXIS_TYPE axis_type);
static void abs_ref_tp_cal_target_vel(AXIS_TYPE axis_type);
static GNC_RES abs_ref_tp_do_velplan(AXIS_TYPE axis_type);
static GNC_RES vp_val_decline_profile(AXIS_TYPE axis_type);
static bool ref_tp_new_vel_valid(AXIS_TYPE axis_type);
static void ref_cal_chan_feedrate();
static void ref_handle_alarm();
static void ref_update_wcs_pos(AXIS_TYPE axis_type);
static bool ref_is_tp_finished(AXIS_TYPE axis_type);
static void abs_ref_tp_get_cmd(AXIS_TYPE axis_type);
static void abs_ref_tp_update_cmd(AXIS_TYPE axis_type);
static void ref_tp_reset_ref_data(AXIS_TYPE axis_type);
static void ref_tp_cal_ref_block(AXIS_TYPE axis_type);
static void ref_tp_axis_reset(AXIS_TYPE axis_type);
static void ref_tp_axis_pause(AXIS_TYPE axis_type);
static bool check_axis_ref_arrive(AXIS_TYPE axis_type);
static void abs_ref_update_state_by_event(AXIS_TYPE axis_type);

void handle_ref_cmd(){
    ref_handle_alarm();
    for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
        switch(rt_data->axis_data[axis_type].home_way){
            case ABSOLUTE_AXIS_HOMING:
                abs_ref_do_home(axis_type);
                break;
            case RELATIVE_AXIS_HOMING:
                relative_ref_do_home(axis_type);
                break;
        }
    }
    ref_cal_chan_feedrate();
}

/// @brief 绝对式回零
static void abs_ref_do_home(AXIS_TYPE axis_type){
    abs_ref_update_state(axis_type);
    abs_ref_check_event(axis_type);
    abs_ref_update_cmd(axis_type);
    abs_ref_execute_cmd(axis_type);
    abs_ref_update_cmd(axis_type);
}

// todo：相对式回零
static void relative_ref_do_home(AXIS_TYPE axis_type){
}

static void abs_ref_update_state(AXIS_TYPE axis_type){
    // 报警或切换模式下，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]) return;
    switch (axis_type)
    {
    case AXIS_X:
        switch (ref_tp[AXIS_X].ref_state)
        {
        case REF_ABS_EXEC_IDLE:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_X_POS_PRESS:
            case SYS_EVENT_X_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_START;
                LOG(LOG_LEVEL_INFO, "回零处理X按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_MOVING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_X_POS_RELEASE:
            case SYS_EVENT_X_NEG_RELEASE:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_PAUSING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理X松开");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_PAUSING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_X_POS_PRESS:
            case SYS_EVENT_X_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理X按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_RESETTING:
            break;
        }
        break;
    case AXIS_Y:
        switch (ref_tp[AXIS_Y].ref_state)
        {
        case REF_ABS_EXEC_IDLE:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_Y_POS_PRESS:
            case SYS_EVENT_Y_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_START;
                LOG(LOG_LEVEL_INFO, "回零处理Y按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_MOVING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_Y_POS_RELEASE:
            case SYS_EVENT_Y_NEG_RELEASE:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_PAUSING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理Y松开");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_PAUSING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_Y_POS_PRESS:
            case SYS_EVENT_Y_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理Y按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_RESETTING:
            break;
        }
        break;
    case AXIS_Z:
        switch (ref_tp[AXIS_Z].ref_state)
        {
        case REF_ABS_EXEC_IDLE:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_Z_POS_PRESS:
            case SYS_EVENT_Z_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_START;
                LOG(LOG_LEVEL_INFO, "回零处理Z按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_MOVING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_Z_POS_RELEASE:
            case SYS_EVENT_Z_NEG_RELEASE:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_PAUSING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理Z松开");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_PAUSING:
            switch (rt_data->nc_event)
            {
            case SYS_EVENT_HMI_RESET:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                LOG(LOG_LEVEL_INFO, "回零处理复位");
                break;
            case SYS_EVENT_Z_POS_PRESS:
            case SYS_EVENT_Z_NEG_PRESS:
                ref_tp[axis_type].ref_state = REF_ABS_EXEC_MOVING;
                ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                LOG(LOG_LEVEL_INFO, "回零处理Z按下");
                break;
            default:
                break;
            }
            break;
        case REF_ABS_EXEC_RESETTING:
            break;
        }
        break;
    default:
        break;
    } 
}

static void abs_ref_check_event(AXIS_TYPE axis_type){
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_RAPID_OVERRIDE:
        switch (ref_tp[axis_type].ref_state)
        {
        case REF_ABS_EXEC_MOVING:
            LOG(LOG_LEVEL_INFO, "回零处理快速进给倍率改变");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_MAC_LOCK_ON:
        LOG(LOG_LEVEL_INFO, "回零模式处理开启机械锁住");
        // TODO: 回零模式机械锁住
        break;
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_INFO, "回零模式处理关闭机械锁住");
        break;    
    case SYS_EVENT_STLK_ON:
    case SYS_EVENT_IT_ON:
        LOG(LOG_LEVEL_INFO, "回零模式处理开启所有轴互锁");
        abs_ref_update_state_by_event(axis_type);
        break;
    case SYS_EVENT_IT1_ON:
        switch (axis_type)
        {
        case AXIS_X:
            LOG(LOG_LEVEL_INFO, "回零模式处理开启轴1锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_IT2_ON:
        switch (axis_type)
        {
        case AXIS_Y:
            LOG(LOG_LEVEL_INFO, "回零模式处理开启轴2锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_IT3_ON:
        switch (axis_type)
        {
        case AXIS_Z:
            LOG(LOG_LEVEL_INFO, "回零模式处理开启轴3锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_STLK_OFF:
    case SYS_EVENT_IT_OFF:
        LOG(LOG_LEVEL_INFO, "回零模式处理关闭所有轴互锁");
        abs_ref_update_state_by_event(axis_type);
        break;
    case SYS_EVENT_IT1_OFF:
        switch (axis_type)
        {
        case AXIS_X:
            LOG(LOG_LEVEL_INFO, "回零模式处理关闭轴1锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_IT2_OFF:
        switch (axis_type)
        {
        case AXIS_Y:
            LOG(LOG_LEVEL_INFO, "回零模式处理关闭轴2锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    case SYS_EVENT_IT3_OFF:
        switch (axis_type)
        {
        case AXIS_Z:
            LOG(LOG_LEVEL_INFO, "回零模式处理关闭轴3锁");
            abs_ref_update_state_by_event(axis_type);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void abs_ref_update_cmd(AXIS_TYPE axis_type){
    switch (ref_tp[axis_type].current_stage)
    {
    case REF_ARRIVED:
        switch (ref_tp[axis_type].ref_state){
            case REF_ABS_EXEC_MOVING:
            case REF_ABS_EXEC_PAUSING:
            case REF_ABS_EXEC_RESETTING:
                LOG(LOG_LEVEL_INFO, "轴%d回零完成", axis_type);
                ref_update_wcs_pos(axis_type);
                set_ref_state(FIRST_REF, axis_type);
                break;
            case REF_ABS_EXEC_IDLE:
                break;
        }
        ref_tp_axis_reset(axis_type);
        break;
    case REF_PAUSED:
        switch (ref_tp[axis_type].ref_state){
            case REF_ABS_EXEC_MOVING:
                ref_tp_axis_pause(axis_type);
                break;
            case REF_ABS_EXEC_PAUSING:
                LOG(LOG_LEVEL_INFO, "轴%d减速完成", axis_type);
                ref_tp_axis_reset(axis_type);
                break;
            case REF_ABS_EXEC_RESETTING:
                LOG(LOG_LEVEL_INFO, "轴%d复位完成", axis_type);
                ref_tp_axis_reset(axis_type);
                break;
            case REF_ABS_EXEC_IDLE:
                break;
        }
        break;
    case REF_END:
        switch (ref_tp[axis_type].ref_state){
            case REF_ABS_EXEC_MOVING:
            case REF_ABS_EXEC_PAUSING:
                LOG(LOG_LEVEL_INFO, "轴%d减速完成", axis_type);
                ref_tp_axis_reset(axis_type);
                break;
            case REF_ABS_EXEC_RESETTING:
                LOG(LOG_LEVEL_INFO, "轴%d复位完成", axis_type);
                ref_tp_axis_reset(axis_type);
                break;
            case REF_ABS_EXEC_IDLE:
                break;
        }
        break;
    case REF_START:
        abs_ref_tp_get_cmd(axis_type);
        ref_tp[axis_type].current_stage = REF_MOVING;
        ref_tp[axis_type].ref_cmd = REF_DO_MOVE_CMD;
        break;
    case REF_BREAKPOINT:
        abs_ref_tp_update_cmd(axis_type);
        ref_tp[axis_type].current_stage = REF_MOVING;
        ref_tp[axis_type].ref_cmd = REF_DO_MOVE_CMD;
        break;
    case REF_MOVING:
        break;
    }
}

static void abs_ref_execute_cmd(AXIS_TYPE axis_type){
    switch (ref_tp[axis_type].ref_cmd)
    {
    case REF_DO_MOVE_CMD:
        abs_ref_tp_execute_move_cmd(axis_type);
        if(ref_is_tp_finished(axis_type)){
            ref_tp[axis_type].current_stage = ref_tp[axis_type].target_stage;
        }
        break;
    case REF_DO_NULL_CMD:
        ref_tp[axis_type].current_stage = ref_tp[axis_type].target_stage;
        break;
    }
}

void ref_handle_estop(){
    ref_tp_reset();
}

static void ref_handle_alarm(){
    // 报警或切换模式下，减速停止
    static bool reset_trigger = false;
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        if(!reset_trigger){
            reset_trigger = true;
            LOG(LOG_LEVEL_INFO, "回零处理报警或切换模式");
            for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
                switch (ref_tp[axis_type].ref_state)
                {
                case REF_ABS_EXEC_MOVING:
                    ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                    ref_tp[axis_type].current_stage = REF_BREAKPOINT;
                    break;
                case REF_ABS_EXEC_PAUSING:
                    ref_tp[axis_type].ref_state = REF_ABS_EXEC_RESETTING;
                    break;
                default:
                    break;
                }
            }
        }
        return;
    }else reset_trigger = false;
}

static void abs_ref_tp_get_cmd(AXIS_TYPE axis_type){
    ref_tp_cal_ref_block(axis_type);
    // 华中：在回参考点偏差范围内，默认到达参考点
    // if(IsLessThanOrEqual(ref_tp[axis_type].ref_block.length, rt_data->axis_data[axis_type].ref_range))
    if(IsLessThanOrEqual(ref_tp[axis_type].ref_block.length, 0)){
        ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
        ref_tp[axis_type].target_stage = REF_ARRIVED;
        return;
    }
    abs_ref_tp_cal_target_vel(axis_type);
    if(IsEqualTo(ref_tp[axis_type].ref_data.current_vel, 0) && IsEqualTo(ref_tp[axis_type].ref_data.target_vel, 0)){
        ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
        ref_tp[axis_type].target_stage = REF_PAUSED;
        return;
    }
    abs_ref_tp_do_velplan(axis_type);
}

static void abs_ref_tp_update_cmd(AXIS_TYPE axis_type){
    abs_ref_tp_cal_target_vel(axis_type);
    if(IsEqualTo(ref_tp[axis_type].ref_data.current_vel, 0) && IsEqualTo(ref_tp[axis_type].ref_data.target_vel, 0)){
        ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
        ref_tp[axis_type].target_stage = REF_PAUSED;
        return;
    }
    // 如果新速度不满足，不更新新速度
    if(!ref_tp_new_vel_valid(axis_type)) return;
    ref_tp_cal_ref_block(axis_type);
    abs_ref_tp_do_velplan(axis_type);
}

/// @brief 计算目标速度
static void abs_ref_tp_cal_target_vel(AXIS_TYPE axis_type){
    switch (ref_tp[axis_type].ref_state){
        case REF_ABS_EXEC_MOVING:
            ref_tp[axis_type].ref_data.target_vel = MIN(ref_tp[axis_type].ref_block.feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, ref_tp[axis_type].ref_block.param_vel);
            break;
        case REF_ABS_EXEC_PAUSING:
        case REF_ABS_EXEC_RESETTING:
        case REF_ABS_EXEC_IDLE:
            ref_tp[axis_type].ref_data.target_vel = 0;
            break;
    }
    if(rt_data->sys_state[SYS_IS_MAC_LOCK_READY] != rt_data->sys_state[SYS_IS_MAC_LOCK]) ref_tp[axis_type].ref_data.target_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT_LOCK]) ref_tp[axis_type].ref_data.target_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT1_LOCK]) ref_tp[AXIS_X].ref_data.target_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT2_LOCK]) ref_tp[AXIS_Y].ref_data.target_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT3_LOCK]) ref_tp[AXIS_Z].ref_data.target_vel = 0;
}

static void abs_ref_tp_execute_move_cmd(AXIS_TYPE axis_type){
    if(ref_is_tp_finished(axis_type)) return;
    ref_tp[axis_type].ref_data.interp_cycle += 1;
    vp_cal_traj_data(rt_data->chan_data[0].vplan_mode, ref_tp[axis_type].ref_data.profile, ref_tp[axis_type].ref_data.interp_cycle, ref_tp[axis_type].ref_data.traj_data, rt_data->ncu_data.cycle);
    abs_ref_tp_update_traj(axis_type);
}

static void abs_ref_tp_update_traj(AXIS_TYPE axis_type){
    ref_tp[axis_type].ref_data.current_distance = ref_tp[axis_type].ref_data.target_distance;
    ref_tp[axis_type].ref_data.target_distance = ref_tp[axis_type].ref_data.traj_data->s - ref_tp[axis_type].ref_data.accum_distance;
    ref_tp[axis_type].ref_data.accum_distance = ref_tp[axis_type].ref_data.traj_data->s;
    ref_tp[axis_type].ref_data.remain_distance = ref_tp[axis_type].ref_block.length - ref_tp[axis_type].ref_data.accum_distance;
    ref_tp[axis_type].ref_data.current_vel = ref_tp[axis_type].ref_data.traj_data->v;
    // 更新位置
    if(ref_tp[axis_type].ref_data.direction == 0) {
        rt_data->axis_data[axis_type].wcs_cmd_pos_cur -= ref_tp[axis_type].ref_data.target_distance;
    }else{
        rt_data->axis_data[axis_type].wcs_cmd_pos_cur += ref_tp[axis_type].ref_data.target_distance;
    }
    // 更新速度
    rt_data->axis_data[axis_type].cmd_vel = ref_tp[axis_type].ref_data.traj_data->v;
    rt_data->axis_data[axis_type].cmd_acc = ref_tp[axis_type].ref_data.traj_data->a;
    rt_data->axis_data[axis_type].cmd_jk = ref_tp[axis_type].ref_data.traj_data->j;
}

// 检查是否执行完速度曲线
static bool ref_is_tp_finished(AXIS_TYPE axis_type){
    if(ref_tp[axis_type].ref_data.interp_cycle >= ref_tp[axis_type].ref_data.profile->t_cycle) {
        ref_tp_reset_ref_data(axis_type);
        return true;
    }
    return false;
}

static void ref_update_wcs_pos(AXIS_TYPE axis_type){
    // 到达零点，重置cmd_pos
    rt_data->axis_data[axis_type].wcs_cmd_pos_cur = rt_data->axis_data[axis_type].home_crds;
    rt_data->axis_data[axis_type].wcs_cmd_pos_pre = rt_data->axis_data[axis_type].home_crds;
    rt_data->axis_data[axis_type].mac_cmd_pos = rt_data->axis_data[axis_type].home_crds;
}

static bool ref_tp_new_vel_valid(AXIS_TYPE axis_type){
    double vs = fabs(rt_data->chan_data[0].cmd_vel); double ve = 0;
    double target_vel = ref_tp[axis_type].ref_data.target_vel;
    double remain_s = fabs(rt_data->axis_data[axis_type].home_crds - rt_data->axis_data[axis_type].mac_cmd_pos);
    double total_decline_length = 0;
    if(vs > target_vel && target_vel >= ve){
        double length1 = vp_cal_decel_dist(rt_data->chan_data[0].vplan_mode, vs, target_vel, ref_tp[axis_type].ref_block.max_acc, ref_tp[axis_type].ref_block.max_jerk);
        double length2 = vp_cal_decel_dist(rt_data->chan_data[0].vplan_mode, target_vel, ve, ref_tp[axis_type].ref_block.max_acc, ref_tp[axis_type].ref_block.max_jerk);
        total_decline_length = length1 + length2; 
    }else if(target_vel > vs && vs >= ve){
        total_decline_length = vp_cal_decel_dist(rt_data->chan_data[0].vplan_mode, vs, ve, ref_tp[axis_type].ref_block.max_acc, ref_tp[axis_type].ref_block.max_jerk);
    }
    if(IsGreaterThanOrEqual(total_decline_length, remain_s)) return false;
    return true;
}

static GNC_RES abs_ref_tp_do_velplan(AXIS_TYPE axis_type){
    GNC_RES res = VP_CAL_SUCCESS;
    ref_tp_reset_ref_data(axis_type);  
    if(IsGreaterThan(ref_tp[axis_type].ref_data.current_vel, ref_tp[axis_type].ref_data.target_vel)){
        res = vp_val_decline_profile(axis_type);
    }else{
        res = vp_cal_vel_profile(rt_data->chan_data[0].vplan_mode, &ref_tp[axis_type].ref_block, ref_tp[axis_type].ref_data.profile, rt_data->ncu_data.cycle); 
        ref_tp[axis_type].target_stage = REF_ARRIVED;
    }
    if(res != VP_CAL_SUCCESS) {
        setAlarm(ALARM_CH, ALARM_ERROR, 1);
        LOG(LOG_LEVEL_ERR, "轴%d回零错误！回零模式报警 %d", axis_type, res);
        LOG(LOG_LEVEL_DEBUG, "vs:%.10f, ve:%.10f, vmax:%.10f, acc:%.10f, jerk:%.10f, length:%.10f\n", ref_tp[axis_type].ref_data.profile->vs, ref_tp[axis_type].ref_data.profile->ve, ref_tp[axis_type].ref_block.max_vel, ref_tp[axis_type].ref_block.max_acc, ref_tp[axis_type].ref_block.max_jerk, ref_tp[axis_type].ref_block.length);
        ref_tp[axis_type].target_stage = REF_END;
        ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
        return res;
    }
    return res;
}

static GNC_RES vp_val_decline_profile(AXIS_TYPE axis_type){
    GNC_RES res = VP_CAL_SUCCESS;
    double vs = ref_tp[axis_type].ref_block.start_vel; double ve = ref_tp[axis_type].ref_data.target_vel;
    double remain_length = fabs(rt_data->axis_data[axis_type].home_crds - rt_data->axis_data[axis_type].mac_cmd_pos);
    ref_tp[axis_type].ref_block.end_vel = ref_tp[axis_type].ref_data.target_vel;
    ref_tp[axis_type].ref_block.length = vp_cal_decel_dist(rt_data->chan_data[0].vplan_mode, vs, ve, ref_tp[axis_type].ref_block.max_acc, ref_tp[axis_type].ref_block.max_jerk); // 减速距离
    if(rt_data->axis_data[axis_type].home_crds > rt_data->axis_data[axis_type].mac_cmd_pos) ref_tp[axis_type].ref_data.direction = 1;  // 回零移动方向
    else ref_tp[axis_type].ref_data.direction = 0;
    // todo：如果减速距离极小
    if(IsEqualTo(ref_tp[axis_type].ref_block.length, 0)){
        ref_tp[axis_type].ref_data.profile->t_cycle = 0;
        ref_tp[axis_type].ref_data.traj_data->v = ref_tp[axis_type].ref_data.target_vel;
        if(IsEqualTo(ve, 0)) ref_tp[axis_type].target_stage = REF_PAUSED;
        else ref_tp[axis_type].target_stage = REF_BREAKPOINT;
        LOG(LOG_LEVEL_DEBUG, "轴%d减速距离极小:%f,当前速度:%f,目标速度:%f", axis_type, ref_tp[axis_type].ref_block.length, vs, ve);
        return res;
    }
    else if(IsEqualTo(ref_tp[axis_type].ref_block.length, remain_length)){
        if(IsEqualTo(ve, 0)) ref_tp[axis_type].target_stage = REF_ARRIVED;
        else ref_tp[axis_type].target_stage = REF_BREAKPOINT;
    }
    else {
        if(IsEqualTo(ve, 0)) ref_tp[axis_type].target_stage = REF_PAUSED;
        else ref_tp[axis_type].target_stage = REF_BREAKPOINT;
    }
    res = vp_cal_vel_profile(rt_data->chan_data[0].vplan_mode, &ref_tp[axis_type].ref_block, ref_tp[axis_type].ref_data.profile, rt_data->ncu_data.cycle); 
    return res;
}

static void ref_cal_chan_feedrate(){
    rt_data->chan_data[0].cmd_vel = sqrt(pow(rt_data->axis_data[0].cmd_vel, 2) + pow(rt_data->axis_data[1].cmd_vel, 2) + pow(rt_data->axis_data[2].cmd_vel, 2));
}

GNC_RES ref_tp_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    ref_tp = (REF_TP*)calloc(SYS_AXES_NUM * sizeof(REF_TP), 1);
    if(ref_tp == NULL) return GNC_DATA_NULL;
    for(int i = 0; i < SYS_AXES_NUM; i++){
        ref_tp[i].ref_data.profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
        ref_tp[i].ref_data.traj_data = (TRAJ_PARAMS*)calloc(sizeof(TRAJ_PARAMS), 1);
        if(ref_tp[i].ref_data.profile == NULL || ref_tp[i].ref_data.traj_data == NULL){
            ref_tp_delete();
            return GNC_DATA_NULL;
        }  
    }
    ref_tp_reset();
    return res;
}

void ref_tp_delete(){
    if(ref_tp == NULL) return;
    for(int i = 0; i < SYS_AXES_NUM; i++){
        if(ref_tp[i].ref_data.profile != NULL){
            free(ref_tp[i].ref_data.profile);
            ref_tp[i].ref_data.profile = NULL;
        }
        if(ref_tp[i].ref_data.traj_data != NULL){
            free(ref_tp[i].ref_data.traj_data);
            ref_tp[i].ref_data.traj_data = NULL;
        }
    }
    free(ref_tp);
    ref_tp = NULL;
}

void ref_tp_reset(){
    if(ref_tp == NULL) return;
    for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
        ref_tp[axis_type].ref_state = REF_ABS_EXEC_IDLE;
        ref_tp[axis_type].current_stage = REF_END;
        ref_tp[axis_type].target_stage = REF_END;
        ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
        ref_tp_reset_ref_data(axis_type);
    }
    rt_set_zero_vel_params();
}

static void ref_tp_axis_reset(AXIS_TYPE axis_type){
    if(ref_tp == NULL) return;
    ref_tp[axis_type].ref_state = REF_ABS_EXEC_IDLE;
    ref_tp[axis_type].current_stage = REF_END;
    ref_tp[axis_type].target_stage = REF_END;
    ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
    ref_tp_reset_ref_data(axis_type);
    rt_set_axis_zero_vel_params(axis_type);
}

static void ref_tp_axis_pause(AXIS_TYPE axis_type){
    if(ref_tp == NULL) return;
    ref_tp[axis_type].current_stage = REF_PAUSED;
    ref_tp[axis_type].target_stage = REF_PAUSED;
    ref_tp[axis_type].ref_cmd = REF_DO_NULL_CMD;
    ref_tp_reset_ref_data(axis_type);
    rt_set_axis_zero_vel_params(axis_type);
}

static void ref_tp_reset_ref_data(AXIS_TYPE axis_type){
    ref_tp[axis_type].ref_data.interp_cycle = 0;
    ref_tp[axis_type].ref_data.accum_distance = 0;
    ref_tp[axis_type].ref_data.remain_distance = 0;
    ref_tp[axis_type].ref_data.current_distance = 0;
    ref_tp[axis_type].ref_data.target_distance = 0;
    memset(ref_tp[axis_type].ref_data.traj_data, 0, sizeof(TRAJ_PARAMS));
    memset(ref_tp[axis_type].ref_data.profile, 0, sizeof(VEL_PROFILE));
}

static void ref_tp_cal_ref_block(AXIS_TYPE axis_type){
    ref_tp[axis_type].ref_block.length = fabs(rt_data->axis_data[axis_type].home_crds - rt_data->axis_data[axis_type].mac_cmd_pos);
    ref_tp[axis_type].ref_block.start_vel = rt_data->axis_data[axis_type].cmd_vel;
    ref_tp[axis_type].ref_block.end_vel  = 0.0;
    ref_tp[axis_type].ref_block.mode = PMC_MODE_ARRIVE_CHECK;
    ref_tp[axis_type].ref_block.type = LINE;
    ref_tp[axis_type].ref_block.feed_vel = rt_data->axis_data[axis_type].jog_lowspd;
    ref_tp[axis_type].ref_block.param_vel = rt_data->axis_data[axis_type].max_feed_vel;
    if(rt_data->sys_state[SYS_IS_IT_LOCK]) ref_tp[axis_type].ref_block.param_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT1_LOCK]) ref_tp[AXIS_X].ref_block.param_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT2_LOCK]) ref_tp[AXIS_Y].ref_block.param_vel = 0;
    if(rt_data->sys_state[SYS_IS_IT3_LOCK]) ref_tp[AXIS_Z].ref_block.param_vel = 0;
    ref_tp[axis_type].ref_block.max_vel = MIN(ref_tp[axis_type].ref_block.feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, ref_tp[axis_type].ref_block.param_vel);
    if(IsGreaterThan(ref_tp[axis_type].ref_block.start_vel, ref_tp[axis_type].ref_block.max_vel)) ref_tp[axis_type].ref_block.max_vel = ref_tp[axis_type].ref_block.start_vel;
    ref_tp[axis_type].ref_block.max_acc = rt_data->axis_data[axis_type].max_feed_acc;
    ref_tp[axis_type].ref_block.max_jerk = rt_data->axis_data[axis_type].max_feed_jk;
    if(rt_data->axis_data[axis_type].home_crds > rt_data->axis_data[axis_type].mac_cmd_pos) ref_tp[axis_type].ref_data.direction = 1;
    else ref_tp[axis_type].ref_data.direction = 0;
}

static void abs_ref_update_state_by_event(AXIS_TYPE axis_type){
    switch (ref_tp[axis_type].current_stage)
    {
    case REF_MOVING:
        ref_tp[axis_type].current_stage = REF_BREAKPOINT;
        break;
    case REF_PAUSED:
        ref_tp[axis_type].current_stage = REF_START;
        break;
    default:
        break;
    }
}

/// @brief 设置回参考点完成信号
/// @param ref_type 回第几参考点
void set_ref_state(REF_TYPE ref_type, AXIS_TYPE axis_type){
    switch (ref_type)
    {
    case FIRST_REF:
        rt_data->axis_data[axis_type].ref_status.ref_on = 1;
        rt_data->axis_data[axis_type].ref_status.ref_setup = 1;
        break;
    case SECOND_REF:
        rt_data->axis_data[axis_type].ref_status.second_ref_on = 1;
        break;
    case THIRD_REF:
        rt_data->axis_data[axis_type].ref_status.third_ref_on = 1;
        break;
    case FORTH_REF:
        rt_data->axis_data[axis_type].ref_status.forth_ref_on = 1;
        break;
    default:
        break;
    }
    LOG(LOG_LEVEL_INFO, "%d轴回第%d参考点完成", axis_type, ref_type);
}

void auto_set_ref_state(REF_TYPE ref_type, int axis_type){
    switch (axis_type)
    {
    case PMC_REF_AXIS_X:
        set_ref_state(ref_type, AXIS_X);
        break;
    case PMC_REF_AXIS_Y:
        set_ref_state(ref_type, AXIS_Y);
        break;  
    case PMC_REF_AXIS_Z:
        set_ref_state(ref_type, AXIS_Z);
        break;
    case PMC_REF_AXIS_X + PMC_REF_AXIS_Y:
        set_ref_state(ref_type, AXIS_X);
        set_ref_state(ref_type, AXIS_Y);
        break;
    case PMC_REF_AXIS_X + PMC_REF_AXIS_Z:
        set_ref_state(ref_type, AXIS_X);
        set_ref_state(ref_type, AXIS_Z);
        break;
    case PMC_REF_AXIS_Y + PMC_REF_AXIS_Z:
        set_ref_state(ref_type, AXIS_Y);
        set_ref_state(ref_type, AXIS_Z);
        break;
    case PMC_REF_AXIS_X + PMC_REF_AXIS_Y + PMC_REF_AXIS_Z:
        set_ref_state(ref_type, AXIS_X);
        set_ref_state(ref_type, AXIS_Y);
        set_ref_state(ref_type, AXIS_Z);
        break;
    default:
        for (int i = 0; i < SYS_AXES_NUM; i++) {
            set_ref_state(cycle_tp->plan_block->ref_number, i);
        }
        break;
    }
}

static bool check_axis_ref_arrive(AXIS_TYPE axis_type){
    int res = 0;
    bool is_arrived = true;
    if((fabs(rt_data->axis_data[axis_type].mac_cmd_pos - rt_data->axis_data[axis_type].home_crds)) > rt_data->axis_data[axis_type].pos_tol) {
        is_arrived = false;
        res = setAlarm(ALARM_AX, ALARM_ERROR, 123 + axis_type);
        if (res != -8) LOG(LOG_LEVEL_ERR, "%d轴回零检测错误", axis_type);
    }
    return is_arrived;
}

bool check_ref_arrive(REF_TYPE ref_type, int axis_type){
    bool is_arrived = true;
    switch (ref_type)
    {
    case FIRST_REF:
        switch (axis_type)
        {
        case PMC_REF_AXIS_X:
            if(!check_axis_ref_arrive(AXIS_X)) return false;
            break;
        case PMC_REF_AXIS_Y:
            if(!check_axis_ref_arrive(AXIS_Y)) return false;
            break;  
        case PMC_REF_AXIS_Z:
            if(!check_axis_ref_arrive(AXIS_Z)) return false;
            break;
        case PMC_REF_AXIS_X + PMC_REF_AXIS_Y:
            is_arrived = check_axis_ref_arrive(AXIS_X);
            is_arrived = check_axis_ref_arrive(AXIS_Y);
            if(!is_arrived) return false;
            break;
        case PMC_REF_AXIS_X + PMC_REF_AXIS_Z:
            is_arrived = check_axis_ref_arrive(AXIS_X);
            is_arrived = check_axis_ref_arrive(AXIS_Z);
            if(!is_arrived) return false;
            break;
        case PMC_REF_AXIS_Y + PMC_REF_AXIS_Z:
            is_arrived = check_axis_ref_arrive(AXIS_Y);
            is_arrived = check_axis_ref_arrive(AXIS_Z);
            if(!is_arrived) return false;
            break;
        case PMC_REF_AXIS_X + PMC_REF_AXIS_Y + PMC_REF_AXIS_Z:
            is_arrived = check_axis_ref_arrive(AXIS_X);
            is_arrived = check_axis_ref_arrive(AXIS_Y);
            is_arrived = check_axis_ref_arrive(AXIS_Z);
            if(!is_arrived) return false;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    auto_set_ref_state(ref_type, axis_type);
    return true;
}


/// @brief 从参考点移动，成为0
void update_ref_state(){
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        if(IsGreaterThan(fabs(rt_data->axis_data[i].home_crds - rt_data->axis_data[i].mac_cmd_pos), rt_data->axis_data[i].ref_range)){
            rt_data->axis_data[i].ref_status.ref_on = 0;
        }
        if(IsGreaterThan(fabs(rt_data->axis_data[i].second_home_crds - rt_data->axis_data[i].mac_cmd_pos), rt_data->axis_data[i].ref_range)){
            rt_data->axis_data[i].ref_status.second_ref_on = 0;
        }
        if(IsGreaterThan(fabs(rt_data->axis_data[i].third_home_crds - rt_data->axis_data[i].mac_cmd_pos), rt_data->axis_data[i].ref_range)){
            rt_data->axis_data[i].ref_status.third_ref_on = 0;
        }
        if(IsGreaterThan(fabs(rt_data->axis_data[i].forth_home_crds - rt_data->axis_data[i].mac_cmd_pos), rt_data->axis_data[i].ref_range)){
            rt_data->axis_data[i].ref_status.forth_ref_on = 0;
        }
    }
}

/// @brief 紧急停止中、伺服报警时，成为0
void reset_ref_state(){
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        rt_data->axis_data[i].ref_status.ref_on = 0;
        rt_data->axis_data[i].ref_status.second_ref_on = 0;
        rt_data->axis_data[i].ref_status.third_ref_on = 0;
        rt_data->axis_data[i].ref_status.forth_ref_on = 0;
    }
}