#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "auto_gcode.h"
#include "auto.h"
#include "global.h"
#include "datatype.h"
#include "plan_queue.h"
#include "pathplan.h"
#include <hmisysapi.h>
#include "varapi.h"
#include "interpolation.h"
#include "stored_stroke_check.h"
#include <alarmapi.h>

static void auto_cal_target_vel();
static void auto_look_ahead();
static GNC_RES auto_do_vel_plan();
static GNC_RES vp_val_decline_profile();
static bool is_newval_unvalid();
static void auto_cal_target_vel();
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block);
static void block_cal_exact_stop_vel(PLAN_BLOCK* current_block);
static void block_cal_tap_vel(PLAN_BLOCK* current_block);
static void block_cal_chop_vel(PLAN_BLOCK* current_block);
static void block_cal_default_vel(PLAN_BLOCK* current_block);
static void cycle_tp_update_block_data();
static void cycle_tp_update_pos_cmd_data();
static bool auto_pos_cmd_cycle_is_finished();
static bool auto_pos_cmd_axis_arrive_check();
static bool auto_pos_cmd_arrive_check();

void auto_get_pos_cmd(){
    cycle_tp_update_pos_cmd_data();
    switch (cycle_tp->plan_block->type)
    {
    case RAPID:
        cycle_tp->exec_cmd = DO_MOVE_CMD;
        break;
    case TAP_RETURN:
    case RIGID_TAP:
        cycle_tp->exec_cmd = DO_TAP_CMD;
        break;
    default:
        cycle_tp->exec_cmd = DO_CHOP_CMD;
        break;
    }
    // 更新目标速度
    auto_cal_target_vel();
    // 当前速度和目标速度为0
    if (IsEqualTo(cycle_tp->pos_cmd_data.current_vel, 0) && IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 0)) {
        cycle_tp->target_stage = PROGRAM_WAIT;
        return;
    }
    // 如果长度为0
    if(IsLessThanOrEqual(cycle_tp->plan_block->length, 0)){
        cycle_tp->exec_cmd = DO_NULL_CMD;
        cycle_tp->target_stage = BLOCK_STOP_2_END;
        return;
    }
    // 速度前瞻
    auto_look_ahead();
    // 行程限位检查
    lookahead_check_soft_limit(&cycle_tp->plan_block->end_pos);
    // 速度规划
    auto_do_vel_plan();
}

void auto_update_pos_cmd(){
    cycle_tp_update_pos_cmd_data();
    switch (cycle_tp->plan_block->type)
    {
    case RAPID:
        cycle_tp->exec_cmd = DO_MOVE_CMD;
        break;
    case TAP_RETURN:
    case RIGID_TAP:
        cycle_tp->exec_cmd = DO_TAP_CMD;
        break;
    default:
        cycle_tp->exec_cmd = DO_CHOP_CMD;
        break;
    }
    // 更新目标速度
    auto_cal_target_vel();
    // 当前速度和目标速度为0
    if (IsEqualTo(cycle_tp->pos_cmd_data.current_vel, 0) && IsEqualTo(cycle_tp->pos_cmd_data.target_vel, 0)) {
        cycle_tp->target_stage = PROGRAM_WAIT;
        return;
    }
    // 判断是否需要重新计算速度曲线
    if(is_newval_unvalid()) return;
    // 更新块信息
    cycle_tp_update_block_data();
    // 如果长度为0
    if(IsLessThanOrEqual(cycle_tp->plan_block->length, 0)){
        cycle_tp->exec_cmd = DO_NULL_CMD;
        cycle_tp->target_stage = BLOCK_STOP_2_END;
        return;
    }
    // 速度前瞻
    auto_look_ahead();
    // 行程限位检查
    lookahead_check_soft_limit(&cycle_tp->plan_block->end_pos);
    // 速度规划
    auto_do_vel_plan();
}

void auto_execute_pos_cmd(){
    if (auto_pos_cmd_cycle_is_finished()) return;
    cycle_tp->pos_cmd_data.interp_cycle += 1;
    // 计算插补速度和距离
    vp_cal_traj_data(cycle_tp->velplan_mode, cycle_tp->pos_cmd_data.vel_profile, cycle_tp->pos_cmd_data.interp_cycle, cycle_tp->pos_cmd_data.traj_data, cycle_tp->ncu_cycle);
    cycle_tp->pos_cmd_data.current_distance = cycle_tp->pos_cmd_data.target_distance;
    cycle_tp->pos_cmd_data.target_distance = cycle_tp->pos_cmd_data.traj_data->s - cycle_tp->pos_cmd_data.accum_distance;
    if(IsLessThan(cycle_tp->pos_cmd_data.target_distance, 0)){
        setAlarm(ALARM_CH, ALARM_ERROR, 2);
        cycle_tp->target_stage = PROGRAM_END;
        cycle_tp->exec_cmd = DO_NULL_CMD;
        LOG(LOG_LEVEL_ERR, "在插补段%d插补距离计算失败", cycle_tp->plan_block->line_number);
        LOG(LOG_LEVEL_DEBUG, "vs:%.10f, ve:%.10f, vmax:%.10f, vpeak:%.10f, acc:%.10f, jerk:%.10f, length:%.10f\n", cycle_tp->pos_cmd_data.vel_profile->vs,  cycle_tp->pos_cmd_data.vel_profile->ve, cycle_tp->plan_block->max_vel,cycle_tp->pos_cmd_data.vel_profile->vpeak, cycle_tp->plan_block->max_acc,  cycle_tp->plan_block->max_jerk, cycle_tp->plan_block->length);
        LOG(LOG_LEVEL_DEBUG, "target %f, s %f, accum_distance %f, t_cycle %f, t[0] %f, ", cycle_tp->pos_cmd_data.target_distance, cycle_tp->pos_cmd_data.traj_data->s, cycle_tp->pos_cmd_data.accum_distance, cycle_tp->pos_cmd_data.vel_profile->t_cycle, cycle_tp->pos_cmd_data.vel_profile->t[0]);
        return;
    }
    cycle_tp->pos_cmd_data.accum_distance = cycle_tp->pos_cmd_data.traj_data->s;
    cycle_tp->pos_cmd_data.remain_distance = cycle_tp->plan_block->length - cycle_tp->pos_cmd_data.accum_distance;
    // 计算插补位置
    POS interp_pos = auto_cal_interp_pos(cycle_tp->pos_cmd_data.current_pos, cycle_tp->pos_cmd_data.accum_distance);
    // 更新运动数据
    cycle_tp->pos_cmd_data.current_pos.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = interp_pos.x;
    cycle_tp->pos_cmd_data.current_pos.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur = interp_pos.y;
    cycle_tp->pos_cmd_data.current_pos.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur = interp_pos.z;
    cycle_tp->pos_cmd_data.current_vel = cycle_tp->pos_cmd_data.traj_data->v;
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, cycle_tp->pos_cmd_data.traj_data->v * TIME_MINUS_2_S, 0);
}

static bool auto_pos_cmd_cycle_is_finished(){
    return cycle_tp->pos_cmd_data.interp_cycle >= cycle_tp->pos_cmd_data.vel_profile->t_cycle;
}

bool auto_pos_cmd_is_finished(){
    if(auto_pos_cmd_arrive_check()) {
        switch(cycle_tp->target_stage){
            case BLOCK_PAUSED:
            case BLOCK_BREAKPOINT:
                cycle_tp_update_block_data();
                break;
            default:
                break;
        }
        cycle_tp_reset_pos_cmd_data();
        return true;
    }
    return false;
}

void auto_update_dwell_cmd(){
    cycle_tp->exec_cmd = DO_DWELL_CMD;
    cycle_tp->plan_block->pause_time -= cycle_tp->dwell_cmd_data.interp_cycle;
    cycle_tp->dwell_cmd_data.interp_cycle = 0;
    switch (cycle_tp->exec_state)
    {
    case EXEC_RUNNING:
        cycle_tp->dwell_cmd_data.dwell_cycle = cycle_tp->plan_block->pause_time;
        cycle_tp->target_stage = BLOCK_STOP_2_END;
        break;
    case EXEC_PAUSING:
    case EXEC_RESET:
        cycle_tp->dwell_cmd_data.dwell_cycle = 0;
        cycle_tp->target_stage = BLOCK_PAUSED;
    default:
        break;
    }
    // LOG(LOG_LEVEL_DEBUG, "dwell_cycle: %d", cycle_tp->dwell_cmd_data.dwell_cycle);
}

void auto_get_dwell_cmd(){
    cycle_tp->exec_cmd = DO_DWELL_CMD;
    cycle_tp->dwell_cmd_data.dwell_cycle = cycle_tp->plan_block->pause_time;
    cycle_tp->dwell_cmd_data.interp_cycle = 0;
    cycle_tp->target_stage = BLOCK_STOP_2_END;
}

void auto_execute_dwell_cmd(){
    cycle_tp->dwell_cmd_data.interp_cycle += 1;
}

bool auto_dwell_cmd_is_finished(){
    if(cycle_tp->dwell_cmd_data.interp_cycle >= cycle_tp->dwell_cmd_data.dwell_cycle){
        cycle_tp_reset_dwell_cmd_data();
        return true;
    }
    return false;
}

void auto_get_modal_cmd(){
    cycle_tp->exec_cmd = DO_NULL_CMD;
    cycle_tp->target_stage = BLOCK_STOP_2_END;
}

void cycle_tp_add_block(){
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* plan_block;
    // 拷贝 pretreat_queue 的数据
    for(int i = 0; i < LOOK_AHEAD_SIZE; i++){
        // 如果前瞻队列已满，提前结束
        if(queue_is_full(plan_queue)) break;
        // 如果预处理队列无数据，提前结束
        if(queue_is_empty(pretreat_queue)) break;
        pretreat_block = queue_peak(pretreat_queue);
        queue_push(plan_queue, pretreat_block);
        queue_pop(pretreat_queue);
    }
}

static bool is_newval_unvalid(){
    double vs = cycle_tp->pos_cmd_data.current_vel;
    double ve = cycle_tp->plan_block->end_vel;
    double target_vel = cycle_tp->pos_cmd_data.target_vel;
    int velplan_mode = cycle_tp->velplan_mode;
    double remain_s = cycle_tp->plan_block->length - cycle_tp->pos_cmd_data.accum_distance;
    double total_decline_length = 0;
    if(IsGreaterThan(vs, target_vel) && IsGreaterThanOrEqual(target_vel, ve)){
        total_decline_length = vp_cal_decel_dist(velplan_mode, vs, target_vel, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk)
        + vp_cal_decel_dist(velplan_mode, target_vel, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    }else if((IsGreaterThanOrEqual(target_vel, vs) && IsGreaterThanOrEqual(vs, ve)) || (IsGreaterThanOrEqual(vs, ve) && IsGreaterThan(ve, target_vel))){
        total_decline_length = vp_cal_decel_dist(velplan_mode, vs, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    } 
    if(IsGreaterThanOrEqual(total_decline_length, remain_s)) return true;
    return false;
}

static void auto_cal_target_vel(){
    switch (cycle_tp->exec_state)
    {
    case EXEC_RUNNING:
        switch (cycle_tp->plan_block->type)
        {
        case RAPID:
            if(rt_get_sys_state(SYS_IS_DRY_RUN)) cycle_tp->pos_cmd_data.target_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);
            else cycle_tp->pos_cmd_data.target_vel = MIN(cycle_tp->plan_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);
            break;
        default:
            if(rt_get_sys_state(SYS_IS_DRY_RUN)) cycle_tp->pos_cmd_data.target_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            else cycle_tp->pos_cmd_data.target_vel = MIN(cycle_tp->plan_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            break;
        }
        if(!rt_data->sys_state[SYS_IS_MAC_LOCK_READY] && rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) cycle_tp->pos_cmd_data.target_vel = 0;
        if(rt_data->sys_state[SYS_IS_MAC_LOCK_READY] && !rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) cycle_tp->pos_cmd_data.target_vel = 0;
        break;
    case EXEC_PAUSING:
    case EXEC_RESET:
    case EXEC_IDLE:
    default:
        cycle_tp->pos_cmd_data.target_vel = 0;
        break;
    }
}

static void auto_look_ahead(){
    double vs = cycle_tp->pos_cmd_data.current_vel;
    double ve = cycle_tp->plan_block->end_vel;
    double target_vel = cycle_tp->pos_cmd_data.target_vel;
    int velplan_mode = cycle_tp->velplan_mode;
    double ncu_cycle = cycle_tp->ncu_cycle;
    PLAN_BLOCK* cur_block;
    PLAN_BLOCK* next_block;
    for(int i = 0; i < plan_queue->size; i++){
        cur_block = queue_get_block(plan_queue, i);
        block_cal_lookahead_vel(cur_block);
    }
    // 修正末段ve
    cur_block->end_vel = 0.0;
    for(int i = 0; i < plan_queue->size - 1; i++){
        cur_block = queue_get_block(plan_queue, i);
        next_block = queue_get_block(plan_queue, i + 1);
        cur_block->end_vel = next_block->start_vel = MIN(cur_block->end_vel, next_block->start_vel);
    }
    // 修正首段vs
    cur_block = queue_peak(plan_queue);
    cur_block->start_vel = vs;
    // 判断是否大于最大速度
    if(cur_block->start_vel > cur_block->max_vel) cur_block->max_vel = cur_block->start_vel;
    // 速度前瞻
    vp_cal_bid_lookahead(velplan_mode, plan_queue, ncu_cycle, par_axis[0][PAR_AX_CORNER_VEL_DIFF].value.val_real, par_axis[1][PAR_AX_CORNER_VEL_DIFF].value.val_real, par_axis[2][PAR_AX_CORNER_VEL_DIFF].value.val_real);
    // if(cur_block->type == CIRCLE_CW || cur_block->type == CIRCLE_CCW){
    //     LOG(LOG_LEVEL_DEBUG, "速度前瞻 %d, angle:%f, length:%f, radius:%f, max_vel:%f, start_vel:%f end_vel:%f", cur_block->line_number, cur_block->angle_rad, cur_block->length, cur_block->circular_data.radius, cur_block->max_vel, cur_block->start_vel, cur_block->end_vel);
    // }else{
    //     LOG(LOG_LEVEL_DEBUG, "速度前瞻 %d, angle:%f, length:%f, max_vel:%f, start_vel:%f end_vel:%f", cur_block->line_number, cur_block->angle_rad, cur_block->length, cur_block->max_vel, cur_block->start_vel, cur_block->end_vel);
    // }
    // 如果无法减速到ve
    if(IsGreaterThan(vs, 0)){
        double del_dist = vp_cal_decel_dist(velplan_mode, cur_block->start_vel, cur_block->end_vel, cur_block->max_acc, cur_block->max_jerk);
        if(IsGreaterThan(del_dist, cur_block->length)) {
            double del_vel = vp_cal_decel_vel(velplan_mode, cur_block->start_vel, cur_block->max_acc, cur_block->max_jerk, cur_block->length);
            cur_block->end_vel = del_vel;
        }
    }
    // 更新cycle_tp速度
    cycle_tp->plan_block->start_vel = cur_block->start_vel;
    cycle_tp->plan_block->end_vel = cur_block->end_vel;
    cycle_tp->plan_block->max_vel = cur_block->max_vel;
}

static GNC_RES auto_do_vel_plan(){
    GNC_RES res = VP_CAL_SUCCESS;
    cycle_tp_reset_pos_cmd_data();
    cycle_tp->pos_cmd_data.remain_distance = cycle_tp->plan_block->length;
    if(IsGreaterThan(cycle_tp->pos_cmd_data.current_vel, cycle_tp->pos_cmd_data.target_vel)){
        res = vp_val_decline_profile();
    }else{
        res = vp_cal_vel_profile(cycle_tp->velplan_mode, cycle_tp->plan_block, cycle_tp->pos_cmd_data.vel_profile, cycle_tp->ncu_cycle); // 计算速度曲线
        if(IsEqualTo(cycle_tp->pos_cmd_data.vel_profile->ve, 0)) cycle_tp->target_stage = BLOCK_STOP_2_END;
        else cycle_tp->target_stage = BLOCK_ARRIVE_2_END;
    }
    if(res != VP_CAL_SUCCESS){
        LOG(LOG_LEVEL_ERR, "自动模式报警 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 2);
        cycle_tp->target_stage = PROGRAM_END;
        cycle_tp->exec_cmd = DO_NULL_CMD;
    }
    return res;
}

static void cycle_tp_update_block_data(){
    // 更新起点位置
    cycle_tp->plan_block->start_pos.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    cycle_tp->plan_block->start_pos.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    cycle_tp->plan_block->start_pos.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
    switch (cycle_tp->plan_block->type)
    {
    case HELIX_CCW:
    case HELIX_CW:
        // 更新导程
        cycle_tp->plan_block->helical_data.screw_offset -= cycle_tp->pos_cmd_data.accum_distance * sin(cycle_tp->plan_block->helical_data.screw_rad);
        break;
    default:
        break;
    }
    block_cal_vector(cycle_tp->plan_block);
    block_cal_length(cycle_tp->plan_block);
}

static void cycle_tp_update_pos_cmd_data(){
    cycle_tp->pos_cmd_data.current_vel = var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f / TIME_MINUS_2_S;
    cycle_tp->pos_cmd_data.current_pos.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    cycle_tp->pos_cmd_data.current_pos.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    cycle_tp->pos_cmd_data.current_pos.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
}

void cycle_tp_reset_pos_cmd_data(){
    if(cycle_tp == NULL || cycle_tp->plan_block == NULL) return;
    if (cycle_tp->pos_cmd_data.vel_profile != NULL) {
        memset(cycle_tp->pos_cmd_data.vel_profile, 0, sizeof(VEL_PROFILE));
    }
    if (cycle_tp->pos_cmd_data.traj_data != NULL) {
        memset(cycle_tp->pos_cmd_data.traj_data, 0, sizeof(TRAJ_PARAMS));
    }
    cycle_tp->pos_cmd_data.interp_cycle = 0;
    cycle_tp->pos_cmd_data.accum_distance = 0.0;
    cycle_tp->pos_cmd_data.remain_distance = 0.0;
    cycle_tp->pos_cmd_data.current_distance = 0.0;
    cycle_tp->pos_cmd_data.target_distance = 0.0;
}

void cycle_tp_reset_dwell_cmd_data(){
    if(cycle_tp == NULL || cycle_tp->plan_block == NULL) return;
    cycle_tp->dwell_cmd_data.dwell_cycle = 0;
    cycle_tp->dwell_cmd_data.interp_cycle = 0;
}

static GNC_RES vp_val_decline_profile(){
    GNC_RES res = VP_CAL_SUCCESS;
    double remain_s = cycle_tp->plan_block->length;
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    int velplan_mode = cycle_tp->velplan_mode;
    double ncu_cycle = cycle_tp->ncu_cycle;
    double vs = cycle_tp->pos_cmd_data.current_vel;
    double ve = cycle_tp->pos_cmd_data.target_vel;
    // 计算减速距离
    double decline_s = vp_cal_decel_dist(velplan_mode, vs, ve, amax, jmax);
    if(IsLessThanOrEqual(decline_s, 0)){ // 当减速距离非常小
        cycle_tp->pos_cmd_data.vel_profile->t_cycle = 0;
        cycle_tp->pos_cmd_data.traj_data->v = ve;
        if(IsEqualTo(ve, 0)) cycle_tp->target_stage = BLOCK_PAUSED;
        else cycle_tp->target_stage = BLOCK_BREAKPOINT;
        LOG(LOG_LEVEL_DEBUG, "减速距离极小:%f,当前速度:%f,目标速度:%f", decline_s, vs, ve);
        return VP_CAL_SUCCESS;
    }
    if (IsGreaterThan(decline_s, remain_s)){ 
        // 在当前段无法减速到ve 
        cycle_tp->plan_block->start_vel = vs; 
        cycle_tp->plan_block->end_vel = vp_cal_decel_vel(velplan_mode, vs, amax, jmax, remain_s);  
        if(IsLessThan(cycle_tp->plan_block->end_vel, 0)) {
            setAlarm(ALARM_CH, ALARM_ERROR, 2);
            cycle_tp->target_stage = PROGRAM_END;
            cycle_tp->exec_cmd = DO_NULL_CMD;
            LOG(LOG_LEVEL_DEBUG, "VP_CAL_ERROR: end_vel %f", cycle_tp->plan_block->end_vel);
            return VP_CAL_ERROR; 
        } 
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = remain_s;
        // LOG(LOG_LEVEL_DEBUG, "length %f", cycle_tp->plan_block->length);
        res = vp_cal_vel_profile(velplan_mode, cycle_tp->plan_block, cycle_tp->pos_cmd_data.vel_profile, ncu_cycle);
        cycle_tp->target_stage = BLOCK_ARRIVE_2_END;
    }
    else if(IsEqualTo(decline_s, remain_s))
    {
        // 在当前段末可以减速到ve
        cycle_tp->plan_block->start_vel = vs;
        cycle_tp->plan_block->end_vel = ve;
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = decline_s;
        // LOG(LOG_LEVEL_DEBUG, "vs %f ve %f length %f", vs, ve, cycle_tp->plan_block->length);
        res = vp_cal_vel_profile(velplan_mode, cycle_tp->plan_block, cycle_tp->pos_cmd_data.vel_profile, ncu_cycle);
        if(IsEqualTo(ve, 0)) cycle_tp->target_stage = BLOCK_STOP_2_END;
        else cycle_tp->target_stage = BLOCK_ARRIVE_2_END;
    }
    else{
        // 在当前段可以减速到ve
        cycle_tp->plan_block->start_vel = vs;
        cycle_tp->plan_block->end_vel = ve;
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = decline_s;
        // LOG(LOG_LEVEL_DEBUG, "vs %f ve %f length %f", vs, ve, cycle_tp->plan_block->length);
        res = vp_cal_vel_profile(velplan_mode, cycle_tp->plan_block, cycle_tp->pos_cmd_data.vel_profile, ncu_cycle);
        if(IsEqualTo(ve, 0)) cycle_tp->target_stage = BLOCK_PAUSED;
        else cycle_tp->target_stage = BLOCK_BREAKPOINT;
    }
    return res;
}

// 计算段的首末速度、最大速度、加速度、加加速度
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block){
    switch (current_block->mode)
    {
        // 切削方式 G64
        case PMC_MODE_CHOP:
            block_cal_chop_vel(current_block);
            break;
        // 攻丝方式 G63、G83、G84
        case PMC_MODE_TAP:
        case PMC_MODE_TAP_RETURE:
            block_cal_tap_vel(current_block);
            break;
        // 准确停止、到位检测 G09、G61、G27、G28
        case PMC_MODE_ARRIVE_CHECK:
        case PMC_MODE_EXACT_STOP:
            block_cal_exact_stop_vel(current_block);
            break;
        default:
            block_cal_default_vel(current_block);
            break;
    }
}

static void block_cal_default_vel(PLAN_BLOCK* current_block){
    current_block->max_vel = current_block->start_vel = current_block->end_vel = 0;
}

// 精准停：终点减速，到位检测
static void block_cal_exact_stop_vel(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
            // 当参数RDR（No.1401#6）的值为1时为空运行速度×JV，其值为0时则为快速移动速度
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);        
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;

        case LINE:
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            // 被钳制在最大切削进给速度上
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;

        default:
            break;
    }
}

// 攻丝方式：终点不减速执行下一程序段，进给倍率和进给暂停无效
static void block_cal_tap_vel(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real, cycle_tp->max_rapid_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel, cycle_tp->max_rapid_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;

        case LINE:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real, cycle_tp->max_feed_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel, cycle_tp->max_feed_vel);
            }
            if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK)) current_block->start_vel = current_block->end_vel = 0;
            else current_block->start_vel = current_block->end_vel = current_block->max_vel;    
            break;
        
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real, cycle_tp->max_feed_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel, cycle_tp->max_feed_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;
        default:
            break;
    }
}

// 切削方式：终点不减速执行下一程序段
static void block_cal_chop_vel(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_rapid_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;
        
        case LINE:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK)) current_block->start_vel = current_block->end_vel = 0;
            else current_block->start_vel = current_block->end_vel = current_block->max_vel;  
            break;
        
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = MIN(par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, cycle_tp->max_feed_vel);
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;
        default:
            break;
    }
}

static bool auto_pos_cmd_axis_arrive_check(){
    int res = 0;
    bool is_arrived = true;
    if((fabs(rt_data->axis_data[AXIS_X].mac_cmd_pos - rt_data->axis_data[AXIS_X].fb_pos_no_comp)) > par_axis[AXIS_X][PAR_AX_POS_TOL].value.val_real) {
        is_arrived = false;
        res = setAlarm(ALARM_AX, ALARM_WARNING, 103);
        if (res != -8) LOG(LOG_LEVEL_DEBUG, "X轴终点%f,实际偏差%f超过目标%f允差,等待到位...", cycle_tp->plan_block->end_pos.x, fabs(cycle_tp->plan_block->end_pos.x - rt_data->axis_data[AXIS_X].fb_pos_no_comp), par_axis[AXIS_X][PAR_AX_POS_TOL].value.val_real);
    }
    if((fabs(rt_data->axis_data[AXIS_Y].mac_cmd_pos - rt_data->axis_data[AXIS_Y].fb_pos_no_comp)) > par_axis[AXIS_Y][PAR_AX_POS_TOL].value.val_real) {
        is_arrived = false;
        res = setAlarm(ALARM_AX, ALARM_WARNING, 104);
        if (res != -8) LOG(LOG_LEVEL_DEBUG, "Y轴终点%f,实际偏差%f超过目标%f允差,等待到位...", cycle_tp->plan_block->end_pos.y, fabs(cycle_tp->plan_block->end_pos.y - rt_data->axis_data[AXIS_Y].fb_pos_no_comp), par_axis[AXIS_Y][PAR_AX_POS_TOL].value.val_real);
    }
    if((fabs(rt_data->axis_data[AXIS_Z].mac_cmd_pos - rt_data->axis_data[AXIS_Z].fb_pos_no_comp)) > par_axis[AXIS_Z][PAR_AX_POS_TOL].value.val_real) {
        is_arrived = false;
        res = setAlarm(ALARM_AX, ALARM_WARNING, 105);
        if (res != -8) LOG(LOG_LEVEL_DEBUG, "Z轴终点%f,实际偏差%f超过目标%f允差,等待到位...", cycle_tp->plan_block->end_pos.z, fabs(cycle_tp->plan_block->end_pos.z - rt_data->axis_data[AXIS_Z].fb_pos_no_comp), par_axis[AXIS_Z][PAR_AX_POS_TOL].value.val_real);
    }
    if(is_arrived){
        delAlarm(ALARM_AX, ALARM_WARNING, 103);
        delAlarm(ALARM_AX, ALARM_WARNING, 104);
        delAlarm(ALARM_AX, ALARM_WARNING, 105);
        return true;
    }else{
        return false;
    }
}

static bool auto_pos_cmd_arrive_check(){
    if(!auto_pos_cmd_cycle_is_finished()) return false;
    // 机械锁住时不进行到位检测
    if(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]) return true;
    switch (cycle_tp->target_stage)
    {
    case BLOCK_STOP_2_END:
        // 如果是到位检测模组，发送到位信号
        switch (cycle_tp->plan_block->mode) {
        case PMC_MODE_ARRIVE_CHECK:
        case PMC_MODE_EXACT_STOP:
            if(!auto_pos_cmd_axis_arrive_check()) return false;
            LOG(LOG_LEVEL_DEBUG,  "[到位检测] 机床指令位置:%f %f %f vel:%f", rt_data->axis_data[AXIS_X].motor_cmd_pos, rt_data->axis_data[AXIS_Y].motor_cmd_pos, rt_data->axis_data[AXIS_Z].motor_cmd_pos, cycle_tp->plan_block->end_vel);
            LOG(LOG_LEVEL_DEBUG,  "[到位检测] 电机无补偿位置:%f %f %f vel:%f", rt_data->axis_data[AXIS_X].fb_pos_no_comp, rt_data->axis_data[AXIS_Y].fb_pos_no_comp, rt_data->axis_data[AXIS_Z].fb_pos_no_comp,cycle_tp->plan_block->end_vel);
            LOG(LOG_LEVEL_DEBUG,  "[到位检测] 电机反馈位置:%f %f %f vel:%f", rt_data->axis_data[AXIS_X].motor_act_pos, rt_data->axis_data[AXIS_Y].motor_act_pos, rt_data->axis_data[AXIS_Z].motor_act_pos,cycle_tp->plan_block->end_vel);
            return true;
        case PMC_MODE_CHOP:
            // 判断G00到位宽度
            if(cycle_tp->plan_block->type == RAPID){
                return auto_pos_cmd_axis_arrive_check();
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    return true;
}
