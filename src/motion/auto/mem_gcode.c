#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "mem_gcode.h"
#include "global.h"
#include "datatype.h"
#include "pathplan.h"
#include <hmisysapi.h>
#include "varapi.h"
#include "interpolation.h"
#include "stored_stroke_check.h"
#include <alarmapi.h>

static bool mem_check_res(GNC_RES res);
static bool mem_check_running();
static double mem_cal_pinc(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos);
static bool cycle_tp_is_finished();
static void mem_cal_axis_vel(POS *current_pos, POS *target_pos, double interp_pinc, double target_vel);

static GNC_RES cycle_tp_update_pausing_tp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos, double target_vel);
static GNC_RES cycle_tp_update_running_tp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos);
static GNC_RES cycle_tp_update_params(VELPLAN_TYPE velplan_type, double cycle_time);
static bool cycle_block_is_arrived(POS current_pos);
static bool cycle_tp_is_newvel_unvalid(VELPLAN_TYPE vplan_mode, double current_vel, double target_vel);
static GNC_RES cycle_tp_update_block(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos);
static GNC_RES cycle_tp_update_vp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, double target_vel);
static GNC_RES plan_block_get_next();
static GNC_RES plan_block_get_update(double current_vel, POS current_pos);
static GNC_RES cycle_tp_cal_running(VELPLAN_TYPE vplan_mode, double ncu_cycle);
static GNC_RES cycle_tp_cal_pausing(VELPLAN_TYPE vplan_mode, double ncu_cycle, double vs, double ve);
static GNC_RES cycle_tp_add_block();
static GNC_RES cycle_tp_look_ahead(VELPLAN_TYPE vplan_mode, double ncu_cycle, double vs);
static bool cycle_block_axis_arrive_check(PLAN_BLOCK* current_block);
static bool cycle_block_arrive_check();
static bool cycle_tp_is_zerovel_and_zerorate(double current_vel);
static bool cycle_tp_check_switching_dist(VELPLAN_TYPE vplan_mode, double current_vel, double target_vel);
static void block_cal_exact_stop_vel(PLAN_BLOCK* current_block);
static void block_cal_tap_vel(PLAN_BLOCK* current_block);
static void block_cal_chop_vel(PLAN_BLOCK* current_block);
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block);
// todo：修改位置
static GNC_RES vp_cal_g04_vel_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time);
static GNC_RES vp_cal_g04_pause_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time);
static void check_traj_data_valid(TRAJ_PARAMS* traj_data);

// G指令执行流程：处理开始和暂停 → 计算插补距离 → 计算插补空间坐标
void auto_execute_gcode(){
    double current_vel = var_chan[0][VAR_CHAN_CMD_FEEDRATE].value.f / TIME_MINUS_2_S;
    POS current_pos = {.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, .y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, .z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur};
    int velplan_mode = par_chan[0][PAR_CH_VPLAN_MODE].value.val_int;
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
    double interp_pinc = 0;
    POS interp_pos = current_pos;
    // 计算插补距离
    interp_pinc = mem_cal_pinc(velplan_mode, ncu_cycle, current_vel, current_pos);
    // 根据插补类型计算插补点：快速G00、直线G01、圆弧G02|03……
    interp_pos = mem_cal_pos(current_pos, interp_pinc);
    // 更新速度参数
    rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur = interp_pos.x;
    rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur = interp_pos.y;
    rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur = interp_pos.z;
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, cycle_tp->traj_data->v * TIME_MINUS_2_S, 0);
    rt_data->chan_data[0].cmd_acc = cycle_tp->traj_data->a;
    rt_data->chan_data[0].cmd_jk = cycle_tp->traj_data->j;
    mem_cal_axis_vel(&current_pos, &interp_pos, interp_pinc, cycle_tp->traj_data->v);
}

// 计算插补距离
static double mem_cal_pinc(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos){
    double interp_distance = 0;
    GNC_RES res = GNC_CAL_SUCCESS;
    // 初始化默认为 IDLE
    switch(cycle_tp->mem_exec_state){
        case EXEC_IDLE:
            break;

        case EXEC_RUNNING:
            // 完成一段的插补
            if (cycle_tp_is_finished() || cycle_tp->is_event_trigger){
                // 如果单段使能
                // if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK) && cycle_tp->is_ve_zero){
                //     cycle_tp->mem_exec_state = EXEC_PAUSING;
                //     cycle_tp->is_ve_zero = false;
                //     LOG(LOG_LEVEL_DEBUG, "单段下暂停");
                //     break;
                // }
                res = cycle_tp_update_running_tp(velplan_mode, ncu_cycle, current_vel, current_pos);
                CHECK_RES(mem_check_res(res));
                cycle_tp->is_event_trigger = false; 
            }
            res = cycle_tp_update_params(velplan_mode, ncu_cycle);
            CHECK_RES(mem_check_res(res));
            interp_distance = cycle_tp->accum_distance;
            break;

        // 执行暂停中
        case EXEC_PAUSING:
            // 检查是否完成该段的减速运动
            if (cycle_tp_is_finished() || cycle_tp->is_event_trigger){ 
                // 如果段内完成暂停         
                if(cycle_tp_is_finished() && cycle_tp->is_paused_inside_block){                     
                    // 复位
                    if(cycle_tp->is_resetting){
                        gcode_handle_reset();
                        // setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0); 
                        auto_check_soft_limit_del_done();
                    }else{
                        setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);               
                    }                             
                    break;
                }else{ 
                    res = cycle_tp_update_pausing_tp(velplan_mode, ncu_cycle, current_vel, current_pos, 0);
                    CHECK_RES(mem_check_res(res));
                }
                cycle_tp->is_event_trigger = false; 
            }
            res = cycle_tp_update_params(velplan_mode, ncu_cycle);
            CHECK_RES(mem_check_res(res));
            interp_distance = cycle_tp->accum_distance;
            break;

        default:
            break;
    }
    check_traj_data_valid(cycle_tp->traj_data);
    return interp_distance;
}

static bool mem_check_res(GNC_RES res){
    switch (res)
    {
    case GNC_CAL_SUCCESS:
    case GNC_OPERATOR_SUCCESS:
    case VP_CAL_SUCCESS:
    case VP_DELCINE_TO_END:
        return true;
    
    case VP_MAX_VEL_IS_ZERO:
        if(cycle_tp->is_resetting){
            gcode_handle_reset();
            return false;
        }
        // 如果进给倍率为0
        if(cycle_tp->mem_exec_state == EXEC_RUNNING){
            cycle_tp->plan_block->type = UNKNOWN;
            memset(cycle_tp->traj_data, 0, sizeof(TRAJ_PARAMS));
            return false;
        }
        break;

    case GNC_DATA_EMPTY:
        if(cycle_tp->is_resetting){
            gcode_handle_reset();
            return false;
        }
        // 运行中无数据，则进入等待 
        if(mem_check_running()){
            if(queue_is_empty(pretreat_queue) && gcode_queue_is_empty(gcode_queue)){
                // if(!rt_get_chan_state(0, CHAN_IS_GCODE_END)) LOG(LOG_LEVEL_DEBUG, "gcode is empty");
                rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
                cycle_tp->is_arrive_at_the_end = false;
            }
            return false;
        }
        return false;

    case VP_LENGTH_IS_ZERO:
        return false;

    case GNC_DATA_NULL:
    case GNC_PARAM_UNVALID:
    case VP_IS_NULL:
    case VP_PARAM_UNVALID:
    case VP_FEED_RATE_IS_ZERO:
    case VP_LENGTH_UNVALID:
        gcode_handle_estop();
        LOG(LOG_LEVEL_ERR, "自动模式报警 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 2);
        return false;

    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    case GNC_ARRIVE_CHECK_FAIL:
    case VP_CAL_ERROR:
    case VP_CAL_S_ERROR:
    case VP_CAL_V_ERROR:
    default:
        gcode_handle_estop();
        LOG(LOG_LEVEL_ERR, "自动模式报警 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 3);
        return false;
    }
}

void gcode_handle_estop(){
    cycle_tp_reset();
    queue_reset(plan_queue);
    queue_reset(pretreat_queue);
    gcode_queue_reset(gcode_queue);
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);   
    rt_set_chan_state(0, CHAN_IS_PARSERING, false);
    rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
}

void gcode_handle_reset(){
    cycle_tp_reset();
    queue_reset(plan_queue);
    queue_reset(pretreat_queue);
    gcode_queue_reset(gcode_queue);
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);  
    rt_set_chan_state(0, CHAN_IS_PARSERING, false);
    rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
}

static bool mem_check_running(){
    if(rt_get_chan_state(0, CHAN_IS_PARSERING)) return true;
    return (!queue_is_empty(plan_queue) || !queue_is_empty(pretreat_queue) || !gcode_queue_is_empty(gcode_queue));
}

// 检查是否执行完速度曲线
static bool cycle_tp_is_finished(){
    if(cycle_tp->interp_cycle > cycle_tp->vel_profile->t_cycle){
        cycle_tp->plan_block->type = UNKNOWN;
        PLAN_BLOCK* head_block = queue_peak(plan_queue);
        head_block->pause_time -= cycle_tp->vel_profile->t_cycle;
        cycle_tp->interp_cycle = 1;
        cycle_tp->vel_profile->t_cycle = 0;
        cycle_tp->current_distance = 0;
        cycle_tp->target_distance = 0;
        cycle_tp->accum_distance = 0;
        cycle_tp->remain_distance = 0;
        return true;
    }
    return false;
}

void cycle_tp_init(){
    cycle_tp = (COORD_TP*)calloc(sizeof(COORD_TP), 1);
    cycle_tp->plan_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
    cycle_tp->vel_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
    cycle_tp->traj_data = (TRAJ_PARAMS*)calloc(sizeof(TRAJ_PARAMS), 1);
    cycle_tp->plan_block->type = UNKNOWN;
    cycle_tp->is_event_trigger = false;
    cycle_tp->interp_cycle = 1;
    cycle_tp->is_paused_inside_block = true;
}

void cycle_tp_reset(){
    cycle_tp->mem_exec_state = EXEC_IDLE;
    cycle_tp->plan_block->type = UNKNOWN;
    memset(cycle_tp->traj_data, 0, sizeof(TRAJ_PARAMS));
    cycle_tp->is_event_trigger = false;
    cycle_tp->is_paused_inside_block = true;
    cycle_tp->is_arrive_at_the_end = false;
    cycle_tp->is_resetting = 0;
    cycle_tp->interp_cycle = 1;
    cycle_tp->vel_profile->t_cycle = 0;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    cycle_tp->accum_distance = 0;
    cycle_tp->remain_distance = 0;
}

static GNC_RES cycle_tp_update_pausing_tp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos, double target_vel){
    GNC_RES res = GNC_CAL_SUCCESS;
    if(cycle_tp_add_block() == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;   // 如果没有插补数据
    // 如果新速度不满足，不更新新速度
    if(cycle_tp->is_event_trigger){
        if(cycle_tp_is_newvel_unvalid(vplan_mode, current_vel, target_vel)){
            if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_paused_inside_block = true;
            else cycle_tp->is_paused_inside_block = false;
            LOG(LOG_LEVEL_DEBUG, "在%d程序段无法满足目标速度%f, 按原速度曲线减速到%f", cycle_tp->plan_block->line_number, target_vel, cycle_tp->plan_block->end_vel);
            return VP_DELCINE_TO_END;
        }
        else cycle_tp->is_arrive_at_the_end = false;
    }
    res = cycle_tp_update_block(vplan_mode, ncu_cycle, current_vel, current_pos);
    if(res != GNC_CAL_SUCCESS) return res;
    res = cycle_tp_update_vp(vplan_mode, ncu_cycle, current_vel, target_vel);
    if(res != VP_CAL_SUCCESS) return res;
    return res;
}

static GNC_RES cycle_tp_update_running_tp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos){
    GNC_RES res = GNC_CAL_SUCCESS;
    if(cycle_tp_add_block() == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;   // 如果没有插补数据
    // 如果速度为0且倍率为0
    if(cycle_tp_is_zerovel_and_zerorate(current_vel)) return VP_MAX_VEL_IS_ZERO;
    // 如果新速度不满足，不更新新速度
    if(cycle_tp->is_event_trigger){
        if(cycle_tp_is_newvel_unvalid(vplan_mode, current_vel, cycle_tp->max_vel)){
            if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_paused_inside_block = true;
            else cycle_tp->is_paused_inside_block = false;
            LOG(LOG_LEVEL_DEBUG, "在%d程序段无法满足目标速度%f, 按原速度曲线减速到%f", cycle_tp->plan_block->line_number, cycle_tp->max_vel, cycle_tp->plan_block->end_vel);
            return VP_DELCINE_TO_END;
        }
        else {
            // 如果加速或减速的距离很小，当前速度 = 目标速度
            if(cycle_tp_check_switching_dist(vplan_mode, current_vel, cycle_tp->max_vel)) current_vel = cycle_tp->max_vel;
            cycle_tp->is_arrive_at_the_end = false;
        }
    }
    res = cycle_tp_update_block(vplan_mode, ncu_cycle, current_vel, current_pos);
    if(res != GNC_CAL_SUCCESS) return res;
    // 如果加工段允许最大速度为0（倍率为0）
    if(IsEqualTo(cycle_tp->plan_block->max_vel, 0) && cycle_tp->plan_block->mode != PMC_MODE_PAUSE) return VP_MAX_VEL_IS_ZERO;
    res = cycle_tp_update_vp(vplan_mode, ncu_cycle, current_vel, cycle_tp->max_vel);
    if(res != VP_CAL_SUCCESS) return res;
    return res;
}

static bool cycle_tp_check_switching_dist(VELPLAN_TYPE vplan_mode, double current_vel, double target_vel){
    double remain_s = cycle_tp->plan_block->length;
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    double decline_s = vp_cal_decel_dist(vplan_mode, current_vel, target_vel, amax, jmax);
    if(IsLessThanOrEqual(fabs(decline_s), 0)){ // 当减速距离非常小
        LOG(LOG_LEVEL_DEBUG, "变速距离极小:%f,当前速度:%f,目标速度:%f", decline_s, current_vel, target_vel);
        return true;
    }
    return false;
}

static GNC_RES cycle_tp_update_params(VELPLAN_TYPE velplan_type, double cycle_time){
    if(cycle_tp->plan_block->mode == PMC_MODE_PAUSE){
        memset(cycle_tp->traj_data, 0, sizeof(TRAJ_PARAMS));
        cycle_tp->current_distance = 0;
        cycle_tp->target_distance = 0;
        cycle_tp->accum_distance = 0;
        cycle_tp->remain_distance = 0;
    }else{
        vp_cal_traj_data(velplan_type, cycle_tp->vel_profile, cycle_tp->interp_cycle, cycle_tp->traj_data, cycle_time);
        cycle_tp->current_distance = cycle_tp->target_distance;
        cycle_tp->target_distance = cycle_tp->traj_data->s - cycle_tp->accum_distance;
        cycle_tp->accum_distance = cycle_tp->traj_data->s;
        cycle_tp->remain_distance = cycle_tp->plan_block->length - cycle_tp->accum_distance;
    }
    cycle_tp->interp_cycle += 1;
    if(IsLessThan(cycle_tp->target_distance, 0)){
        LOG(LOG_LEVEL_DEBUG, "VP_CAL_S_ERROR: cycle_tp->target_distance %f", cycle_tp->target_distance);
        return VP_CAL_S_ERROR;
    }
    return GNC_CAL_SUCCESS;
}

static bool cycle_tp_is_zerovel_and_zerorate(double current_vel){
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    if(head_block->mode == PMC_MODE_PAUSE || head_block->mode == PMC_MODE_TAP) return false;
    if(head_block->type != RAPID && IsEqualTo(current_vel, 0) && IsEqualTo(var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f, 0)){
        cycle_tp->max_vel = 0;
        return true;
    }
    if(head_block->type == RAPID && IsEqualTo(current_vel, 0) && IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
        cycle_tp->max_vel = 0;
        return true;
    }
    // 更新最大运行速度
    if(cycle_tp->plan_block->type == RAPID){
        if(rt_get_sys_state(SYS_IS_DRY_RUN)) cycle_tp->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
        else cycle_tp->max_vel = cycle_tp->plan_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
    };
    if(cycle_tp->plan_block->type != RAPID){
        if(rt_get_sys_state(SYS_IS_DRY_RUN)) cycle_tp->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT;
        else cycle_tp->max_vel = cycle_tp->plan_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT;
    }
    return false;
}

static bool cycle_tp_is_newvel_unvalid(VELPLAN_TYPE vplan_mode, double current_vel, double target_vel){
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    if(head_block->mode == PMC_MODE_PAUSE) return false;
    if(IsEqualTo(head_block->length, 0)) return false;
    double vs = current_vel;
    double ve = head_block->end_vel;
    double remain_s = head_block->length - cycle_tp->accum_distance;
    double total_decline_length = 0;
    if(IsGreaterThan(vs, target_vel) && IsGreaterThanOrEqual(target_vel, ve)){
        total_decline_length = vp_cal_decel_dist(vplan_mode, vs, target_vel, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk)
        + vp_cal_decel_dist(vplan_mode, target_vel, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    }else if((IsGreaterThan(target_vel, vs) && IsGreaterThanOrEqual(vs, ve)) || (IsGreaterThanOrEqual(vs, ve) && IsGreaterThan(ve, target_vel))){
        total_decline_length = vp_cal_decel_dist(vplan_mode, vs, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    } 
    if(IsGreaterThanOrEqual(total_decline_length, remain_s)) return true;
    return false;
}

static GNC_RES cycle_tp_update_block(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, POS current_pos){
    GNC_RES res = GNC_CAL_SUCCESS;
    // 如果到达加工段终点
    if(cycle_block_is_arrived(current_pos)){
        if(cycle_block_arrive_check()) res = plan_block_get_next();
        else res = GNC_ARRIVE_CHECK_FAIL;
        // LOG(LOG_LEVEL_INFO, "获取下一个插补段");
    }else{
        res = plan_block_get_update(current_vel, current_pos);
        // LOG(LOG_LEVEL_INFO, "更新当前插补段");
    }
    if(res != GNC_CAL_SUCCESS) return res;
    if(cycle_tp_look_ahead(vplan_mode, ncu_cycle, current_vel) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    // 更新当前行信息
    queue_copy_block(plan_queue, 0, cycle_tp->plan_block);
    setVarChanInt(0, VAR_CHAN_RUN_ROW, cycle_tp->plan_block->line_number, 0);
    // LOG(LOG_LEVEL_INFO, "gline: %d", cycle_tp->plan_block->line_number);
    // 对插补段终点进行行程限位检查
    lookahead_check_soft_limit(&cycle_tp->plan_block->end_pos);
    return res;
}

static GNC_RES cycle_tp_update_vp(VELPLAN_TYPE vplan_mode, double ncu_cycle, double current_vel, double target_vel){
    GNC_RES res = VP_CAL_SUCCESS;
    // LOG(LOG_LEVEL_DEBUG, "current_vel:%f, target_vel:%f", current_vel, target_vel);
    if(IsGreaterThan(current_vel, target_vel)){
        res = cycle_tp_cal_pausing(vplan_mode, ncu_cycle, current_vel, target_vel);
    }else{
        res = cycle_tp_cal_running(vplan_mode, ncu_cycle);
    }
    if(res != VP_CAL_SUCCESS){
        return res;
    }
    if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_paused_inside_block = true;
    else cycle_tp->is_paused_inside_block = false;
    return res;
}

static GNC_RES plan_block_get_next(){
    PLAN_BLOCK* head_block = NULL;
    while(true){
        head_block = queue_peak(plan_queue);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, head_block->line_number, 0);
        // LOG(LOG_LEVEL_INFO, "gline: %d", head_block->line_number);
        queue_pop(plan_queue);
        if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;
        head_block = queue_peak(plan_queue);
        // 如果长度小于等于0
        if(IsGreaterThan(head_block->length, 0) || head_block->mode == PMC_MODE_PAUSE) return GNC_CAL_SUCCESS;
    }
}

static GNC_RES plan_block_get_update(double current_vel, POS current_pos){
    GNC_RES res = GNC_CAL_SUCCESS;
    // 更新段起点和速度，重新计算插补段参数
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    // 如果是g04暂停
    if(head_block->mode == PMC_MODE_PAUSE){
        if(head_block->pause_time >= cycle_tp->interp_cycle) 
            head_block->pause_time = head_block->pause_time - cycle_tp->interp_cycle + 1;
        return res;
    }
    head_block->start_pos = current_pos;
    head_block->start_vel = current_vel;
    block_cal_vector(head_block);
    block_cal_length(head_block);
    // LOG(LOG_LEVEL_INFO, "head_block->length: %f", head_block->length);
    // 如果长度小于阈值
    if(IsLessThanOrEqual(head_block->length, MEM_END_TOL)) res = plan_block_get_next();
    return res;
}  

static GNC_RES cycle_tp_cal_running(VELPLAN_TYPE vplan_mode, double ncu_cycle){
    GNC_RES res = VP_CAL_SUCCESS;
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    // LOG(LOG_LEVEL_DEBUG, "length %f", cycle_tp->plan_block->length);
    if(cycle_tp->plan_block->mode == PMC_MODE_PAUSE){
        if(cycle_tp->mem_exec_state == EXEC_PAUSING) res = vp_cal_g04_pause_profile(vplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle);
        else res = vp_cal_g04_vel_profile(vplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle);
    }else{
        res = vp_cal_vel_profile(vplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle); // 计算速度曲线
    }
    // 该段末速度为0
    if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_ve_zero = true;
    else cycle_tp->is_ve_zero = false;
    cycle_tp->is_arrive_at_the_end = true;
    return res;
}

static GNC_RES cycle_tp_cal_pausing(VELPLAN_TYPE vplan_mode, double ncu_cycle, double vs, double ve){
    GNC_RES res = VP_CAL_SUCCESS;
    double remain_s = cycle_tp->plan_block->length;
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    // 计算减速距离
    double decline_s = vp_cal_decel_dist(vplan_mode, vs, ve, amax, jmax);
    if(IsLessThanOrEqual(decline_s, 0)){ // 当减速距离非常小
        cycle_tp->vel_profile->t_cycle = 0;
        cycle_tp->traj_data->v = ve;
        cycle_tp->is_paused_inside_block = true;
        LOG(LOG_LEVEL_DEBUG, "减速距离极小:%f,当前速度:%f,目标速度:%f", decline_s, vs, ve);
        return VP_LENGTH_IS_ZERO;
    }
    if (decline_s > remain_s){ // 在当前段无法减速到ve 
        cycle_tp->plan_block->start_vel = vs; 
        cycle_tp->plan_block->end_vel = vp_cal_decel_vel(vplan_mode, vs, amax, jmax, remain_s);  
        if(IsLessThan(cycle_tp->plan_block->end_vel, 0)) {
            LOG(LOG_LEVEL_DEBUG, "VP_CAL_ERROR: end_vel %f", cycle_tp->plan_block->end_vel);
            return VP_CAL_ERROR; 
        } 
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = remain_s;
        // LOG(LOG_LEVEL_DEBUG, "length %f", cycle_tp->plan_block->length);
        res = vp_cal_vel_profile(vplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle);
        cycle_tp->is_arrive_at_the_end = true;
    }
    else{ // 在当前段可以减速到ve
        cycle_tp->plan_block->start_vel = vs;
        cycle_tp->plan_block->end_vel = ve;
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = decline_s;
        // LOG(LOG_LEVEL_DEBUG, "vs %f ve %f length %f", vs, ve, cycle_tp->plan_block->length);
        res = vp_cal_vel_profile(vplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle);
        cycle_tp->is_arrive_at_the_end = false;
    }
    return res;
}

// 检查是否到目标点
static bool cycle_block_is_arrived(POS current_pos){
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    // 如果是g04暂停
    if(head_block->mode == PMC_MODE_PAUSE){
        if(head_block->pause_time < cycle_tp->interp_cycle) return true;
        else return false;
    }
    // LOG(LOG_LEVEL_DEBUG, "is arrive: %d, line number: %d, length: %f", cycle_tp->is_arrive_at_the_end, cycle_tp->plan_block->line_number, head_block->length);
    if(!cycle_tp->is_arrive_at_the_end) return false;
    return true;
}

static bool cycle_block_axis_arrive_check(PLAN_BLOCK* current_block){
    if((fabs(current_block->end_pos.x - rt_data->axis_data[AXIS_X].motor_act_pos)) > par_axis[AXIS_X][PAR_AX_POS_TOL].value.val_real) {
        LOG(LOG_LEVEL_ERR, "X轴定位%f超过%f允差", fabs(current_block->end_pos.x - rt_data->axis_data[AXIS_X].motor_act_pos), par_axis[AXIS_X][PAR_AX_POS_TOL].value.val_real);
        setAlarm(ALARM_AX, ALARM_ERROR, 103);
        return false;
    }
    if((fabs(current_block->end_pos.y - rt_data->axis_data[AXIS_Y].motor_act_pos)) > par_axis[AXIS_Y][PAR_AX_POS_TOL].value.val_real) {
        LOG(LOG_LEVEL_ERR, "Y轴定位%f超过%f允差", fabs(current_block->end_pos.y - rt_data->axis_data[AXIS_Y].motor_act_pos), par_axis[AXIS_Y][PAR_AX_POS_TOL].value.val_real);
        setAlarm(ALARM_AX, ALARM_ERROR, 104);
        return false;
    }
    if((fabs(current_block->end_pos.z - rt_data->axis_data[AXIS_Z].motor_act_pos)) > par_axis[AXIS_Z][PAR_AX_POS_TOL].value.val_real) {
        LOG(LOG_LEVEL_ERR, "Z轴定位%f超过%f允差", fabs(current_block->end_pos.z - rt_data->axis_data[AXIS_Z].motor_act_pos), par_axis[AXIS_Z][PAR_AX_POS_TOL].value.val_real);
        setAlarm(ALARM_AX, ALARM_ERROR, 105);
        return false;
    }
    return true;
}

static bool cycle_block_arrive_check(){
    // LOG(LOG_LEVEL_DEBUG, "motor_cmd_pos:%f %f %f", rt_data->axis_data[0].motor_cmd_pos, rt_data->axis_data[1].motor_cmd_pos, rt_data->axis_data[2].motor_cmd_pos);
    // LOG(LOG_LEVEL_DEBUG, "motor_act_pos:%f %f %f", rt_data->axis_data[0].motor_act_pos, rt_data->axis_data[1].motor_act_pos, rt_data->axis_data[2].motor_act_pos);       
    PLAN_BLOCK* current_block = queue_peak(plan_queue);
    // 如果是到位检测模组，发送到位信号
    if(current_block->mode == PMC_MODE_ARRIVE_CHECK){
        if(!cycle_block_axis_arrive_check(current_block)) return false;
        LOG(LOG_LEVEL_DEBUG,  "到位检测 pos:%f %f %f vel:%f", rt_data->axis_data[AXIS_X].motor_act_pos, rt_data->axis_data[AXIS_Y].motor_act_pos, rt_data->axis_data[AXIS_Z].motor_act_pos, current_block->end_vel);
    }
    if(current_block->mode == PMC_MODE_EXACT_STOP) {
        if(!cycle_block_axis_arrive_check(current_block)) return false;
        LOG(LOG_LEVEL_DEBUG,  "精准停 pos:%f %f %f vel:%f", rt_data->axis_data[AXIS_X].motor_act_pos, rt_data->axis_data[AXIS_Y].motor_act_pos, rt_data->axis_data[AXIS_Z].motor_act_pos, current_block->end_vel);
    }
    // 判断到位宽度
    if(current_block->type == RAPID){
        return cycle_block_axis_arrive_check(current_block);
    }
    return true;
}

static GNC_RES cycle_tp_add_block(){
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
    if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;
    return GNC_CAL_SUCCESS;
}

static GNC_RES cycle_tp_look_ahead(VELPLAN_TYPE vplan_mode, double ncu_cycle, double vs){
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
    vp_cal_bid_lookahead(vplan_mode, plan_queue, ncu_cycle);
    // 如果无法减速到ve
    if(IsGreaterThan(vs, 0)){
        double del_dist = vp_cal_decel_dist(vplan_mode, cur_block->start_vel, cur_block->end_vel, cur_block->max_acc, cur_block->max_jerk);
        if(IsGreaterThan(del_dist, cur_block->length)) {
            double del_vel = vp_cal_decel_vel(vplan_mode, cur_block->start_vel, cur_block->max_acc, cur_block->max_jerk, cur_block->length);
            cur_block->end_vel = del_vel;
        }
    }
    // LOG(LOG_LEVEL_INFO, "2 %f %f", cur_block->length, cur_block->feed_vel);
    return GNC_CAL_SUCCESS;
}

// 计算段的首末速度、最大速度、加速度、加加速度
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block){
    switch (current_block->mode)
    {
        // 切削方式 G64
        case PMC_MODE_CHOP:
            block_cal_chop_vel(current_block);
            break;
        // 攻丝方式 G63
        case PMC_MODE_TAP:
            block_cal_tap_vel(current_block);
            break;
        // 准确停止、到位检测 G09、G61、G27、G28
        case PMC_MODE_ARRIVE_CHECK:
        case PMC_MODE_EXACT_STOP:
            block_cal_exact_stop_vel(current_block);
            break;
        default:
            break;
    }
}

// 精准停：终点减速，到位检测
static void block_cal_exact_stop_vel(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
            // 当参数RDR（No.1401#6）的值为1时为空运行速度×JV，其值为0时则为快速移动速度
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;        
            }
            else{
                current_block->max_vel = current_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
            }
            current_block->start_vel = 0;
            current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            // 被钳制在最大切削进给速度上
            double min_feed = MIN3(par_axis[0][PAR_AX_FEED_SPD].value.val_real, par_axis[1][PAR_AX_FEED_SPD].value.val_real, par_axis[2][PAR_AX_FEED_SPD].value.val_real);
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT;
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, min_feed);
            }
            current_block->start_vel = 0;
            current_block->end_vel = 0;
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
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real;
            }
            else{
                current_block->max_vel = current_block->feed_vel;
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real;
            }
            else{
                current_block->max_vel = current_block->feed_vel;
            }
            if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK)) current_block->start_vel = current_block->end_vel = 0;
            else current_block->start_vel = current_block->end_vel = current_block->max_vel;    
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
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
            }
            else{
                current_block->max_vel = current_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
            }
            current_block->start_vel = current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            double min_feed = MIN3(par_axis[0][PAR_AX_FEED_SPD].value.val_real, par_axis[1][PAR_AX_FEED_SPD].value.val_real, par_axis[2][PAR_AX_FEED_SPD].value.val_real);
            if (rt_get_sys_state(SYS_IS_DRY_RUN))
            {
                current_block->max_vel = par_chan[0][PAR_CH_DRYRUN_SPD].value.val_real * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT;
            }
            else{
                current_block->max_vel = MIN(current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / OVERRIDE_PERCENT, min_feed);
            }
            if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK)) current_block->start_vel = current_block->end_vel = 0;
            else current_block->start_vel = current_block->end_vel = current_block->max_vel;  
            break;
        default:
            break;
    }
}

// 计算单轴速度
static void mem_cal_axis_vel(POS *current_pos, POS *target_pos, double interp_pinc, double target_vel){
    double x_axis_vel = 0;
    double y_axis_vel = 0;
    double z_axis_vel = 0;
    if(IsGreaterThan(interp_pinc, 0)){
        VECTOR vec = POS_VECTOR(target_pos, current_pos);
        x_axis_vel = target_vel * vec.x / interp_pinc;
        y_axis_vel = target_vel * vec.y / interp_pinc;
        z_axis_vel = target_vel * vec.z / interp_pinc;
    }
    setVarAxisDouble(AXIS_X, VAR_AXIS_CMD_VEL, x_axis_vel, 0);
    setVarAxisDouble(AXIS_Y, VAR_AXIS_CMD_VEL, y_axis_vel, 0);
    setVarAxisDouble(AXIS_Z, VAR_AXIS_CMD_VEL, z_axis_vel, 0);
}

static GNC_RES vp_cal_g04_vel_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time){
    GNC_RES res = VP_CAL_SUCCESS;
    vel_profile->t_cycle = (double)plan_block->pause_time;
    vel_profile->vs = 0;
    vel_profile->ve = 0;
    switch(velplan_type){
        case T_SHAPE:
            vel_profile->t[0] = vel_profile->t_cycle;
            vel_profile->t[1] = vel_profile->t_cycle;
            vel_profile->t[2] = vel_profile->t_cycle;
            vel_profile->v[0] = 0;
            vel_profile->v[1] = 0;
            vel_profile->v[2] = 0;
            vel_profile->s[0] = 0;
            vel_profile->s[1] = 0;
            vel_profile->s[2] = 0;
            break;
        case S_SHAPE:
        case SIN_SHAPE:
            vel_profile->t[0] = vel_profile->t_cycle;
            vel_profile->t[1] = vel_profile->t_cycle;
            vel_profile->t[2] = vel_profile->t_cycle;
            vel_profile->t[3] = vel_profile->t_cycle;
            vel_profile->t[4] = vel_profile->t_cycle;
            vel_profile->t[5] = vel_profile->t_cycle;
            vel_profile->v[0] = 0;
            vel_profile->v[1] = 0;
            vel_profile->v[2] = 0;
            vel_profile->v[3] = 0;
            vel_profile->v[4] = 0;
            vel_profile->v[5] = 0;
            vel_profile->s[0] = 0;
            vel_profile->s[1] = 0;
            vel_profile->s[2] = 0;
            vel_profile->s[3] = 0;
            vel_profile->s[4] = 0;
            vel_profile->s[5] = 0;
        break;
        default:
            res = GNC_CAL_ERROR;
        break;
    }
    return res;
}

static GNC_RES vp_cal_g04_pause_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time){
    GNC_RES res = VP_CAL_SUCCESS;
    vel_profile->t_cycle = 1.0;
    vel_profile->vs = 0;
    vel_profile->ve = 0;
    switch(velplan_type){
        case T_SHAPE:
        case S_SHAPE:
        case SIN_SHAPE:
            vel_profile->t[0] = 0;
            vel_profile->t[1] = 0;
            vel_profile->t[2] = 0;
            vel_profile->t[3] = 0;
            vel_profile->t[4] = 0;
            vel_profile->t[5] = 0;
            vel_profile->v[0] = 0;
            vel_profile->v[1] = 0;
            vel_profile->v[2] = 0;
            vel_profile->v[3] = 0;
            vel_profile->v[4] = 0;
            vel_profile->v[5] = 0;
            vel_profile->s[0] = 0;
            vel_profile->s[1] = 0;
            vel_profile->s[2] = 0;
            vel_profile->s[3] = 0;
            vel_profile->s[4] = 0;
            vel_profile->s[5] = 0;
        break;
        default:
            res = GNC_CAL_ERROR;
        break;
    }
    return res;
}

static void check_traj_data_valid(TRAJ_PARAMS* traj_data){
    if(IsLessThan(traj_data->s, 0)){
        traj_data->s = 0;
    }
    if(IsLessThan(traj_data->v, 0)){
        traj_data->v = 0;
    }
}
