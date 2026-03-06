#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "mem.h"
#include "global.h"
#include "datatype.h"
#include "pathplan.h"
#include "log.h"
#include "logapi.h"
#include "varapi.h"
#include <alarmapi.h>

static bool mem_check_running();
static bool cycle_tp_is_finished();
static void mem_set_state();
static double mem_cal_pinc();
static bool mem_check_res(GNC_RES res);
static bool mem_check_running();
static void mem_cal_pos(double accum_distance);
static void interpolate_line(PLAN_BLOCK* current_block, double accum_distance);
static void interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance);
static void interpolate_circle(PLAN_BLOCK* current_block, double accum_distance);

GNC_RES cycle_tp_update_tp(double target_vel);
GNC_RES cycle_tp_update_params(VELPLAN_TYPE velplan_type, double cycle_time);
static bool cycle_block_is_arrived();
static bool cycle_tp_is_newvel_unvalid(double target_vel);
static GNC_RES cycle_tp_update_block();
static GNC_RES cycle_tp_update_vp(double current_vel, double target_vel);
static GNC_RES plan_block_get_next();
static GNC_RES plan_block_get_update();
static GNC_RES cycle_tp_cal_running(VELPLAN_TYPE velplan_type, double vs);
static GNC_RES cycle_tp_cal_pausing(VELPLAN_TYPE velplan_type, double vs, double ve);
static GNC_RES cycle_tp_add_block();
static GNC_RES cycle_tp_look_ahead(double vs);
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel);

// MEM 模式下的执行流程：处理开始和暂停 → 计算插补距离 → 计算插补空间坐标
void handle_run_cmd(){
    double interp_pinc = 0;
    // 设置自动加工状态
    mem_set_state();
    // 调试：模拟机床锁住，快速显示加工点位
    if(sys->is_mac_lock){
        if(gcode_queue_is_empty(gcode_queue)) return;
        GCODE_BLOCK *block = gcode_queue_peak(gcode_queue);
        var_axis[AXIS_X].cmd_pos = block->pos.x;
        var_axis[AXIS_Y].cmd_pos = block->pos.y;
        var_axis[AXIS_Z].cmd_pos = block->pos.z;
        gcode_queue_pop(gcode_queue);
        return;
    }
    // double start, end;
    // start = clock();
    // 计算插补距离
    interp_pinc = mem_cal_pinc();
    // end = clock();
    // sys->cpu_time_used = ((double) (end - start));
    // 根据插补类型计算插补点：快速G00、直线G01、圆弧G02|03……
    mem_cal_pos(interp_pinc);
    // 更新速度参数
    var_chan[var_sys.active_chan].cmd_feedrate = cycle_tp->traj_data->v; // 通道合成指令速度
    var_chan[var_sys.active_chan].cmd_acc = cycle_tp->traj_data->a;
    var_chan[var_sys.active_chan].cmd_jerk = cycle_tp->traj_data->j;
    // LOG_OUTPUT(INFO, "插补距离: %.4f, 插补速度: %.4f", cycle_tp->traj_data->s, var_chan[var_sys.active_chan].cmd_feedrate);
    // LOG_OUTPUT(INFO, "当前位置： %f %f %f", var_axis[AXIS_X].cmd_pos, var_axis[AXIS_Y].cmd_pos, var_axis[AXIS_Z].cmd_pos);
}

static void mem_set_state(){
    GNC_RES res = GNC_CAL_SUCCESS;

    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // log_system("[mem] 记忆模式处理复位");
            LOG_OUTPUT(INFO, "记忆模式处理复位");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                cycle_tp->mem_exec_state = EXEC_PAUSING;
                cycle_tp->is_event_trigger = true;
                var_chan[var_sys.active_chan].is_resetting = true;
            }else if(cycle_tp->mem_exec_state == EXEC_PAUSING){
                var_chan[var_sys.active_chan].is_resetting = true;
            }
            // 通知译码复位
            var_chan[0].parser_reset = true;
            break;
        
        case SYS_EVENT_RERUN:
            // log_system("[mem] 记忆模式处理自动运行");
            LOG_OUTPUT(INFO, "记忆模式处理自动运行");
            if(cycle_tp->mem_exec_state == EXEC_IDLE){
                cycle_tp->mem_exec_state = EXEC_RUNNING;
                cycle_tp->is_event_trigger = true;
                var_chan[var_sys.active_chan].is_running = true;
                // 通知译码运行，初始化译码器
                sys->is_parse_running = true;
                // 译码正在运行，译码结束译码器将其置false
                var_chan[0].is_parsering = true;
            }
            else if(cycle_tp->mem_exec_state == EXEC_PAUSING && !var_chan[var_sys.active_chan].is_resetting){
                cycle_tp->mem_exec_state = EXEC_RUNNING;
                cycle_tp->is_event_trigger = true;
                var_chan[var_sys.active_chan].is_running = true;
            }
            break;

        // todo：自动模式下调整快速倍率是否会影响 G00 速度
        case SYS_EVENT_FEED_OVERRIDE:
            // log_system("[mem] 记忆模式处理进给倍率切换");
            LOG_OUTPUT(INFO, "记忆模式处理进给倍率切换");
            getVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, &var_chan[var_sys.active_chan].feed_override, 0);
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                cycle_tp->is_event_trigger = true;
            }
            break;
        
        case SYS_EVENT_PAUSE:
            // log_system("[mem] 记忆模式处理暂停");
            LOG_OUTPUT(INFO, "记忆模式处理暂停");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                cycle_tp->mem_exec_state = EXEC_PAUSING;
                cycle_tp->is_event_trigger = true;
            }
            break;
        
        default:
            // log_system("[mem] 记忆模式不支持该操作");
            LOG_OUTPUT(INFO, "记忆模式不支持该操作");
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

static void check_traj_data_valid(TRAJ_PARAMS* traj_data){
    if(IsLessThan(traj_data->s, 0)){
        traj_data->s = 0;
    }
    if(IsLessThan(traj_data->v, 0)){
        traj_data->v = 0;
    }
}

// 计算插补距离
static double mem_cal_pinc(){
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
                if(var_chan[var_sys.active_chan].is_sbl){
                    cycle_tp->mem_exec_state = EXEC_IDLE;
                    break;
                }
                if(!queue_is_empty(plan_queue)){
                    // cycle_tp->max_vel = cycle_tp->plan_block->feed_vel * var_chan[var_sys.active_chan].feed_override;
                    cycle_tp->max_vel = cycle_tp->plan_block->feed_vel;
                }
                res = cycle_tp_update_tp(cycle_tp->max_vel);
                CHECK_RES(mem_check_res(res));
                cycle_tp->is_event_trigger = false; 
                shm_var_update_prog();
            }
            res = cycle_tp_update_params(par_chan[var_sys.active_chan].vplan_mode, par_ncu.cycle);
            CHECK_RES(mem_check_res(res));
            interp_distance = cycle_tp->accum_distance;
            break;

        // 执行暂停中
        case EXEC_PAUSING:
            // 检查是否完成该段的减速运动
            if (cycle_tp_is_finished() || cycle_tp->is_event_trigger){ 
                // 如果段内完成暂停         
                if(cycle_tp_is_finished() && cycle_tp->is_paused_inside_block){ 
                    if(var_chan[var_sys.active_chan].is_resetting){
                        var_chan[var_sys.active_chan].is_resetting = false;
                        var_chan[var_sys.active_chan].is_running = false;
                        var_chan[var_sys.active_chan].is_moving = false;
                        cycle_tp->mem_exec_state = EXEC_IDLE;
                        // 复位，译码、预处理、前瞻队列均重置
                        tp_reset();
                        rt_par_reset();
                        // 重置当前译码行
                        setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                    }else{
                        var_chan[var_sys.active_chan].is_moving = false;
                        // LOG_OUTPUT(INFO, "程序暂停完成, %d", cycle_tp->plan_block->type);
                    }                  
                    break;
                }else{ 
                    res = cycle_tp_update_tp(0);
                    CHECK_RES(mem_check_res(res));
                }
                cycle_tp->is_event_trigger = false; 
                shm_var_update_prog();
            }
            res = cycle_tp_update_params(par_chan[var_sys.active_chan].vplan_mode, par_ncu.cycle);
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
        // 如果进给倍率为0
        if(cycle_tp->mem_exec_state == EXEC_RUNNING){
            cycle_tp->plan_block->type = UNKNOWN;
            return false;
        }
        break;

    case GNC_DATA_EMPTY:
        // 运行中无数据，则进入等待 
        if(mem_check_running()){
            if(queue_is_empty(pretreat_queue) && gcode_queue_is_empty(gcode_queue)){
                var_chan[0].is_gcode_ending = true;
            }
            return false;
        }
        // 译码已结束后无数据，则程序运行完成
        cycle_tp->mem_exec_state = EXEC_IDLE;
        var_chan[var_sys.active_chan].is_running = false;
        var_chan[var_sys.active_chan].is_progend = true;
        var_chan[var_sys.active_chan].is_moving = false;
        var_chan[0].is_gcode_ending = true;
        tp_reset();
        // log_system("[mem] 程序运行结束");
        LOG_OUTPUT(INFO, "程序运行结束");
        return false;

    case GNC_DATA_NULL:
    case GNC_PARAM_UNVALID:
    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    case VP_IS_NULL:
    case VP_CAL_ERROR:
    case VP_PARAM_UNVALID:
    case VP_LENGTH_IS_ZERO:
    case VP_FEED_RATE_IS_ZERO:
    case VP_LENGTH_UNVALID:
    default:
        cycle_tp->mem_exec_state = EXEC_IDLE;
        var_chan[var_sys.active_chan].is_estop = true;
        var_chan[var_sys.active_chan].is_resetting = false;
        var_chan[var_sys.active_chan].is_running = false;
        var_chan[var_sys.active_chan].is_progend = true;
        var_chan[var_sys.active_chan].is_moving = false;
        // log_system("[mem] 记忆模式报警(%d)", res);
        LOG_OUTPUT(ERROR, "记忆模式报警 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 3);
        return false;
    }
}

static bool mem_check_running(){
    if(var_chan[0].is_parsering) return true;
    return (!queue_is_empty(plan_queue) || !queue_is_empty(pretreat_queue) || !gcode_queue_is_empty(gcode_queue));
}

// 检查是否执行完速度曲线
static bool cycle_tp_is_finished(){
    if(cycle_tp->interp_cycle > cycle_tp->vel_profile->t_cycle){
        cycle_tp->plan_block->type = UNKNOWN;
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
    cycle_tp->is_paused_inside_block = false;
}

void cycle_tp_reset(){
    cycle_tp->plan_block->type = UNKNOWN;
    memset(cycle_tp->traj_data, 0, sizeof(TRAJ_PARAMS));
    cycle_tp->is_event_trigger = false;
    cycle_tp->is_paused_inside_block = false;
}

static void mem_cal_pos(double accum_distance){
    // PLAN_BLOCK* current_block = queue_peak(plan_queue);
    PLAN_BLOCK* current_block = cycle_tp->plan_block;
    switch(current_block->type){
        case RAPID:
            interpolate_rapid(current_block, accum_distance);
            break;
        case LINE:
            interpolate_line(current_block, accum_distance);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            interpolate_circle(current_block, accum_distance);
            break;
        case UNKNOWN:
        default:
            break;
    }
}

// 计算直线段插补点
static void interpolate_line(PLAN_BLOCK* current_block, double accum_distance) {
    var_axis[AXIS_X].cmd_pos = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    var_axis[AXIS_Y].cmd_pos = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    var_axis[AXIS_Z].cmd_pos = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
}

// 计算快速段插补点
static void interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance) {
    var_axis[AXIS_X].cmd_pos = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    var_axis[AXIS_Y].cmd_pos = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    var_axis[AXIS_Z].cmd_pos = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
}

// 计算圆弧段插补点
static void interpolate_circle(PLAN_BLOCK* current_block, double accum_distance) {
    double angle = accum_distance / current_block->circular_block.radius;
    double angle_interp = 0;
    switch(current_block->circular_block.dir){
        case CLOCK:
            angle_interp = current_block->circular_block.statr_rad - angle;
            break;
        case COUNTERCLOCK:
            angle_interp = current_block->circular_block.statr_rad + angle;
            break;
        default:
            break;
    }

    switch(current_block->circular_block.plane){
        case PLANE_XY:
            var_axis[AXIS_X].cmd_pos = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            var_axis[AXIS_Y].cmd_pos = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            var_axis[AXIS_Z].cmd_pos = var_axis[AXIS_Z].cmd_pos;
            break;
        case PLANE_XZ:
            var_axis[AXIS_X].cmd_pos = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            var_axis[AXIS_Y].cmd_pos = var_axis[AXIS_Y].cmd_pos;
            var_axis[AXIS_Z].cmd_pos = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            break;
        case PLANE_YZ:
            var_axis[AXIS_X].cmd_pos = var_axis[AXIS_X].cmd_pos;
            var_axis[AXIS_Y].cmd_pos = current_block->circular_block.center_pos.y + current_block->circular_block.radius * cos(angle_interp);
            var_axis[AXIS_Z].cmd_pos = current_block->circular_block.center_pos.z + current_block->circular_block.radius * sin(angle_interp);
            break;
        default:
            break;
    }
}

GNC_RES cycle_tp_update_tp(double target_vel){
    GNC_RES res = GNC_CAL_SUCCESS;
    if(cycle_tp_add_block() == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;   // 如果没有插补数据
    // 如果新速度不满足，不更新新速度
    if(cycle_tp->is_event_trigger && cycle_tp_is_newvel_unvalid(target_vel)){
        if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_paused_inside_block = true;
        else cycle_tp->is_paused_inside_block = false;
        return VP_DELCINE_TO_END;
    }
    res = cycle_tp_update_block();
    if(res != GNC_CAL_SUCCESS) return res;
    // 如果加工段允许最大速度为0（倍率为0）
    if(IsEqualTo(cycle_tp->plan_block->max_vel, 0)) return VP_MAX_VEL_IS_ZERO; 
    res = cycle_tp_update_vp(var_chan[var_sys.active_chan].act_feedrate, target_vel);
    if(res != GNC_CAL_SUCCESS) return res;
    return res;
}

GNC_RES cycle_tp_update_params(VELPLAN_TYPE velplan_type, double cycle_time){
    vp_cal_traj_data(velplan_type, cycle_tp->vel_profile, cycle_tp->interp_cycle, cycle_tp->traj_data, cycle_time);
    cycle_tp->current_distance = cycle_tp->target_distance;
    cycle_tp->target_distance = cycle_tp->traj_data->s - cycle_tp->accum_distance;
    cycle_tp->accum_distance = cycle_tp->traj_data->s;
    cycle_tp->remain_distance = cycle_tp->plan_block->length - cycle_tp->accum_distance;
    cycle_tp->interp_cycle += 1;
    if(IsLessThan(cycle_tp->target_distance, 0)){
        return VP_CAL_S_ERROR;
    }
    return GNC_CAL_SUCCESS;
}

static bool cycle_tp_is_newvel_unvalid(double target_vel){
    double vs = var_chan[0].act_feedrate;
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    double ve = head_block->end_vel;
    double remain_s = head_block->length - cycle_tp->accum_distance;
    double total_decline_length = 0;
    if(IsGreaterThan(vs, target_vel) && IsGreaterThanOrEqual(target_vel, ve)){
        total_decline_length = vp_cal_decel_dist(par_chan[var_sys.active_chan].vplan_mode, vs, target_vel, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk)
        + vp_cal_decel_dist(par_chan[var_sys.active_chan].vplan_mode, target_vel, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    }else if((IsGreaterThan(target_vel, vs) && IsGreaterThanOrEqual(vs, ve)) || (IsGreaterThanOrEqual(vs, ve) && IsGreaterThan(ve, target_vel))){
        total_decline_length = vp_cal_decel_dist(par_chan[var_sys.active_chan].vplan_mode, vs, ve, cycle_tp->plan_block->max_acc, cycle_tp->plan_block->max_jerk);
    } 
    if(IsGreaterThanOrEqual(total_decline_length, remain_s)) return true;
    return false;
}

static GNC_RES cycle_tp_update_block(){
    GNC_RES res = GNC_CAL_SUCCESS;
    // 如果到达加工段终点
    if(cycle_block_is_arrived()){
        res = plan_block_get_next();
        // LOG_OUTPUT(INFO, "获取下一个插补段");
    }else{
        res = plan_block_get_update();
        // LOG_OUTPUT(INFO, "更新当前插补段");
    }
    if(res == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    if(IsLessThan(var_chan[var_sys.active_chan].act_feedrate, 0)) {
        return VP_CAL_V_ERROR;
    }
    if(cycle_tp_look_ahead(var_chan[var_sys.active_chan].act_feedrate) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    queue_copy_block(plan_queue, 0, cycle_tp->plan_block);
    return res;
}

static GNC_RES cycle_tp_update_vp(double current_vel, double target_vel){
    GNC_RES res = GNC_CAL_SUCCESS;
    if(current_vel > target_vel){
        res = cycle_tp_cal_pausing(par_chan[var_sys.active_chan].vplan_mode, current_vel, target_vel);
    }else{
        res = cycle_tp_cal_running(par_chan[var_sys.active_chan].vplan_mode, current_vel);
    }
    if(res != VP_CAL_SUCCESS)  return res;
    if(IsEqualTo(cycle_tp->vel_profile->ve, 0)) cycle_tp->is_paused_inside_block = true;
    else cycle_tp->is_paused_inside_block = false;
    return res;
}

static GNC_RES plan_block_get_next(){
    if(queue_pop(plan_queue) != GNC_OPERATOR_SUCCESS) return GNC_DATA_EMPTY;
    return GNC_CAL_SUCCESS;
}

static GNC_RES plan_block_get_update(){
    GNC_RES res = GNC_CAL_SUCCESS;
    // 更新段起点和速度，重新计算插补段参数
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    head_block->start_pos = (POS){.x = var_axis[AXIS_X].cmd_pos, .y = var_axis[AXIS_Y].cmd_pos, .z = var_axis[AXIS_Z].cmd_pos};
    head_block->start_vel = var_chan[var_sys.active_chan].act_feedrate;
    block_cal_vector(head_block);
    block_cal_length(head_block);
    if(IsLessThanOrEqual(head_block->length, MEM_END_TOL)) res = plan_block_get_next();
    return res;
}

static GNC_RES cycle_tp_cal_running(VELPLAN_TYPE velplan_type, double vs){
    GNC_RES res = VP_CAL_SUCCESS;
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    res = vp_cal_vel_profile(velplan_type, cycle_tp->plan_block, cycle_tp->vel_profile, par_ncu.cycle); // 计算速度曲线
    return res;
}

static GNC_RES cycle_tp_cal_pausing(VELPLAN_TYPE velplan_type, double vs, double ve){
    GNC_RES res = VP_CAL_SUCCESS;
    double remain_s = cycle_tp->plan_block->length;
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    // 计算减速距离
    double decline_s = vp_cal_decel_dist(velplan_type, vs, ve, amax, jmax);
    if(IsLessThan(decline_s, 0)){
        return VP_CAL_S_ERROR;
    }
    if (decline_s > remain_s){ // 在当前段无法减速到ve 
        cycle_tp->plan_block->start_vel = vs; 
        cycle_tp->plan_block->end_vel = vp_cal_decel_vel(velplan_type, vs, amax, jmax, remain_s);  
        if(IsLessThan(cycle_tp->plan_block->end_vel, 0)) return VP_CAL_ERROR;  
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = remain_s;
        res = vp_cal_vel_profile(velplan_type, cycle_tp->plan_block, cycle_tp->vel_profile, par_ncu.cycle);
    }
    else{ // 在当前段可以减速到ve
        cycle_tp->plan_block->start_vel = vs;
        cycle_tp->plan_block->end_vel = ve;
        cycle_tp->plan_block->max_vel = vs;
        cycle_tp->plan_block->length = decline_s;
        res = vp_cal_vel_profile(velplan_type, cycle_tp->plan_block, cycle_tp->vel_profile, par_ncu.cycle);
    }
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    return res;
}

// 检查是否到目标点
static bool cycle_block_is_arrived(){
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    if((fabs(head_block->end_pos.x - var_axis[AXIS_X].cmd_pos)) > MEM_END_TOL) return false;
    if((fabs(head_block->end_pos.y - var_axis[AXIS_Y].cmd_pos)) > MEM_END_TOL) return false;
    if((fabs(head_block->end_pos.z - var_axis[AXIS_Z].cmd_pos)) > MEM_END_TOL) return false;
    return true;
}

static GNC_RES cycle_tp_add_block(){
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* cur_block;
    PLAN_BLOCK* pre_block;
    // 拷贝 pretreat_queue 的数据
    for(int i = 0; i < LOOK_AHEAD_SIZE; i++){
        // 如果前瞻队列已满，提前结束
        if(queue_is_full(plan_queue)) break;
        // 如果预处理队列无数据，提前结束
        if(queue_is_empty(pretreat_queue)) break;
        pretreat_block = queue_peak(pretreat_queue);
        // 注意：如果长度为0，删除该段
        if(IsLessThanOrEqual(pretreat_block->length, 0)) {
            queue_pop(pretreat_queue);
            continue;
        }
        queue_push(plan_queue, pretreat_block);
        queue_pop(pretreat_queue);
    }
    if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;
    return GNC_CAL_SUCCESS;
}

static GNC_RES cycle_tp_look_ahead(double vs){
    PLAN_BLOCK* cur_block;
    PLAN_BLOCK* pre_block;
    /******************************************
    1. 初始化首末速度，从末端往前计算，每次前瞻的末段末速度为0，plan_queue->next_vel = 0
    2. 只有两段以上才进行遍历 i > 0
    3. 当为G01时，首段初速度会修正为前一段的末速度；其他类型的初速度为0 */ 
    plan_queue->next_vel = 0;
    // 两段及以上，初始化首末速度
    for(int i = plan_queue->size - 1; i > 0; i--){
        cur_block = queue_get_block(plan_queue, i);
        pre_block = queue_get_block(plan_queue, i - 1);
        block_cal_lookahead_vel(cur_block, pre_block->type, plan_queue->next_vel);
        plan_queue->next_vel = cur_block->start_vel;
    }
    // 修正首段速度
    cur_block = queue_peak(plan_queue);
    block_cal_lookahead_vel(cur_block, plan_queue->pre_head_type, plan_queue->next_vel);
    // LOG_OUTPUT(INFO, "1 %f %f", cur_block->length, cur_block->feed_vel);
    cur_block->start_vel = vs;
    // 判断是否大于最大速度
    if(cur_block->start_vel > cur_block->max_vel) cur_block->max_vel = cur_block->start_vel;
    /*******************************************/
    vp_cal_bid_lookahead(par_chan[0].vplan_mode, plan_queue, par_ncu.cycle);
    // LOG_OUTPUT(INFO, "2 %f %f", cur_block->length, cur_block->feed_vel);
    return GNC_CAL_SUCCESS;
}

// 计算段的首末速度、最大速度、加速度、加加速度
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel){
    switch(current_block->type){
        case RAPID:
            // current_block->max_vel = current_block->max_vel * var_chan[0].rapid_override;
            current_block->max_vel = current_block->max_vel;
            current_block->start_vel = 0;
            current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            // current_block->max_vel = current_block->feed_vel * var_chan[0].feed_override;
            current_block->max_vel = current_block->feed_vel;
            // 预留：如果开启单段，首末速度为0
            current_block->end_vel = next_start_vel;
            if(sys->is_lookahead && (!var_chan[0].is_sbl) && (pre_type == LINE || pre_type == CIRCLE_CCW || pre_type == CIRCLE_CW)){
                // block1 <-- block2 <-- block3
                //  ------   --------   ------
                // |?--100| |100--100| |100--0|
                //  ------   --------   ------
                // 如果开启前瞻，段首末速度初始化为段内最大允许速度
                current_block->start_vel = current_block->max_vel;  
            }else{
                current_block->start_vel = 0;
            }
            break;

        default:
            break;
    }
}