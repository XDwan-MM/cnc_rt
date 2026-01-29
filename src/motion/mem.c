#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "mem.h"
#include "global.h"
#include "datatype.h"
#include "pathplan.h"
#include "log.h"
#include "varapi.h"
#include <alarmapi.h>

static GNC_RES cycle_tp_init(VELPLAN_TYPE velplan_type, double vs);
static GNC_RES pausing_tp_init(VELPLAN_TYPE velplan_type, double vs);
static bool tp_is_arrived(COORD_TP* tp);
static GNC_RES tp_params_update(VELPLAN_TYPE velplan_type, double cycle_time, COORD_TP* tp, TRAJ_PARAMS* traj_data);
static GNC_RES cycle_tp_block_update(PLAN_BLOCK* current_block, POS current_pos, double current_vel);
static GNC_RES cycle_tp_get_first();
static GNC_RES cycle_tp_get_next();
static void mem_set_state();
static double mem_cal_pinc();
static void mem_cal_pos(double accum_distance);
static bool mem_check_res(GNC_RES res);
static void interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance);
static void interpolate_line(PLAN_BLOCK* current_block, double accum_distance);
static void interpolate_circle(PLAN_BLOCK* current_block, double accum_distance);
static bool mem_check_running();

// MEM 模式下的执行流程：处理开始和暂停 → 计算插补距离 → 计算插补空间坐标
void handle_run_cmd(){
    double interp_pinc = 0;
    // 设置自动加工状态
    mem_set_state();
    // 调试：模拟空跑，快速显示加工点位
    if(sys->is_mac_lock){
        if(gcode_queue_is_empty(gcode_queue)) return;
        GCODE_BLOCK *block = gcode_queue_peak(gcode_queue);
        var_axis[AXIS_X].cmd_pos = block->pos.x;
        var_axis[AXIS_Y].cmd_pos = block->pos.y;
        var_axis[AXIS_Z].cmd_pos = block->pos.z;
        gcode_queue_pop(gcode_queue);
        printf("sys->is_mac_lock\n");
        return;
    }
    // double start, end;
    // start = clock();
    // 计算插补距离
    interp_pinc = mem_cal_pinc();
    // end = clock();
    // sys->cpu_time_used = ((double) (end - start));
    // 更新：【插补功能】张哲
    // 根据插补类型计算插补点：快速G00、直线G01、圆弧G02|03……
    mem_cal_pos(interp_pinc);
    // interpolationExec(interp_pinc);
    // 更新速度参数
    var_chan[0].cmd_feedrate = traj_data->v; // 通道合成指令速度
    var_chan[0].cmd_acc = traj_data->a;
    var_chan[0].cmd_jerk = traj_data->j;
}

static void mem_set_state(){
    GNC_RES res = GNC_CAL_SUCCESS;

    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            LOG_OUTPUT(INFO, "click reset");
            if(mem_exec_state == EXEC_RUNNING || mem_exec_state == EXEC_PAUSING){
                mem_exec_state = EXEC_CANCELING;
                res = cycle_tp_get_first();
                CHECK_RES(mem_check_res(res));
                res = pausing_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate);
                CHECK_RES(mem_check_res(res));
                var_chan[0].is_resetting = true;
                LOG_OUTPUT(ERROR, "reset line number: %d", cycle_tp->plan_block->line_number);
            }else if(mem_exec_state == EXEC_WAITING || mem_exec_state == EXEC_IDLE){
                mem_exec_state = EXEC_IDLE;
                var_chan[0].is_running = false;
                var_chan[0].is_moving = false;
                var_chan[0].is_resetting = false;
                // 复位，译码、预处理、前瞻队列均重置
                tp_reset();
                rt_par_reset();
                // 重置当前译码行
                setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                LOG_OUTPUT(INFO, "reset program");
            }
            break;
        
        case SYS_EVENT_RERUN:
            LOG_OUTPUT(INFO, "click run");
            LOG_OUTPUT(INFO, "vplan_mode: %d", par_chan[0].vplan_mode);
            // 上一个状态为空闲或复位
            if(mem_exec_state == EXEC_IDLE && queue_is_empty(plan_queue)){
                mem_exec_state = EXEC_WAITING;
            }
            // 上一个状态或当前状态为暂停
            else if((mem_exec_state == EXEC_IDLE || mem_exec_state == EXEC_PAUSING)
                    && !queue_is_empty(plan_queue)){
                mem_exec_state = EXEC_WAITING;
            }
            break;
        
        case SYS_EVENT_PAUSE:
            LOG_OUTPUT(INFO, "click pause");
            if(mem_exec_state == EXEC_RUNNING){
                mem_exec_state = EXEC_PAUSING;
                printf("pause line number:%d\n", cycle_tp->plan_block->line_number);
                LOG_OUTPUT(ERROR, "pause line number: %d res%d", cycle_tp->plan_block->line_number,res);
                res = cycle_tp_get_first();
                CHECK_RES(mem_check_res(res));
                res = pausing_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate);
                CHECK_RES(mem_check_res(res));
            }
            break;
        
        case SYS_EVENT_X_NEG_PRESS:
        case SYS_EVENT_Y_NEG_PRESS:
        case SYS_EVENT_Z_NEG_PRESS:
        case SYS_EVENT_X_POS_PRESS:
        case SYS_EVENT_Y_POS_PRESS:
        case SYS_EVENT_Z_POS_PRESS:
            LOG_OUTPUT(INFO, "mem mode is not support this operation");
            break;

        default:
            LOG_OUTPUT(INFO, "mem mode is not support this operation");
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

// 计算插补距离
static double mem_cal_pinc(){
    double interp_distance = 0;
    GNC_RES res = GNC_CAL_SUCCESS;
    // 初始化默认为 IDLE
    switch(mem_exec_state){
        case EXEC_IDLE:
            memset(traj_data, 0, sizeof(TRAJ_PARAMS));
            break;

        case EXEC_WAITING:
            res = cycle_tp_get_first();
            CHECK_RES(mem_check_res(res));
            break;

        case EXEC_RUNNING:
            // 完成一段的插补
            if (tp_is_arrived(cycle_tp)){
                // 预留：如果单段使能，跳转到 IDLE，等待下一次 run 事件 
                res = cycle_tp_get_next();
                CHECK_RES(mem_check_res(res));
            }
            res = tp_params_update(par_chan[0].vplan_mode, par_ncu.cycle, cycle_tp, traj_data);
            CHECK_RES(mem_check_res(res));
            interp_distance = cycle_tp->accum_distance;
            break;

        // 执行暂停中
        case EXEC_PAUSING:
            // 检查是否完成该段的减速运动
            if (tp_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    var_chan[0].is_moving = false;
                    break;
                }else{
                    res = cycle_tp_get_next();
                    CHECK_RES(mem_check_res(res));
                    res = pausing_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate);
                    CHECK_RES(mem_check_res(res));
                }
            }
            res = tp_params_update(par_chan[0].vplan_mode, par_ncu.cycle, pausing_tp, traj_data);
            CHECK_RES(mem_check_res(res));
            interp_distance = pausing_tp->accum_distance;
            break;

        case EXEC_CANCELING:
            // 检查是否完成该段的减速运动
            if (tp_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    var_chan[0].is_running = false;
                    var_chan[0].is_moving = false;
                    var_chan[0].is_resetting = false;
                    // 复位，译码、预处理、前瞻队列均重置
                    tp_reset();
                    setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                    break;
                }else{
                    res = cycle_tp_get_next();
                    CHECK_RES(mem_check_res(res));
                    res = pausing_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate);
                    CHECK_RES(mem_check_res(res));
                }
            }
            res = tp_params_update(par_chan[0].vplan_mode, par_ncu.cycle, pausing_tp, traj_data);
            CHECK_RES(mem_check_res(res));
            interp_distance = pausing_tp->accum_distance;
            break;

        default:
            break;
    }
    return interp_distance;
}

static bool mem_check_res(GNC_RES res){
    switch (res)
    {
    case GNC_CAL_SUCCESS:
    case GNC_OPERATOR_SUCCESS:
        // 如果等到新数据，进入running
        if(mem_exec_state == EXEC_WAITING){
            mem_exec_state = EXEC_RUNNING;
            var_chan[0].is_moving = true;
            var_chan[0].is_running = true;
            LOG_OUTPUT(INFO, "program run");
        }
        return true;

    case GNC_DATA_EMPTY:
        LOG_OUTPUT(INFO, "waiting for data");
        // 等待中无数据，则继续等待
        if(mem_check_running() && mem_exec_state == EXEC_WAITING) return true;   
        // 运行中无数据，则进入等待 
        if(mem_check_running() && mem_exec_state == EXEC_RUNNING){
            mem_exec_state = EXEC_WAITING;
            return false;
        }
        // 译码已结束后无数据，则程序运行完成
        mem_exec_state = EXEC_IDLE;
        var_chan[0].is_running = false;
        var_chan[0].is_moving = false;
        var_chan[0].is_progend = true;
        tp_reset();
        LOG_OUTPUT(INFO, "program end");
        return false;

    case GNC_DATA_NULL:
    case GNC_PARAM_UNVALID:
    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    default:
        mem_exec_state = EXEC_IDLE;
        var_chan[0].is_estop = true;
        var_chan[0].is_resetting = false;
        var_chan[0].is_running = false;
        var_chan[0].is_moving = false;
        LOG_ERROR("alarm: %d", res);
        LOG_OUTPUT(ERROR, "current_mode: ESTOP !!!");
        setAlarm(ALARM_CH,ALARM_ERROR,1);

        return false;
    }
}

static bool mem_check_running(){
    if(sys->is_parse_running) return true;
    return (!queue_is_empty(plan_queue) || !queue_is_empty(pretreat_queue) || !gcode_queue_is_empty(gcode_queue));
}

static GNC_RES cycle_tp_get_first(){
    POS current_pos = {.x = var_axis[AXIS_X].act_pos, .y = var_axis[AXIS_Y].act_pos, .z = var_axis[AXIS_Z].act_pos};
    cycle_tp_block_update(cycle_tp->plan_block, current_pos, var_chan[0].act_feedrate);
    if(chan_look_ahead(var_chan[0].act_feedrate) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    if(cycle_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate) == GNC_CAL_ERROR) return GNC_CAL_ERROR;
    shm_var_update_prog();
    LOG_OUTPUT(ERROR, "cycle line number: %d", cycle_tp->plan_block->line_number);
    return GNC_CAL_SUCCESS;
}

static GNC_RES cycle_tp_get_next(){
    // 1、弹出首段 2、前瞻 3、初始化
    // LOG_OUTPUT(INFO, "plan_queue size: %d", queue_get_size(plan_queue));
    if(queue_pop(plan_queue) != GNC_OPERATOR_SUCCESS) return GNC_DATA_EMPTY;
    if(chan_look_ahead(var_chan[0].act_feedrate) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    if(cycle_tp_init(par_chan[0].vplan_mode, var_chan[0].act_feedrate) == GNC_CAL_ERROR) return GNC_CAL_ERROR;
    shm_var_update_prog();
    LOG_OUTPUT(ERROR, "cycle line number: %d", cycle_tp->plan_block->line_number);
    return GNC_CAL_SUCCESS;
}

// 更新段起点和速度，重新计算插补段参数
static GNC_RES cycle_tp_block_update(PLAN_BLOCK* current_block, POS current_pos, double current_vel){
    if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;
    PLAN_BLOCK* head_block = queue_peak(plan_queue);
    head_block->start_pos = current_pos;
    head_block->start_vel = current_vel;
    block_cal_vector(current_block);
    block_cal_length(current_block);
    if(IsEqualTo(head_block->length, 0)) queue_pop(plan_queue);
    return GNC_CAL_SUCCESS;
}

// 检查是否到目标点
static bool tp_is_arrived(COORD_TP *tp){
    if(fabs(tp->remain_distance) <= END_TOL || (tp->interp_cycle > tp->vel_profile->t_cycle)) return true;
    return false;
}

static GNC_RES cycle_tp_init(VELPLAN_TYPE velplan_type, double vs){
    GNC_RES res = GNC_CAL_SUCCESS;
    cycle_tp->plan_block = queue_peak(plan_queue); // 获取首段
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    res = vp_cal_vel_profile(velplan_type, cycle_tp->plan_block, cycle_tp->vel_profile, par_ncu.cycle); // 计算速度曲线

    if(res != GNC_CAL_SUCCESS) LOG_ERROR("vp cal vel profile fail%d\n",res);
    return res;
}

static GNC_RES pausing_tp_init(VELPLAN_TYPE velplan_type, double vs){
    GNC_RES res = GNC_CAL_SUCCESS;
    vs = fabs(vs);
    double remain_s = cycle_tp->remain_distance;
    // 预留：暂停和复位的 j、a、v 速度参数可以另外设置
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    memcpy(pausing_tp->plan_block, cycle_tp->plan_block, sizeof(PLAN_BLOCK));

    // 计算减速距离
    double decline_s = vp_cal_decel_dist(velplan_type, vs, 0, amax, jmax);
    if(decline_s < 0){
        LOG_ERROR("decline_s < 0");
        return GNC_CAL_ERROR;
    }

    if (decline_s > remain_s){ // 在当前段无法减速到0 
        pausing_tp->plan_block->start_vel = vs; 
        pausing_tp->plan_block->end_vel = vp_cal_decel_vel(velplan_type, vs, amax, jmax, remain_s);   
        if(pausing_tp->plan_block->end_vel < 0) return GNC_CAL_ERROR;  
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = remain_s;
        is_paused_in_block = false;
        res = vp_cal_vel_profile(velplan_type, pausing_tp->plan_block, pausing_tp->vel_profile, par_ncu.cycle);
        if(res != GNC_CAL_SUCCESS) LOG_ERROR("decline_s > remain_s, vp cal vel profile fail!");
    }
    else{ // 在当前段可以减速到0
        pausing_tp->plan_block->start_vel = vs;
        pausing_tp->plan_block->end_vel = 0;
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = decline_s;
        is_paused_in_block = true;
        res = vp_cal_vel_profile(velplan_type, pausing_tp->plan_block, pausing_tp->vel_profile, par_ncu.cycle);
        if(res != GNC_CAL_SUCCESS) LOG_ERROR("decline_s <= remain_s, vp cal vel profile fail!");
    }
    pausing_tp->interp_cycle = 1;
    pausing_tp->accum_distance = 0;  
    pausing_tp->remain_distance = pausing_tp->plan_block->length;
    pausing_tp->current_distance = 0;
    pausing_tp->target_distance = 0;
    return res;
}

static GNC_RES tp_params_update(VELPLAN_TYPE velplan_type, double cycle_time, COORD_TP *tp, TRAJ_PARAMS* traj_data){
    vp_cal_traj_data(velplan_type, tp->vel_profile, tp->interp_cycle, traj_data, cycle_time);
    tp->current_distance = tp->target_distance;
    tp->target_distance = traj_data->s - tp->accum_distance;
    tp->accum_distance = traj_data->s;
    tp->remain_distance = tp->plan_block->length - tp->accum_distance;
    tp->interp_cycle += 1;
    if(tp->target_distance <= 0){
        LOG_ERROR("tp->target_distance <= 0");
        return GNC_CAL_ERROR;
    }
    return GNC_CAL_SUCCESS;
}

GNC_RES plan_tp_init(){
    cycle_tp = (COORD_TP*)calloc(sizeof(COORD_TP), 1);
    if(cycle_tp == NULL) return GNC_DATA_NULL;
    cycle_tp->plan_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
    if(cycle_tp->plan_block == NULL) return GNC_DATA_NULL;
    cycle_tp->vel_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
    if(cycle_tp->vel_profile == NULL) return GNC_DATA_NULL;

    pausing_tp = (COORD_TP*)calloc(sizeof(COORD_TP), 1);   
    if(pausing_tp == NULL) return GNC_DATA_NULL;
    pausing_tp->plan_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
    if(pausing_tp->plan_block == NULL) return GNC_DATA_NULL;
    pausing_tp->vel_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
    if(pausing_tp->vel_profile == NULL) return GNC_DATA_NULL;

    mem_exec_state = EXEC_IDLE;
    is_paused_in_block = false;
    return GNC_OPERATOR_SUCCESS;
}

void plan_tp_reset(){
    cycle_tp->plan_block->type = UNKNOWN;
    pausing_tp->plan_block->type = UNKNOWN;
    mem_exec_state = EXEC_IDLE;
    is_paused_in_block = false;
}


static void mem_cal_pos(double accum_distance){
    if(accum_distance <= 0) return;
    PLAN_BLOCK* current_block = queue_peak(plan_queue);
    if(current_block == NULL) return;
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
