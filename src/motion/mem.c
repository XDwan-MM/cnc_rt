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
#include "rt_data.h"
#include <alarmapi.h>

static GNC_RES cycle_tp_init(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs);
static GNC_RES pausing_tp_init(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs);
static bool tp_is_arrived(COORD_TP* tp);
static GNC_RES tp_params_update(VELPLAN_TYPE velplan_mode, double cycle_time, COORD_TP* tp, TRAJ_PARAMS* traj_data);
static GNC_RES cycle_tp_block_update(PLAN_BLOCK* current_block, POS current_pos, double current_vel);
static GNC_RES cycle_tp_get_first(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_feedrate, POS current_pos);
static GNC_RES cycle_tp_get_next(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_feedrate);
static void mem_set_state(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos);
static double mem_cal_pinc(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos);
static POS mem_cal_pos(POS current_pos, double accum_distance);
static bool mem_check_res(GNC_RES res);
static bool mem_check_running();
static POS interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_line(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_circle(POS current_pos, PLAN_BLOCK* current_block, double accum_distance);
static GNC_RES chan_look_ahead(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs);
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel);

// MEM 模式下的执行流程：处理开始和暂停 → 计算插补距离 → 计算插补空间坐标
void handle_run_cmd(){
    double current_vel = var_chan[0][VAR_CHAN_ACT_FEEDRATE].value.f;
    POS current_pos = {.x = var_axis[AXIS_X][VAR_AXIS_CMD_POS_EX].value.f, .y = var_axis[AXIS_Y][VAR_AXIS_CMD_POS_EX].value.f, .z = var_axis[AXIS_Z][VAR_AXIS_CMD_POS_EX].value.f};
    int velplan_mode = par_chan[0][PAR_CH_VPLAN_MODE].value.val_int;
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
    double interp_pinc = 0;
    POS interp_pos = current_pos;
    // 设置自动加工状态
    mem_set_state(velplan_mode, ncu_cycle, current_vel, current_pos);
    // 调试：模拟空跑，快速显示加工点位
    if(rt_data->sys_state[SYS_IS_MAC_LOCK]){
        if(gcode_queue_is_empty(gcode_queue)) return;
        GCODE_BLOCK *block = gcode_queue_peak(gcode_queue);
        setVarAxisDouble(AXIS_X, VAR_AXIS_CMD_POS_EX, block->pos.x, 0);
        setVarAxisDouble(AXIS_Y, VAR_AXIS_CMD_POS_EX, block->pos.y, 0);
        setVarAxisDouble(AXIS_Z, VAR_AXIS_CMD_POS_EX, block->pos.z, 0);
        gcode_queue_pop(gcode_queue);
        LOG_OUTPUT(INFO, "开启机床锁住");
        return;
    }
    // double start, end;
    // start = clock();
    // 计算插补距离
    interp_pinc = mem_cal_pinc(velplan_mode, ncu_cycle, current_vel, current_pos);
    // end = clock();
    // sys->cpu_time_used = ((double) (end - start));
    interp_pos = mem_cal_pos(current_pos, interp_pinc);
    // interpolationExec(interp_pinc);
    // 更新参数
    setVarAxisDouble(AXIS_X, VAR_AXIS_CMD_POS_EX, interp_pos.x, 0);
    setVarAxisDouble(AXIS_Y, VAR_AXIS_CMD_POS_EX, interp_pos.y, 0);
    setVarAxisDouble(AXIS_Z, VAR_AXIS_CMD_POS_EX, interp_pos.z, 0);
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, traj_data->v, 0);
    rt_data->chan_data[0].cmd_acc = traj_data->a;
    rt_data->chan_data[0].cmd_jk = traj_data->j;
}

static void mem_set_state(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos){
    GNC_RES res = GNC_CAL_SUCCESS;
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // log_system("[mem] 记忆模式处理复位");
            LOG_OUTPUT(INFO, "记忆模式处理复位");
            if(mem_exec_state == EXEC_RUNNING || mem_exec_state == EXEC_PAUSING){
                mem_exec_state = EXEC_CANCELING;
                res = cycle_tp_get_first(velplan_mode, ncu_cycle, current_vel, current_pos);
                CHECK_RES(mem_check_res(res));
                res = pausing_tp_init(velplan_mode, ncu_cycle, current_vel);
                CHECK_RES(mem_check_res(res));
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 1, 0);
            }else if(mem_exec_state == EXEC_WAITING || mem_exec_state == EXEC_IDLE){
                mem_exec_state = EXEC_IDLE;        
                rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
                setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
                setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
                // 复位，译码、预处理、前瞻队列均重置
                tp_reset();
                // 重置当前译码行
                setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
            }
            // 通知译码复位
            rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
            break;
        
        case SYS_EVENT_RERUN:
            // log_system("[mem] 记忆模式处理自动运行");
            LOG_OUTPUT(INFO, "记忆模式处理自动运行");
            // 上一个状态为空闲或复位
            if(mem_exec_state == EXEC_IDLE && queue_is_empty(plan_queue)){
                //　通知译码运行，初始化译码器
                rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, true);
                // 译码正在运行，译码结束译码器将其置false
                rt_set_chan_state(0, CHAN_IS_PARSERING, true);
                mem_exec_state = EXEC_WAITING;
                setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
            }
            // 上一个状态或当前状态为暂停
            else if((mem_exec_state == EXEC_IDLE || mem_exec_state == EXEC_PAUSING)
                    && !queue_is_empty(plan_queue)){
                mem_exec_state = EXEC_WAITING;
            }
            break;
        
        case SYS_EVENT_PAUSE:
            // log_system("[mem] 记忆模式处理暂停");
            LOG_OUTPUT(INFO, "记忆模式处理暂停");
            if(mem_exec_state == EXEC_RUNNING){
                mem_exec_state = EXEC_PAUSING;
                // log_system("[mem] 进给保持运行行号(%d)", cycle_tp->plan_block->line_number);
                LOG_OUTPUT(WARN, "进给保持运行行号 %d", cycle_tp->plan_block->line_number);
                res = cycle_tp_get_first(velplan_mode, ncu_cycle, current_vel, current_pos);
                CHECK_RES(mem_check_res(res));
                res = pausing_tp_init(velplan_mode, ncu_cycle, current_vel);
                CHECK_RES(mem_check_res(res));
            }
            break;
        
        case SYS_EVENT_X_NEG_PRESS:
        case SYS_EVENT_Y_NEG_PRESS:
        case SYS_EVENT_Z_NEG_PRESS:
        case SYS_EVENT_X_POS_PRESS:
        case SYS_EVENT_Y_POS_PRESS:
        case SYS_EVENT_Z_POS_PRESS:
        default:
            // log_system("[mem] 记忆模式不支持该操作");
            LOG_OUTPUT(INFO, "记忆模式不支持该操作");
            break;
    }
    rt_data->nc_event = SYS_EVENT_NONE;
}

// 计算插补距离
static double mem_cal_pinc(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_vel, POS current_pos){
    double interp_distance = 0;
    GNC_RES res = GNC_CAL_SUCCESS;
    // 初始化默认为 IDLE
    switch(mem_exec_state){
        case EXEC_IDLE:
            memset(traj_data, 0, sizeof(TRAJ_PARAMS));
            break;

        case EXEC_WAITING:
            res = cycle_tp_get_first(velplan_mode, ncu_cycle, current_vel, current_pos);
            CHECK_RES(mem_check_res(res));
            break;

        case EXEC_RUNNING:
            // 完成一段的插补
            if (tp_is_arrived(cycle_tp)){
                // 预留：如果单段使能，跳转到 IDLE，等待下一次 run 事件 
                res = cycle_tp_get_next(velplan_mode, ncu_cycle, current_vel);
                CHECK_RES(mem_check_res(res));
            }
            res = tp_params_update(velplan_mode, ncu_cycle, cycle_tp, traj_data);
            CHECK_RES(mem_check_res(res));
            interp_distance = cycle_tp->accum_distance;
            break;

        // 执行暂停中
        case EXEC_PAUSING:
            // 检查是否完成该段的减速运动
            if (tp_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
                    break;
                }else{
                    res = cycle_tp_get_next(velplan_mode, ncu_cycle, current_vel);
                    CHECK_RES(mem_check_res(res));
                    res = pausing_tp_init(velplan_mode, ncu_cycle, current_vel);
                    CHECK_RES(mem_check_res(res));
                }
            }
            res = tp_params_update(velplan_mode, ncu_cycle, pausing_tp, traj_data);
            CHECK_RES(mem_check_res(res));
            interp_distance = pausing_tp->accum_distance;
            break;

        case EXEC_CANCELING:
            // 检查是否完成该段的减速运动
            if (tp_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
                    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
                    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
                    setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
                    // 复位，译码、预处理、前瞻队列均重置
                    tp_reset();
                    setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                    break;
                }else{
                    res = cycle_tp_get_next(velplan_mode, ncu_cycle, current_vel);
                    CHECK_RES(mem_check_res(res));
                    res = pausing_tp_init(velplan_mode, ncu_cycle, current_vel);
                    CHECK_RES(mem_check_res(res));
                }
            }
            res = tp_params_update(velplan_mode, ncu_cycle, pausing_tp, traj_data);
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
            setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
            // log_system("[mem] 程序开始运行");
            LOG_OUTPUT(INFO, "程序开始运行");
        }
        return true;

    case GNC_DATA_EMPTY:
        // 等待中无数据，则继续等待
        if(mem_check_running() && mem_exec_state == EXEC_WAITING){
            rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
            return true;
        }   
        // 运行中无数据，则进入等待 
        if(mem_check_running() && mem_exec_state == EXEC_RUNNING){
            mem_exec_state = EXEC_WAITING;
            rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
            return false;
        }
        // 译码已结束后无数据，则程序运行完成
        mem_exec_state = EXEC_IDLE;
        rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
        rt_set_chan_state(0, CHAN_IS_GCODE_END, false);
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);
        tp_reset();
        // log_system("[mem] 程序运行结束");
        LOG_OUTPUT(INFO, "程序运行结束");
        return false;

    case GNC_DATA_NULL:
    case GNC_PARAM_UNVALID:
    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    default:
        mem_exec_state = EXEC_IDLE;
        setVarChanInt(0, VAR_CHAN_IS_ESTOP, 1, 0);
        setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
        setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
        // log_system("[mem] 记忆模式报警(%d)", res);
        LOG_OUTPUT(ERROR, "记忆模式报警 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 3);
        return false;
    }
}

static bool mem_check_running(){
    if(rt_get_chan_state(0, CHAN_IS_PARSERING)) return true;
    return (!queue_is_empty(plan_queue) || !queue_is_empty(pretreat_queue) || !gcode_queue_is_empty(gcode_queue));
}

static GNC_RES cycle_tp_get_first(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_feedrate, POS current_pos){
    cycle_tp_block_update(cycle_tp->plan_block, current_pos, current_feedrate);
    if(chan_look_ahead(velplan_mode, ncu_cycle, current_feedrate) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    if(cycle_tp_init(velplan_mode, ncu_cycle, current_feedrate) == GNC_CAL_ERROR) return GNC_CAL_ERROR;
    // printf("first pos: %f %f %f", cycle_tp->plan_block->start_pos.x, cycle_tp->plan_block->start_pos.y, cycle_tp->plan_block->start_pos.z);
    // printf("first seg end pos: %f %f %f", cycle_tp->plan_block->end_pos.x, cycle_tp->plan_block->end_pos.y, cycle_tp->plan_block->end_pos.z);
    shm_var_update_prog();
    return GNC_CAL_SUCCESS;
}

static GNC_RES cycle_tp_get_next(VELPLAN_TYPE velplan_mode, double ncu_cycle, double current_feedrate){
    // 1、弹出首段 2、前瞻 3、初始化
    if(queue_pop(plan_queue) != GNC_OPERATOR_SUCCESS) return GNC_DATA_EMPTY;
    if(chan_look_ahead(velplan_mode, ncu_cycle, current_feedrate) == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    if(cycle_tp_init(velplan_mode, ncu_cycle, current_feedrate) != GNC_CAL_SUCCESS) return GNC_CAL_ERROR;
    shm_var_update_prog();
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
    if(tp->interp_cycle > tp->vel_profile->t_cycle) return true;
    return false;
}

static GNC_RES cycle_tp_init(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs){
    GNC_RES res = GNC_CAL_SUCCESS;
    cycle_tp->plan_block = queue_peak(plan_queue); // 获取首段
    cycle_tp->interp_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    res = vp_cal_vel_profile(velplan_mode, cycle_tp->plan_block, cycle_tp->vel_profile, ncu_cycle); // 计算速度曲线
    if(res != GNC_CAL_SUCCESS) LOG_ERROR("vp cal vel profile fail%d\n",res);
    return res;
}

static GNC_RES pausing_tp_init(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs){
    GNC_RES res = GNC_CAL_SUCCESS;
    vs = fabs(vs);
    double remain_s = cycle_tp->remain_distance;
    // 预留：暂停和复位的 j、a、v 速度参数可以另外设置
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    memcpy(pausing_tp->plan_block, cycle_tp->plan_block, sizeof(PLAN_BLOCK));

    // 计算减速距离
    double decline_s = vp_cal_decel_dist(velplan_mode, vs, 0, amax, jmax);
    if(IsLessThan(decline_s, 0)){
        LOG_ERROR("decline_s < 0");
        return GNC_CAL_ERROR;
    }

    if (decline_s > remain_s){ // 在当前段无法减速到0 
        pausing_tp->plan_block->start_vel = vs; 
        pausing_tp->plan_block->end_vel = vp_cal_decel_vel(velplan_mode, vs, amax, jmax, remain_s);   
        if(pausing_tp->plan_block->end_vel < 0) return GNC_CAL_ERROR;  
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = remain_s;
        is_paused_in_block = false;
        res = vp_cal_vel_profile(velplan_mode, pausing_tp->plan_block, pausing_tp->vel_profile, ncu_cycle);
        if(res != GNC_CAL_SUCCESS) LOG_ERROR("decline_s > remain_s, vp cal vel profile fail!");
    }
    else{ // 在当前段可以减速到0
        pausing_tp->plan_block->start_vel = vs;
        pausing_tp->plan_block->end_vel = 0;
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = decline_s;
        is_paused_in_block = true;
        res = vp_cal_vel_profile(velplan_mode, pausing_tp->plan_block, pausing_tp->vel_profile, ncu_cycle);
        if(res != GNC_CAL_SUCCESS) LOG_ERROR("decline_s <= remain_s, vp cal vel profile fail!");
    }
    pausing_tp->interp_cycle = 1;
    pausing_tp->accum_distance = 0;  
    pausing_tp->remain_distance = pausing_tp->plan_block->length;
    pausing_tp->current_distance = 0;
    pausing_tp->target_distance = 0;
    return res;
}

static GNC_RES tp_params_update(VELPLAN_TYPE velplan_mode, double cycle_time, COORD_TP *tp, TRAJ_PARAMS* traj_data){
    vp_cal_traj_data(velplan_mode, tp->vel_profile, tp->interp_cycle, traj_data, cycle_time);
    tp->current_distance = tp->target_distance;
    tp->target_distance = traj_data->s - tp->accum_distance;
    tp->accum_distance = traj_data->s;
    tp->remain_distance = tp->plan_block->length - tp->accum_distance;
    tp->interp_cycle += 1;
    if(IsLessThan(tp->target_distance, 0)){
        LOG_ERROR("tp->target_distance < 0");
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


static POS mem_cal_pos(POS current_pos, double accum_distance){
    if(accum_distance <= 0) return current_pos;
    PLAN_BLOCK* current_block = queue_peak(plan_queue);
    if(current_block == NULL) return;
    POS interp_pos;
    switch(current_block->type){
        case RAPID:
            interp_pos = interpolate_rapid(current_block, accum_distance);
            break;
        case LINE:
            interp_pos = interpolate_line(current_block, accum_distance);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            interp_pos = interpolate_circle(current_pos, current_block, accum_distance);
            break;
        default:
            interp_pos = current_pos;
            break;
    }
    return interp_pos;
}

// 计算直线段插补点
static POS interpolate_line(PLAN_BLOCK* current_block, double accum_distance) {
    POS interp_pos;
    interp_pos.x = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    interp_pos.y = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    interp_pos.z = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
    return interp_pos;
}

// 计算快速段插补点
static POS interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance) {
    POS interp_pos;
    interp_pos.x = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    interp_pos.y = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    interp_pos.z = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
    return interp_pos;
}

// 计算圆弧段插补点
static POS interpolate_circle(POS current_pos, PLAN_BLOCK* current_block, double accum_distance) {
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
    POS interp_pos;
    switch(current_block->circular_block.plane){
        case PLANE_XY:
            interp_pos.x = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.y = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            interp_pos.z = current_pos.z;
            break;
        case PLANE_XZ:
            interp_pos.x = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.y = current_pos.y;
            interp_pos.z = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            break;
        case PLANE_YZ:
            interp_pos.x = current_pos.x;
            interp_pos.y = current_block->circular_block.center_pos.y + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.z = current_block->circular_block.center_pos.z + current_block->circular_block.radius * sin(angle_interp);
            break;
        default:
            interp_pos.x = current_pos.x;
            interp_pos.y = current_pos.y;
            interp_pos.z = current_pos.z;
            break;
    }
    return interp_pos;
}

GNC_RES chan_look_ahead(VELPLAN_TYPE velplan_mode, double ncu_cycle, double vs){
    vs = fabs(vs);
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
        if(pretreat_block == NULL) return GNC_DATA_NULL;
        // 注意：如果长度为0，删除该段
        if(IsEqualTo(pretreat_block->length, 0)){
            queue_pop(pretreat_queue);
            if(queue_is_empty(plan_queue) && queue_is_empty(pretreat_queue) && gcode_queue_is_empty(gcode_queue)){
                rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
            }
            continue;
        }
        queue_push(plan_queue, pretreat_block);
        queue_pop(pretreat_queue);
    }

    if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;

    /******************************************
    1. 初始化首末速度，从末端往前计算，每次前瞻的末段末速度为0，plan_queue->next_vel = 0
    2. 只有两段以上才进行遍历 i > 0
    3. 当为G01时，首段初速度会修正为前一段的末速度；其他类型的初速度为0 */
    plan_queue->next_vel = 0;
    // 两段及以上，初始化首末速度
    for(int i = plan_queue->size - 1; i > 0; i--){
        cur_block = queue_get_block(plan_queue, i);
        if(cur_block == NULL) return GNC_DATA_NULL;
        pre_block = queue_get_block(plan_queue, i - 1);
        if(pre_block == NULL) return GNC_DATA_NULL;
        block_cal_lookahead_vel(cur_block, pre_block->type, plan_queue->next_vel);
        plan_queue->next_vel = cur_block->start_vel;
    }
    // 修正首段速度
    cur_block = queue_peak(plan_queue);
    block_cal_lookahead_vel(cur_block, plan_queue->pre_head_type, plan_queue->next_vel);
    cur_block->start_vel = vs;
    // 判断是否大于最大速度
    if(cur_block->start_vel > cur_block->max_vel) cur_block->max_vel = cur_block->start_vel;
    /*******************************************/
    vp_cal_bid_lookahead(velplan_mode, plan_queue, ncu_cycle);
    return GNC_CAL_SUCCESS;
}

// 计算段的首末速度、最大速度、加速度、加加速度
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel){
    switch(current_block->type){
        case RAPID:
            current_block->max_vel = current_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f;
            current_block->start_vel = 0;
            current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            current_block->max_vel = current_block->feed_vel * var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f;
            // 预留：如果开启单段，首末速度为0
            current_block->end_vel = next_start_vel;
            if(rt_data->sys_state[SYS_IS_LOOKAHEAD_ON] && (pre_type == LINE || pre_type == CIRCLE_CCW || pre_type == CIRCLE_CW)){
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
