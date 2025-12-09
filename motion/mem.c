#include "mem.h"
#include "interpolation.h"

static bool check_is_arrived(MEM_TP *tp);
static GNC_RES cycle_param_init(PARAMS *params, double vs);
static GNC_RES pausing_param_init(PARAMS *params, double vs);
static double cal_interp_params(VELPLAN_TYPE velplan_type, PARAMS *params, MEM_TP *tp);
static void update_plan_block(PLAN_BLOCK* current_block, POS *current_pos, double current_vel, double target_distance);
static void set_mem_state();
static double cal_interp_distance();

// MEM 模式下的执行流程：处理开始和暂停 → 计算插补距离 → 计算插补空间坐标
void handle_run_cmd(){
    static double interp_pin = 0;
    POS cur_pos = {0};
    // 设置自动加工状态
    set_mem_state();
    // 计算插补距离
    interp_pin = cal_interp_distance();
    // 更新：【插补功能】张哲
    //根据插补类型计算插补点：快速G00、直线G01、圆弧G02|03……
  // printf("cycle_tp->plan_block->type%d %f\n",cycle_tp->plan_block->type,interp_pin);
    switch (cycle_tp->plan_block->type) {
    case RAPID:
    case LINE:
        tcLine(cycle_tp, interp_pin, &cur_pos);
        break;
    case CIRCLE_CCW:

        break;
    case CIRCLE_CW:

        break;
    case HELIX:

        break;
    case SPLINE:

        break;
    case RIGID_TAP:

        break;
    case PROBE:

        break;
    case UNKNOWN:

        break;
    default:
        break;
    }
    hal->target_pos = cur_pos;
    // 调试：自动加工的插补距离
    hal->interp_pin = interp_pin;
}

static void set_mem_state(){
    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            if(mem_exec_state == EXEC_RUNNING || mem_exec_state == EXEC_PAUSING){
                mem_exec_state = EXEC_CANCELING;
                pausing_param_init(params, hal->current_vel);
            }
            break;
        
        case SYS_EVENT_RERUN:
            if(mem_exec_state == EXEC_IDLE){
                mem_exec_state = EXEC_RUNNING;
                printf("debug: click run\n");
            }
            break;
        
        case SYS_EVENT_PAUSE:
            if(mem_exec_state == EXEC_RUNNING){
                mem_exec_state = EXEC_PAUSING;
                pausing_param_init(params, hal->current_vel);
            }
            break;
        
        case SYS_EVENT_X:
        case SYS_EVENT_Y:
        case SYS_EVENT_Z:
            printf("warning: mem mode is not support this operation\n");
            break;

        default:
            printf("warning: mem mode is not support this operation\n");
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

// 计算插补距离
static double cal_interp_distance(){
    double interp_distance = 0;
    GNC_RES res = GNC_CAL_SUCCESS;
    // 初始化默认为 IDLE
    switch(mem_exec_state){
        case EXEC_IDLE:
            break;

        case EXEC_RUNNING:
            // 完成一段的插补
            if (check_is_arrived(cycle_tp)){
                // 预留：如果单段使能，跳转到 IDLE，等待下一次 run 事件
                cycle_param_init(params, hal->current_vel);      
                if(res == GNC_DATA_EMPTY){
                    mem_exec_state == EXEC_IDLE;
                    printf("debug: plan queue is empty, program end\n");
                    break;
                } 
            }
            interp_distance = cal_interp_params(velplan_type, params, cycle_tp);
            break;

        // 执行暂停中
        case EXEC_PAUSING:
            // 更新该段的起始位置和长度，即当前位置
            update_plan_block(cycle_tp->plan_block, &hal->current_pos, hal->current_vel, pausing_tp->accum_distance);
            // 检查是否完成该段的减速运动
            if (check_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    break;
                }
                else{
                    cycle_param_init(params, hal->current_vel);
                    pausing_param_init(params, hal->current_vel);
                }
            }
            interp_distance = cal_interp_params(velplan_type, params, pausing_tp);
            break;

        case EXEC_CANCELING:
            // 更新该段的起始位置和长度，即当前位置
            update_plan_block(cycle_tp->plan_block, &hal->current_pos, hal->current_vel, pausing_tp->target_distance);
            // 检查是否完成该段的减速运动
            if (check_is_arrived(pausing_tp)) {             
                if(is_paused_in_block == true){ // 如果段内完成暂停                    
                    mem_exec_state = EXEC_IDLE;
                    // 复位，译码、预处理、前瞻队列均重置
                    tp_reset();
                    break;
                }
                else{
                    cycle_param_init(params, hal->current_vel);
                    pausing_param_init(params, hal->current_vel);
                }
            }
            interp_distance = cal_interp_params(velplan_type, params, pausing_tp);
            break;

        default:
            // 更新：【报警模块】翟亚飞
            break;
    }
    return interp_distance;
}

// 更新段起点和速度，重新计算插补段参数
static void update_plan_block(PLAN_BLOCK* current_block, POS *current_pos, double current_vel, double target_distance){
    // memcpy(&current_block->start_pos, current_pos, sizeof(POS));
    // block_cal_length(current_block);
    // block_cal_vector(current_block);
    cycle_tp->plan_block->length = cycle_tp->remain_distance - target_distance;
    cycle_tp->plan_block->start_vel = current_vel;
    cycle_tp->accum_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    cycle_tp->vel_profile = cal_vel_profile(velplan_type, cycle_tp->plan_block); // 计算速度曲线
}

// 检查是否到目标点
static bool check_is_arrived(MEM_TP *tp){
    if(fabs(tp->remain_distance) < END_TOL) return true;
    return false;
}

static GNC_RES cycle_param_init(PARAMS *params, double vs){
    GNC_RES res = chan_look_ahead(vs); // 速度前瞻
    if(res == GNC_DATA_EMPTY) return GNC_DATA_EMPTY;
    cycle_tp->plan_block = queue_pop(plan_queue); // 获取首段
    cycle_tp->accum_cycle = 1;
    cycle_tp->accum_distance = 0;  
    cycle_tp->remain_distance = cycle_tp->plan_block->length;
    cycle_tp->current_distance = 0;
    cycle_tp->target_distance = 0;
    cycle_tp->vel_profile = cal_vel_profile(velplan_type, cycle_tp->plan_block); // 计算速度曲线
    return GNC_CAL_SUCCESS;
}

static GNC_RES pausing_param_init(PARAMS *params, double vs){
    vs = fabs(vs);
    double remain_s = cycle_tp->remain_distance;
    double amax = cycle_tp->plan_block->max_acc;
    double jmax = cycle_tp->plan_block->max_jerk;
    memcpy(pausing_tp->plan_block, cycle_tp->plan_block, sizeof(PLAN_BLOCK));

    // 计算减速距离
    double decline_s = cal_cri_length(params->velplan_type, vs, 0, amax, jmax);
    if(decline_s < 0) return GNC_CAL_ERROR;

    if (decline_s > remain_s){ // 在当前段无法减速到0 
        pausing_tp->plan_block->start_vel = vs; 
        pausing_tp->plan_block->end_vel = cal_end_vel(params->velplan_type, vs, amax, jmax, remain_s);   
        if(pausing_tp->plan_block->end_vel < 0) return GNC_CAL_ERROR;  
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = remain_s;
        is_paused_in_block = false;
        pausing_tp->vel_profile = cal_vel_profile(params->velplan_type, pausing_tp->plan_block);
    }
    else{ // 在当前段可以减速到0
        pausing_tp->plan_block->start_vel = vs;
        pausing_tp->plan_block->end_vel = 0;
        pausing_tp->plan_block->max_vel = vs;
        pausing_tp->plan_block->length = decline_s;
        is_paused_in_block = true;
        pausing_tp->vel_profile = cal_vel_profile(params->velplan_type, pausing_tp->plan_block);
    }
    if (pausing_tp->vel_profile == NULL) return GNC_CAL_ERROR;
    pausing_tp->accum_cycle = 1;
    pausing_tp->accum_distance = 0;  
    pausing_tp->remain_distance = pausing_tp->plan_block->length;
    pausing_tp->current_distance = 0;
    pausing_tp->target_distance = 0;
    return GNC_CAL_SUCCESS;
}

static double cal_interp_params(VELPLAN_TYPE velplan_type, PARAMS *params, MEM_TP *tp){
    double distance = cal_traj_distance(velplan_type, tp->vel_profile, tp->accum_cycle);
    tp->current_distance = tp->target_distance;
    tp->target_distance = distance - tp->accum_distance;
    tp->accum_distance = distance;
    tp->remain_distance = tp->plan_block->length - tp->accum_distance;
    tp->accum_cycle += 1;
    return tp->target_distance;
}

void plan_tp_init(){
    cycle_tp = (MEM_TP*)malloc(sizeof(MEM_TP));
    memset(cycle_tp, 0, sizeof(MEM_TP));
    cycle_tp->plan_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    cycle_tp->vel_profile = (VEL_PROFILE*)malloc(sizeof(VEL_PROFILE));

    pausing_tp = (MEM_TP*)malloc(sizeof(MEM_TP));  
    memset(pausing_tp, 0, sizeof(MEM_TP));  
    pausing_tp->plan_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    pausing_tp->vel_profile = (VEL_PROFILE*)malloc(sizeof(VEL_PROFILE));

    mem_exec_state = EXEC_IDLE;
    is_paused_in_block = false;
}

void plan_tp_reset(){
    memset(cycle_tp, 0, sizeof(MEM_TP));
    memset(pausing_tp, 0, sizeof(MEM_TP));  
    mem_exec_state = EXEC_IDLE;
    is_paused_in_block = false;
}

