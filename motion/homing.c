#include "homing.h"

static void set_homing_state();
static void do_abs_homing();
static void abs_homing_param_init(HOMING_AXIS_TYPE axis_type);
static void abs_cancel_param_init(HOMING_AXIS_TYPE axis_type);
static double update_abs_homing(HOMING_TP *homing_tp, HOMING_AXIS_TYPE axis_type);
static bool check_is_axis_homed(HOMING_AXIS_TYPE axis_type);
static void cal_axis_homing_params(VELPLAN_TYPE velplan_type, PARAMS *params, HOMING_TP *tp);

void handle_homing_cmd(){
    // todo：检查软限位
    switch(params->homing_type){
        case ABSOLUTE_AXIS_HOMING:
            set_homing_state();
            do_abs_homing();
            break;
        case RELATIVE_AXIS_HOMING:
            // 预留：相对式回零
            break;
    }
}

static void do_abs_homing(){
    hal->target_pos.x = hal->current_pos.x + update_abs_homing(x_homing_tp, AXIS_X);
    hal->target_pos.y = hal->current_pos.y + update_abs_homing(y_homing_tp, AXIS_Y);
    hal->target_pos.z = hal->current_pos.z + update_abs_homing(z_homing_tp, AXIS_Z);
    // 调试：显示回零总速度
    hal->interp_pin = POS_Distance(&hal->target_pos, &hal->current_pos) / params->cycle_time;
}

static void set_homing_state(){
    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            if(x_homing_tp->homing_state == ABS_EXEC_HOMING){
                if(check_is_axis_homed(AXIS_X)){
                    x_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                // 创建减速tp
                abs_cancel_param_init(AXIS_X);
                x_homing_tp->homing_state = ABS_EXEC_CANCELING;
            }
            if(y_homing_tp->homing_state == ABS_EXEC_HOMING){
                if(check_is_axis_homed(AXIS_Y)){
                    y_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                abs_cancel_param_init(AXIS_Y);
                y_homing_tp->homing_state = ABS_EXEC_CANCELING;
            }
                
            if(z_homing_tp->homing_state == ABS_EXEC_HOMING){
                if(check_is_axis_homed(AXIS_Z)){
                    z_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                abs_cancel_param_init(AXIS_Z);
                z_homing_tp->homing_state = ABS_EXEC_CANCELING;
            }             
            break;

        case SYS_EVENT_X:
            // todo：复位暂停的过程是否可以再次按下X重新进行回零
            if(sys->is_switching_mode) break; // 换挡模式下，不允许操作
            if(x_homing_tp->homing_state == ABS_EXEC_IDLE){
                if(check_is_axis_homed(AXIS_X)){
                    x_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                // 创建回零TP
                abs_homing_param_init(AXIS_X);
                x_homing_tp->homing_state = ABS_EXEC_HOMING;
                printf("debug: click x homing\n");
            }
            break;

        case SYS_EVENT_Y:
            if(sys->is_switching_mode) break;
            if(y_homing_tp->homing_state == ABS_EXEC_IDLE){
                if(check_is_axis_homed(AXIS_Y)){
                    y_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                abs_homing_param_init(AXIS_Y);
                y_homing_tp->homing_state = ABS_EXEC_HOMING;
                printf("debug: click y homing\n");
            }
            break;

        case SYS_EVENT_Z:
            if(sys->is_switching_mode) break;
            if(z_homing_tp->homing_state == ABS_EXEC_IDLE){
                if(check_is_axis_homed(AXIS_Z)){
                    z_homing_tp->homing_state = ABS_EXEC_IDLE;
                    break;
                }
                abs_homing_param_init(AXIS_Z);
                z_homing_tp->homing_state = ABS_EXEC_HOMING;
                printf("debug: click z homing\n");
            }
            break;

        case SYS_EVENT_RERUN:
        case SYS_EVENT_PAUSE:
            printf("warning: home mode is not support this operation\n");
            break;

        default:
            printf("warning: home mode is not support this operation\n");
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

static double update_abs_homing(HOMING_TP *homing_tp, HOMING_AXIS_TYPE axis_type){
    double s = 0;
    switch(homing_tp->homing_state){
        case ABS_EXEC_IDLE:
            break; 

        case ABS_EXEC_HOMING:
            // 检查是否完成回零
            if(check_is_axis_homed(axis_type)){
                homing_tp->homing_state = ABS_EXEC_IDLE;
                break;
            }
            // 进行回零 
            cal_axis_homing_params(T_SHAPE, params, homing_tp);
            s = homing_tp->target_distance;
            break;

        case ABS_EXEC_CANCELING:
            // 检查是否完成减速
            if(fabs(homing_tp->remain_distance) < HOME_TOL){
                homing_tp->homing_state = ABS_EXEC_IDLE;
                break;
            }
            // 进行回零 
            cal_axis_homing_params(T_SHAPE, params, homing_tp);
            s = homing_tp->target_distance;
            break;

        default:
            // error
            break;
    };
    if(homing_tp->homing_direction = AXIS_HOMING_NEGATIVE) s = -s;
    return s;
};

static void abs_homing_param_init(HOMING_AXIS_TYPE axis_type){
    switch (axis_type)
    {
    case AXIS_X:
        x_homing_tp->homing_block->start_pos = hal->current_pos;
        x_homing_tp->homing_block->end_pos = params->home_pos;
        x_homing_tp->homing_block->length = fabs(params->home_pos.x - hal->current_pos.x); // 设置的x轴零点与当前位置之间的距离
        x_homing_tp->homing_block->start_vel = fabs(hal->x_current_vel);  // todo：根据实际数据修改为当前速度
        x_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, x_homing_tp->homing_block); // 速度控制方式：T型、S型、SIN型
        x_homing_tp->accum_cycle = 1;
        x_homing_tp->accum_distance = 0;
        x_homing_tp->remain_distance = x_homing_tp->homing_block->length; // 单个轴的零点距离
        if(params->home_pos.x > hal->current_pos.x) x_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
        break;
    case AXIS_Y:
        y_homing_tp->homing_block->start_pos = hal->current_pos; 
        y_homing_tp->homing_block->end_pos = params->home_pos;
        y_homing_tp->homing_block->length = fabs(params->home_pos.y - hal->current_pos.y);
        y_homing_tp->homing_block->start_vel = fabs(hal->y_current_vel); 
        y_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, y_homing_tp->homing_block);
        y_homing_tp->accum_cycle = 1;
        y_homing_tp->accum_distance = 0;
        y_homing_tp->remain_distance = y_homing_tp->homing_block->length;
        if(params->home_pos.y > hal->current_pos.y) y_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;
        break;
    case AXIS_Z:
        z_homing_tp->homing_block->start_pos = hal->current_pos; 
        z_homing_tp->homing_block->end_pos = params->home_pos;
        z_homing_tp->homing_block->length = fabs(params->home_pos.z - hal->current_pos.z);
        z_homing_tp->homing_block->start_vel = fabs(hal->z_current_vel); 
        z_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, z_homing_tp->homing_block);
        z_homing_tp->accum_cycle = 1;
        z_homing_tp->accum_distance = 0;
        z_homing_tp->remain_distance = z_homing_tp->homing_block->length;
        if(params->home_pos.z > hal->current_pos.z) z_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;
        break;
    default:
        // 更新：【报警模块】翟亚飞
        break;
    }
}

static void abs_cancel_param_init(HOMING_AXIS_TYPE axis_type){
    switch (axis_type)
    {
    case AXIS_X:
        x_homing_tp->homing_block->start_vel = fabs(hal->x_current_vel);  // todo：根据实际数据修改为当前速度
        // 注意：速度、加速度、加加速度和距离均为非负数，请校验
        x_homing_tp->homing_block->length = cal_cri_length(T_SHAPE, fabs(hal->x_current_vel), 0, x_homing_tp->homing_block->max_acc, x_homing_tp->homing_block->max_jerk); // 减速距离
        // 注意：velplan_type 只有T型、S型、SIN型 三种类型
        x_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, x_homing_tp->homing_block); // 速度控制方式：T型、S型、SIN型
        x_homing_tp->accum_cycle = 1;
        x_homing_tp->accum_distance = 0;
        x_homing_tp->remain_distance = x_homing_tp->homing_block->length; // 单个轴的零点距离
        if(params->home_pos.x > hal->current_pos.x) x_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
        break;
    case AXIS_Y:
        y_homing_tp->homing_block->start_vel = fabs(hal->y_current_vel);
        y_homing_tp->homing_block->length = cal_cri_length(T_SHAPE, fabs(hal->y_current_vel), 0, y_homing_tp->homing_block->max_acc, y_homing_tp->homing_block->max_jerk);
        y_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, y_homing_tp->homing_block);
        y_homing_tp->accum_cycle = 1;
        y_homing_tp->accum_distance = 0;
        y_homing_tp->remain_distance = y_homing_tp->homing_block->length;
        if(params->home_pos.y > hal->current_pos.y) y_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;
        break;
    case AXIS_Z:
        z_homing_tp->homing_block->start_vel = fabs(hal->z_current_vel);
        z_homing_tp->homing_block->length = cal_cri_length(T_SHAPE, fabs(hal->z_current_vel), 0, z_homing_tp->homing_block->max_acc, z_homing_tp->homing_block->max_jerk);
        z_homing_tp->homing_profile = cal_vel_profile(T_SHAPE, z_homing_tp->homing_block);
        z_homing_tp->accum_cycle = 1;
        z_homing_tp->accum_distance = 0;
        z_homing_tp->remain_distance = z_homing_tp->homing_block->length;
        if(params->home_pos.z > hal->current_pos.z) z_homing_tp->homing_direction = AXIS_HOMING_POSITIVE;
        break;
    default:
        // 更新：【报警模块】翟亚飞
        break;
    }
}

static void cal_axis_homing_params(VELPLAN_TYPE velplan_type, PARAMS *params, HOMING_TP *tp){
    double distance = cal_traj_distance(velplan_type, tp->homing_profile, tp->accum_cycle);
    tp->current_distance = tp->target_distance;
    tp->target_distance = distance - tp->accum_distance;
    tp->accum_distance = distance;
    tp->remain_distance = tp->homing_block->length - tp->accum_distance;
    tp->accum_cycle += 1;
}

// 检查轴是否都已回零
static bool check_is_axis_homed(HOMING_AXIS_TYPE axis_type){
    switch (axis_type)
    {
    case AXIS_X:
        if(fabs(hal->current_pos.x - params->home_pos.x) < HOME_TOL) return true;
        break;
    case AXIS_Y:
        if(fabs(hal->current_pos.y - params->home_pos.y) < HOME_TOL) return true;
        break;
    case AXIS_Z:
        if(fabs(hal->current_pos.z - params->home_pos.z) < HOME_TOL) return true;
        break;
    default:
        return false;
    }
    return false;
}

void homing_tp_init(HOMING_TP* homing_tp){
    memset(homing_tp, 0, sizeof(HOMING_TP));
    homing_tp->homing_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    homing_tp->homing_profile = (VEL_PROFILE*)malloc(sizeof(VEL_PROFILE));
    memset(homing_tp->homing_block, 0, sizeof(PLAN_BLOCK));
    memset(homing_tp->homing_profile, 0, sizeof(VEL_PROFILE));
    homing_tp->homing_block->type = LINE;
    homing_tp->homing_state = ABS_EXEC_IDLE;
    homing_tp->homing_block->max_acc = params->homing_acc;
    homing_tp->homing_block->max_vel = params->homing_vel;
    homing_tp->homing_block->max_jerk = params->homing_jerk;
    homing_tp->homing_block->end_vel = 0;
    homing_tp->homing_direction = AXIS_HOMING_POSITIVE;
}

void homing_tp_reset(HOMING_TP* homing_tp){
    homing_tp->homing_state = ABS_EXEC_IDLE;
}