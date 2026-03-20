#include "homing.h"
#include "datatype.h"
#include "global.h"
#include "log.h"
#include <alarmapi.h>

static void homing_set_state();
static void homing_is_all_homed();
static void homing_is_resetting();
static void abs_homing_do_home(AXIS_TYPE axis_type);
static void relative_homing_do_home(AXIS_TYPE axis_type);
static bool abs_homing_is_tp_finished(AXIS_TYPE axis_type);
static bool abs_homing_is_axis_homed(AXIS_TYPE axis_type);
static double abs_homing_tp_pinc(AXIS_TYPE axis_type);
static bool homing_check_res(GNC_RES res);
static void homing_handle_alarm();

GNC_RES abs_homing_tp_update_params(AXIS_TYPE axis_type, double current_vel, double target_vel);
void abs_homing_tp_update_traj(VELPLAN_TYPE velplan_type, double cycle_time, AXIS_TYPE axis_type);
static bool homing_tp_new_vel_valid(AXIS_TYPE axis_type, double target_vel);
static GNC_RES abs_homing_tp_init(AXIS_TYPE axis_type);
static GNC_RES abs_declining_tp_init(AXIS_TYPE axis_type, double target_vel);

void handle_homing_cmd(){
    homing_set_state();
    for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
        switch(par_axis[axis_type][PAR_AX_HOME_WAY].value.val_uint){
            case ABSOLUTE_AXIS_HOMING:
                abs_homing_do_home(axis_type);
                break;
            case RELATIVE_AXIS_HOMING:
                // 预留：相对式回零
                relative_homing_do_home(axis_type);
                break;
        }
        // 更新速度参数
        setVarAxisDouble(axis_type, VAR_AXIS_CMD_VEL, homing_tp[axis_type].traj_data->v, 0);
        rt_data->axis_data[axis_type].cmd_acc = homing_tp[axis_type].traj_data->a;
        rt_data->axis_data[axis_type].cmd_jk = homing_tp[axis_type].traj_data->j;
    }
    homing_is_all_homed();
    homing_is_resetting();
}

static void homing_set_state(){
    // 处理报警，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING]){
        homing_handle_alarm();
        return;
    }
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // log_system("[home] 回零处理复位");
            LOG_OUTPUT(INFO, "回零处理复位");
            for(int i = 0; i < SYS_AXES_NUM; i++){
                if(homing_tp[i].homing_state == ABS_EXEC_HOMING){
                    homing_tp[i].is_event_trigger = true;
                    homing_tp[i].homing_state = ABS_EXEC_RESETTING;
                }
            }        
            break;

        case SYS_EVENT_X_POS_PRESS:
            // log_system("[home] 回零处理X+按下");
            LOG_OUTPUT(INFO, "回零处理X+按下");
            if(rt_data->sys_state[SYS_IS_SWITCHING_MODE]) break; // 换挡模式下，不允许操作
            if(homing_tp[AXIS_X].homing_state == ABS_EXEC_IDLE){
                homing_tp[AXIS_X].is_event_trigger = true;  
                homing_tp[AXIS_X].homing_state = ABS_EXEC_HOMING;   
            }
            break;

        case SYS_EVENT_Y_POS_PRESS:
            // log_system("[home] 回零处理Y+按下");
            LOG_OUTPUT(INFO, "回零处理Y+按下");
            if(rt_data->sys_state[SYS_IS_SWITCHING_MODE]) break;
            if(homing_tp[AXIS_Y].homing_state == ABS_EXEC_IDLE){
                homing_tp[AXIS_Y].is_event_trigger = true;
                homing_tp[AXIS_Y].homing_state = ABS_EXEC_HOMING;
            }
            break;

        case SYS_EVENT_Z_POS_PRESS:
            // log_system("[home] 回零处理Z+按下");
            LOG_OUTPUT(INFO, "回零处理Z+按下");
            if(rt_data->sys_state[SYS_IS_SWITCHING_MODE]) break;
            if(homing_tp[AXIS_Z].homing_state == ABS_EXEC_IDLE){
                homing_tp[AXIS_Z].is_event_trigger = true;
                homing_tp[AXIS_Z].homing_state = ABS_EXEC_HOMING;
            }
            break;
        
        case SYS_EVENT_RAPID_OVERRIDE:
            // log_system("[home] 回零处理快速进给倍率改变");
            LOG_OUTPUT(INFO, "回零处理快速进给倍率改变");
            for(int i = 0; i < SYS_AXES_NUM; i++){
                if(homing_tp[i].homing_state == ABS_EXEC_HOMING){
                    homing_tp[i].is_event_trigger = true;
                }
            }    
            break;

        case SYS_EVENT_PAUSE:
        case SYS_EVENT_RERUN:
            LOG_OUTPUT(INFO, "回零模式不支持此操作");
            break;
        
        default:
            break;
    }
    rt_data->nc_event = SYS_EVENT_NONE;
}

static bool homing_check_res(GNC_RES res){
    switch (res)
    {
    case GNC_CAL_SUCCESS:
    case GNC_OPERATOR_SUCCESS:
    case VP_CAL_SUCCESS:
        return true;
    case VP_MAX_VEL_IS_ZERO:
        return false;

    case GNC_DATA_NULL:
    case GNC_PARAM_UNVALID:
    case VP_IS_NULL:
    case VP_PARAM_UNVALID:
    case VP_LENGTH_IS_ZERO:
    case VP_FEED_RATE_IS_ZERO:
    case VP_LENGTH_UNVALID:
        homing_handle_estop();
        // log_system("[home] 回零错误(%d)", res);
        LOG_OUTPUT(ERROR, "回零错误 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 0);
        return false;

    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    case VP_CAL_ERROR:
    case VP_CAL_S_ERROR:
    case VP_CAL_V_ERROR:
    default:
        homing_handle_estop();
        // log_system("[home] 回零错误(%d)", res);
        LOG_OUTPUT(ERROR, "回零错误 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 1);
        return false;
    }
}

void homing_handle_estop(){
    homing_tp_reset();
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
}

static void homing_handle_alarm(){
    for(int i = 0; i < SYS_AXES_NUM; i++){
        if(homing_tp[i].homing_state == ABS_EXEC_HOMING){
            homing_tp[i].is_event_trigger = true;
            homing_tp[i].homing_state = ABS_EXEC_RESETTING;
            LOG_INFO("回零下报警，开始制动");
        }
    }
}

static double abs_homing_tp_pinc(AXIS_TYPE axis_type){
    GNC_RES res = GNC_CAL_SUCCESS;
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
    double rapid_vel = par_axis[axis_type][PAR_AX_RAPID_SPD].value.val_real;
    double rapid_override = var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f;
    double current_vel = fabs(var_axis[axis_type][VAR_AXIS_CMD_VEL].value.f);
    double s = 0;
    switch(homing_tp[axis_type].homing_state){
        case ABS_EXEC_IDLE:
            break; 

        case ABS_EXEC_HOMING:
            // 检查是否执行完速度曲线
            if(abs_homing_is_tp_finished(axis_type) || homing_tp[axis_type].is_event_trigger){
                homing_tp[axis_type].is_event_trigger = false;
                // 检查是否回零
                if(abs_homing_is_axis_homed(axis_type)){
                    homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                    setVarAxisInt(0, VAR_AXIS_IS_HOMEF, 1, 0);
                    LOG_INFO("轴%d完成回零", axis_type);
                    break;
                }
                homing_tp[axis_type].max_vel = rapid_vel * rapid_override;
                res = abs_homing_tp_update_params(axis_type, current_vel, homing_tp[axis_type].max_vel);
                CHECK_RES(homing_check_res(res));
                var_chan[0][VAR_CHAN_IS_HOMING].value.i = true;
            }
            // 进行回零 
            abs_homing_tp_update_traj(T_SHAPE, ncu_cycle, axis_type);
            s = homing_tp[axis_type].target_distance;
            break;

        case ABS_EXEC_RESETTING:
            // 复位事件
            if(homing_tp[axis_type].is_event_trigger){
                homing_tp[axis_type].is_event_trigger = false;
                res = abs_homing_tp_update_params(axis_type, current_vel, 0);
                CHECK_RES(homing_check_res(res));
                homing_tp[axis_type].is_resetting = true;
            }
            // 检查是否完成减速
            if(abs_homing_is_tp_finished(axis_type)){
                homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                homing_tp[axis_type].is_resetting = false;
                break;
            }
            // 进行回零 
            abs_homing_tp_update_traj(T_SHAPE, ncu_cycle, axis_type);
            s = homing_tp[axis_type].target_distance;
            break;

        default:
            // error
            break;
    };
    if(homing_tp[axis_type].homing_direction == AXIS_HOMING_NEGATIVE) s = -s;
    return s;
}

static void abs_homing_do_home(AXIS_TYPE axis_type){
    double axis_cmd_pos = var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f;
    axis_cmd_pos = axis_cmd_pos + abs_homing_tp_pinc(axis_type);
    setVarAxisDouble(axis_type, VAR_AXIS_CMD_POS_EX, axis_cmd_pos, 0);
}

static void relative_homing_do_home(AXIS_TYPE axis_type){
    // 预留：相对式回零
}

static void homing_is_all_homed(){
    for(int i = 0; i < SYS_AXES_NUM; i++){
        switch(par_axis[i][PAR_AX_HOME_WAY].value.val_uint){
            case ABSOLUTE_AXIS_HOMING:
                if(homing_tp[i].homing_state != ABS_EXEC_IDLE) return;
                break;
            case RELATIVE_AXIS_HOMING:
                break;
        }
    }
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
}

static void homing_is_resetting(){
    bool is_resetting = homing_tp[AXIS_X].is_resetting || homing_tp[AXIS_Y].is_resetting || homing_tp[AXIS_Z].is_resetting;
    setVarChanInt(0, VAR_CHAN_IS_RESETTING, is_resetting, 0);
}

// 检查是否执行完速度曲线
static bool abs_homing_is_tp_finished(AXIS_TYPE axis_type){
    if(homing_tp[axis_type].interp_cycle > homing_tp[axis_type].homing_profile->t_cycle) return true;
    return false;
}

// 检查轴是否已回零
static bool abs_homing_is_axis_homed(AXIS_TYPE axis_type){
    double home_crds = par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real;
    double current_pos = var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f;
    if (fabs(home_crds - current_pos) < HOME_TOL) return true;
    return false;
}

void homing_tp_init(){
    homing_tp = (HOMING_TP*)calloc(SYS_AXES_NUM * sizeof(HOMING_TP), 41);
    for(int i = 0; i < SYS_AXES_NUM; i++){
        homing_tp[i].homing_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
        homing_tp[i].homing_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
        homing_tp[i].traj_data = (TRAJ_PARAMS*)calloc(sizeof(TRAJ_PARAMS), 1);
        homing_tp[i].homing_block->type = LINE;
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
        homing_tp[i].homing_block->max_vel = par_axis[i][PAR_AX_RAPID_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f;
        homing_tp[i].homing_block->feed_vel = par_axis[i][PAR_AX_RAPID_SPD].value.val_real;
        homing_tp[i].homing_block->max_acc = par_axis[i][PAR_AX_RAPID_ACC].value.val_real;
        homing_tp[i].homing_block->max_jerk = par_axis[i][PAR_AX_RAPID_JK].value.val_real;
        homing_tp[i].homing_block->end_vel = 0;
        homing_tp[i].homing_direction = AXIS_HOMING_POSITIVE;
    }
}

void homing_tp_reset(){
    for(int i = 0; i < SYS_AXES_NUM; i++){
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
        memset(homing_tp[i].traj_data, 0, sizeof(TRAJ_PARAMS));
    }
}

GNC_RES abs_homing_tp_update_params(AXIS_TYPE axis_type, double current_vel, double target_vel){
    GNC_RES res = VP_CAL_SUCCESS;
    // 如果新速度不满足，不更新新速度
    if(!homing_tp_new_vel_valid(axis_type, target_vel)) return res;
    homing_tp[axis_type].homing_block->max_vel = par_axis[axis_type][PAR_AX_RAPID_SPD].value.val_real * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f;
    homing_tp[axis_type].homing_block->length = fabs(par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real - var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f); // 设置的x轴零点与当前位置之间的距离
    homing_tp[axis_type].homing_block->start_vel = fabs(var_axis[axis_type][VAR_AXIS_CMD_VEL].value.f);  // 当前速度
    homing_tp[axis_type].homing_distance = homing_tp[axis_type].homing_block->length;
    if(current_vel > homing_tp[axis_type].homing_block->max_vel) homing_tp[axis_type].homing_block->max_vel = current_vel;
    // 如果加工段允许最大速度为0（倍率为0）
    if(IsEqualTo(homing_tp[axis_type].homing_block->max_vel, 0)) return VP_MAX_VEL_IS_ZERO; 
    if(current_vel > target_vel){
        res = abs_declining_tp_init(axis_type, target_vel);
    }else{
        res = abs_homing_tp_init(axis_type);
    }
    if(res != VP_CAL_SUCCESS)  return res;
    // if(IsEqualTo(homing_tp[axis_type].homing_profile->ve, 0)) 
    if(IsEqualTo(homing_tp[axis_type].homing_block->length, homing_tp[axis_type].homing_distance)) homing_tp[axis_type].is_finish_at_the_end = true;
    else homing_tp[axis_type].is_finish_at_the_end = false;
    return res;
}

void abs_homing_tp_update_traj(VELPLAN_TYPE velplan_type, double cycle_time, AXIS_TYPE axis_type){
    vp_cal_traj_data(velplan_type, homing_tp[axis_type].homing_profile, homing_tp[axis_type].interp_cycle, homing_tp[axis_type].traj_data, cycle_time);
    homing_tp[axis_type].current_distance = homing_tp[axis_type].target_distance;
    homing_tp[axis_type].target_distance = homing_tp[axis_type].traj_data->s - homing_tp[axis_type].accum_distance;
    homing_tp[axis_type].accum_distance = homing_tp[axis_type].traj_data->s;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length - homing_tp[axis_type].accum_distance;
    homing_tp[axis_type].interp_cycle += 1;
}

static bool homing_tp_new_vel_valid(AXIS_TYPE axis_type, double target_vel){
    double vs = fabs(var_axis[axis_type][VAR_AXIS_CMD_VEL].value.f); double ve = 0;
    double remain_s = fabs(par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real - var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f); ;
    double total_decline_length = 0;
    if(vs > target_vel && target_vel >= ve){
        double length1 = vp_cal_decel_dist(T_SHAPE, vs, target_vel, homing_tp[axis_type].homing_block->max_acc, homing_tp[axis_type].homing_block->max_jerk);
        double length2 = vp_cal_decel_dist(T_SHAPE, target_vel, ve, homing_tp[axis_type].homing_block->max_acc, homing_tp[axis_type].homing_block->max_jerk);
        total_decline_length = length1 + length2; 
    }else if(target_vel > vs && vs >= ve){
        total_decline_length = vp_cal_decel_dist(T_SHAPE, vs, ve, homing_tp[axis_type].homing_block->max_acc, homing_tp[axis_type].homing_block->max_jerk);
    }
    if(IsGreaterThanOrEqual(total_decline_length, remain_s)) return false;
    return true;
}

static GNC_RES abs_homing_tp_init(AXIS_TYPE axis_type){
    GNC_RES res = VP_CAL_SUCCESS;
    if(IsEqualTo(homing_tp[axis_type].max_vel, 0)) return res; // 如果倍率为0，且未开始回零
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, ncu_cycle); // 速度控制方式：T型、S型、SIN型
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if(par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real > var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f) homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
    else homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    // if(res != VP_CAL_SUCCESS) LOG_OUTPUT(ERROR, "vp cal vel profile fail");
    return res;
}

static GNC_RES abs_declining_tp_init(AXIS_TYPE axis_type, double target_vel){
    GNC_RES res = VP_CAL_SUCCESS;
    double vs = homing_tp[axis_type].homing_block->start_vel; double ve = 0;
    double amax = homing_tp[axis_type].homing_block->max_acc;
    double jmax = homing_tp[axis_type].homing_block->max_jerk;
    double ncu_cycle = (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S;
    homing_tp[axis_type].homing_block->end_vel = target_vel;
    homing_tp[axis_type].homing_block->length = vp_cal_decel_dist(T_SHAPE, vs, ve, amax, jmax); // 减速距离
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, ncu_cycle); // 速度控制方式：T型、S型、SIN型
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if(par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real > var_axis[axis_type][VAR_AXIS_CMD_POS_EX].value.f) homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
    else homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    // if(res != VP_CAL_SUCCESS) LOG_OUTPUT(ERROR, "vp cal vel profile fail");
    return res;
}