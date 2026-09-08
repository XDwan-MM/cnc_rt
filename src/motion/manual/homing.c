#include "homing.h"
#include "datatype.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <stdlib.h>
#include <string.h>
#include "stored_stroke_check.h"
#include "rt_data.h"

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
static void homing_handle_reset();
static void homing_cal_fv();
GNC_RES abs_homing_tp_update_params(AXIS_TYPE axis_type, double current_vel, double target_vel);
void abs_homing_tp_update_traj(VELPLAN_TYPE velplan_type, double cycle_time, AXIS_TYPE axis_type);
static bool homing_tp_new_vel_valid(AXIS_TYPE axis_type, double target_vel);
static GNC_RES abs_homing_tp_init(AXIS_TYPE axis_type);
static GNC_RES abs_declining_tp_init(AXIS_TYPE axis_type, double target_vel);
static void reset_pos_at_home(AXIS_TYPE axis_type);

void handle_homing_cmd(){
    homing_set_state();
    for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
        switch(rt_data->axis_data[axis_type].home_way){
            case ABSOLUTE_AXIS_HOMING:
                abs_homing_do_home(axis_type);
                break;
            case RELATIVE_AXIS_HOMING:
                // 预留：相对式回零
                relative_homing_do_home(axis_type);
                break;
        }
        // 更新速度参数
        rt_data->axis_data[axis_type].cmd_vel = homing_tp[axis_type].traj_data->v;
        rt_data->axis_data[axis_type].cmd_acc = homing_tp[axis_type].traj_data->a;
        rt_data->axis_data[axis_type].cmd_jk = homing_tp[axis_type].traj_data->j;
    }
    homing_cal_fv();
    homing_is_all_homed();
}

static void homing_cal_fv(){
    rt_data->chan_data[0].cmd_vel = sqrt(pow(homing_tp[0].traj_data->v, 2) + pow(homing_tp[1].traj_data->v, 2) + pow(homing_tp[2].traj_data->v, 2));
}

static void homing_set_state(){
    // 处理报警，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        homing_handle_alarm();
        return;
    }
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;

        // 回零时按复位，856梯形图会产生XYZ释放G100.0~G100.2变为0
        case SYS_EVENT_HMI_RESET:
            LOG(LOG_LEVEL_INFO, "回零处理复位");
            for(int i = 0; i < SYS_AXES_NUM; i++){
                homing_tp[i].is_ref_on = false;
            }        
            break;

        case SYS_EVENT_X_POS_PRESS:
        case SYS_EVENT_X_NEG_PRESS:
            LOG(LOG_LEVEL_INFO, "回零处理X按下");
            if(IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 2);
                LOG(LOG_LEVEL_WARN, "快速倍率为0");
            }else{
                delAlarm(ALARM_SY, ALARM_WARNING, 2);
            }
            homing_tp[AXIS_X].is_ref_on = true;
            break;

        case SYS_EVENT_Y_POS_PRESS:
        case SYS_EVENT_Y_NEG_PRESS:
            LOG(LOG_LEVEL_INFO, "回零处理Y按下");
            if(IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 2);
                LOG(LOG_LEVEL_WARN, "快速倍率为0");
            }else{
                delAlarm(ALARM_SY, ALARM_WARNING, 2);
            }
            homing_tp[AXIS_Y].is_ref_on = true;
            break;

        case SYS_EVENT_Z_POS_PRESS:
        case SYS_EVENT_Z_NEG_PRESS:
            LOG(LOG_LEVEL_INFO, "回零处理Z按下");
            if(IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 2);
                LOG(LOG_LEVEL_WARN, "快速倍率为0");
            }else{
                delAlarm(ALARM_SY, ALARM_WARNING, 2);
            }
            homing_tp[AXIS_Z].is_ref_on = true;
            break;
        
        case SYS_EVENT_X_POS_RELEASE:
        case SYS_EVENT_X_NEG_RELEASE:
            LOG(LOG_LEVEL_INFO, "回零处理X松开");
            homing_tp[AXIS_X].is_ref_on = false;
            break;

        case SYS_EVENT_Y_POS_RELEASE:
        case SYS_EVENT_Y_NEG_RELEASE:
            LOG(LOG_LEVEL_INFO, "回零处理Y松开");
            homing_tp[AXIS_Y].is_ref_on = false;
            break;

        case SYS_EVENT_Z_POS_RELEASE:
        case SYS_EVENT_Z_NEG_RELEASE:
            LOG(LOG_LEVEL_INFO, "回零处理Z松开");
            homing_tp[AXIS_Z].is_ref_on = false;
            break;

        case SYS_EVENT_RAPID_OVERRIDE:
            LOG(LOG_LEVEL_INFO, "回零处理快速进给倍率改变");
            if(IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 2);
                LOG(LOG_LEVEL_WARN, "快速倍率为0");
            }
            for(int i = 0; i < SYS_AXES_NUM; i++){
                if(homing_tp[i].homing_state == ABS_EXEC_HOMING){
                    homing_tp[i].is_event_trigger = true;
                }
            }    
            break;

        case SYS_EVENT_PAUSE:
        case SYS_EVENT_RERUN:
            LOG(LOG_LEVEL_WARN, "回零模式不支持此操作");
            break;
        
        default:
            break;
    }
    //　根据回零状态和方向
    for(int axis_type = 0; axis_type < SYS_AXES_NUM; axis_type++){
        if(homing_tp[axis_type].is_ref_on){
            if(rt_data->sys_state[SYS_IS_SWITCHING_MODE]) break;// 换挡模式下，不允许操作
            if(homing_tp[axis_type].homing_state == ABS_EXEC_IDLE){
                homing_tp[axis_type].is_event_trigger = true;
                homing_tp[axis_type].homing_state = ABS_EXEC_HOMING;
            }
        }else{
            if(homing_tp[axis_type].homing_state == ABS_EXEC_HOMING){
                homing_tp[axis_type].is_event_trigger = true;
                homing_tp[axis_type].homing_state = ABS_EXEC_RESETTING;
            }
        }
    }
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
        LOG(LOG_LEVEL_ERR, "回零错误 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 0);
        return false;

    case GNC_CAL_ERROR:
    case GNC_OPERATOR_ERROR:
    case VP_CAL_ERROR:
    case VP_CAL_S_ERROR:
    case VP_CAL_V_ERROR:
    default:
        homing_handle_estop();
        LOG(LOG_LEVEL_ERR, "回零错误 %d", res);
        setAlarm(ALARM_CH, ALARM_ERROR, 1);
        return false;
    }
}

void homing_handle_estop(){
    homing_tp_reset();
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
}

static void homing_handle_reset(){
    homing_tp_reset();
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
}

static void homing_handle_alarm(){
    for(int i = 0; i < SYS_AXES_NUM; i++){
        if(homing_tp[i].homing_state == ABS_EXEC_HOMING){
            homing_tp[i].is_event_trigger = true;
            homing_tp[i].homing_state = ABS_EXEC_RESETTING;
            homing_tp[i].is_ref_on = false;
            LOG(LOG_LEVEL_WARN, "开始制动");
        }
    }
}

static double abs_homing_tp_pinc(AXIS_TYPE axis_type){
    GNC_RES res = GNC_CAL_SUCCESS;
    double ncu_cycle = rt_data->ncu_data.cycle;
    double rapid_vel = MIN(rt_data->axis_data[0].max_rapid_vel, rt_data->axis_data[0].jog_lowspd);
    double rapid_override = var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
    double current_vel = fabs(rt_data->chan_data[0].cmd_vel);
    double s = 0;
    switch(homing_tp[axis_type].homing_state){
        case ABS_EXEC_IDLE:
            homing_tp[axis_type].traj_data->v = 0;
            break; 

        case ABS_EXEC_HOMING:
            // 检查是否执行完速度曲线
            if(abs_homing_is_tp_finished(axis_type) || homing_tp[axis_type].is_event_trigger){
                homing_tp[axis_type].is_event_trigger = false;
                // 检查是否回零
                if(abs_homing_is_axis_homed(axis_type)){
                    homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                    homing_tp[axis_type].is_ref_on = false;
                    // 判断是否在允差范围内到达零点
                    set_ref_state(FIRST_REF, axis_type);
                    reset_pos_at_home(axis_type);
                    LOG(LOG_LEVEL_INFO, "轴%d完成回零", axis_type);
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
                if(res == VP_MAX_VEL_IS_ZERO || res == VP_LENGTH_IS_ZERO){
                    homing_handle_reset();
                    break;
                }
                CHECK_RES(homing_check_res(res));
                homing_tp[axis_type].is_resetting = true;
            }
            // 检查是否完成减速
            if(abs_homing_is_tp_finished(axis_type)){
                homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                homing_tp[axis_type].is_resetting = false;
                homing_tp[axis_type].is_ref_on = false;
                manual_check_soft_limit_del_done(axis_type);
                break;
            }
            // 进行回零 
            abs_homing_tp_update_traj(T_SHAPE, ncu_cycle, axis_type);
            s = homing_tp[axis_type].target_distance;
            break;

        default:
            break;
    };
    if(homing_tp[axis_type].homing_direction == AXIS_HOMING_NEGATIVE) s = -s;
    return s;
}

static void abs_homing_do_home(AXIS_TYPE axis_type){
    double axis_homing_pinc = abs_homing_tp_pinc(axis_type);
    rt_data->axis_data[axis_type].wcs_cmd_pos_cur += axis_homing_pinc;
}

static void relative_homing_do_home(AXIS_TYPE axis_type){
    // 预留：相对式回零
}

static void homing_is_all_homed(){
    for(int i = 0; i < SYS_AXES_NUM; i++){
        switch(rt_data->axis_data[i].home_way){
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
    double home_crds = rt_data->axis_data[axis_type].home_crds;
    double current_pos = rt_data->axis_data[axis_type].mac_cmd_pos;
    if (fabs(home_crds - current_pos) < rt_data->axis_data[axis_type].ref_range) return true;
    return false;
}

static void reset_pos_at_home(AXIS_TYPE axis_type){
    // 到达零点，重置cmd_pos
    rt_data->axis_data[axis_type].wcs_cmd_pos_cur = rt_data->axis_data[axis_type].home_crds;
    rt_data->axis_data[axis_type].wcs_cmd_pos_pre = rt_data->axis_data[axis_type].home_crds;
    rt_data->axis_data[axis_type].mac_cmd_pos = rt_data->axis_data[axis_type].home_crds;
}

GNC_RES homing_tp_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    homing_tp_delete();
    homing_tp = (HOMING_TP*)calloc(SYS_AXES_NUM * sizeof(HOMING_TP), 1);
    if(homing_tp == NULL) return GNC_DATA_NULL;
    for(int i = 0; i < SYS_AXES_NUM; i++){
        homing_tp[i].homing_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
        homing_tp[i].homing_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
        homing_tp[i].traj_data = (TRAJ_PARAMS*)calloc(sizeof(TRAJ_PARAMS), 1);
        if(homing_tp[i].homing_block == NULL || homing_tp[i].homing_profile == NULL || homing_tp[i].traj_data == NULL){
            homing_tp_delete();
            return GNC_DATA_NULL;
        }
        homing_tp[i].homing_block->type = LINE;
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
        homing_tp[i].homing_block->max_vel = rt_data->axis_data[i].jog_lowspd;
        homing_tp[i].homing_block->feed_vel = rt_data->axis_data[i].jog_lowspd;
        homing_tp[i].homing_block->max_acc = rt_data->axis_data[i].max_feed_acc;
        homing_tp[i].homing_block->max_jerk = rt_data->axis_data[i].max_feed_jk;
        homing_tp[i].homing_block->end_vel = 0;
        homing_tp[i].homing_direction = AXIS_HOMING_POSITIVE;
    }
    return res;
}

void homing_tp_delete(){
    if(homing_tp == NULL) return;
    for(int i = 0; i < SYS_AXES_NUM; i++){
        free(homing_tp[i].homing_block);
        homing_tp[i].homing_block = NULL;
        free(homing_tp[i].homing_profile);
        homing_tp[i].homing_profile = NULL;
        free(homing_tp[i].traj_data);
        homing_tp[i].traj_data = NULL;
    }
    free(homing_tp);
    homing_tp = NULL;
}

void homing_tp_reset(){
    if(homing_tp == NULL) return;
    for(int i = 0; i < SYS_AXES_NUM; i++){
        if(homing_tp[i].traj_data == NULL) continue;
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
        homing_tp[i].is_ref_on = false;
        memset(homing_tp[i].traj_data, 0, sizeof(TRAJ_PARAMS));
    }
}

GNC_RES abs_homing_tp_update_params(AXIS_TYPE axis_type, double current_vel, double target_vel){
    GNC_RES res = VP_CAL_SUCCESS;
    homing_tp[axis_type].homing_block->feed_vel = MIN(rt_data->axis_data[axis_type].max_rapid_vel, rt_data->axis_data[axis_type].jog_lowspd);
    homing_tp[axis_type].homing_block->max_vel = homing_tp[axis_type].homing_block->feed_vel * var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / OVERRIDE_PERCENT;
    homing_tp[axis_type].homing_block->max_acc = rt_data->axis_data[axis_type].max_feed_acc;
    homing_tp[axis_type].homing_block->max_jerk = rt_data->axis_data[axis_type].max_feed_jk;
    // 如果新速度不满足，不更新新速度
    if(!homing_tp_new_vel_valid(axis_type, target_vel)) return res;
    homing_tp[axis_type].homing_block->length = fabs(rt_data->axis_data[axis_type].home_crds - rt_data->axis_data[axis_type].mac_cmd_pos); // 设置的x轴零点与当前位置之间的距离
    // LOG(LOG_LEVEL_DEBUG, "wcs_cmd_pos_cur %f, mac_cmd_pos %f home_pos:%f length %f", rt_data->axis_data[axis_type].wcs_cmd_pos_cur, rt_data->axis_data[axis_type].mac_cmd_pos, par_axis[axis_type][PAR_AX_HOME_CRDS].value.val_real, homing_tp[axis_type].homing_block->length);
    homing_tp[axis_type].homing_block->start_vel = fabs(rt_data->chan_data[0].cmd_vel);  // 当前速度
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
    double vs = fabs(rt_data->chan_data[0].cmd_vel); double ve = 0;
    double remain_s = fabs(rt_data->axis_data[axis_type].home_crds - rt_data->axis_data[axis_type].mac_cmd_pos);
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
    double ncu_cycle = rt_data->ncu_data.cycle;
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if(rt_data->axis_data[axis_type].home_crds > rt_data->axis_data[axis_type].mac_cmd_pos) homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
    else homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, ncu_cycle); 
    // if(res != VP_CAL_SUCCESS) LOG(LOG_LEVEL_ERR, "vp cal vel profile fail");
    return res;
}

static GNC_RES abs_declining_tp_init(AXIS_TYPE axis_type, double target_vel){
    GNC_RES res = VP_CAL_SUCCESS;
    double vs = homing_tp[axis_type].homing_block->start_vel; double ve = 0;
    double amax = homing_tp[axis_type].homing_block->max_acc;
    double jmax = homing_tp[axis_type].homing_block->max_jerk;
    double ncu_cycle = rt_data->ncu_data.cycle;
    homing_tp[axis_type].homing_block->end_vel = target_vel;
    homing_tp[axis_type].homing_block->length = vp_cal_decel_dist(T_SHAPE, vs, ve, amax, jmax); // 减速距离
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if(rt_data->axis_data[axis_type].home_crds > rt_data->axis_data[axis_type].mac_cmd_pos) homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE;  // 回零移动方向
    else homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    // todo：如果减速距离极小
    if(IsEqualTo(homing_tp[axis_type].homing_block->length, 0)){
        homing_tp[axis_type].homing_profile->t_cycle = 0;
        homing_tp[axis_type].traj_data->v = target_vel;
        return VP_LENGTH_IS_ZERO;
    }
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, ncu_cycle); 
    // if(res != VP_CAL_SUCCESS) LOG(LOG_LEVEL_ERR, "vp cal vel profile fail");
    return res;
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