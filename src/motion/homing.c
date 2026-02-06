#include "homing.h"
#include "datatype.h"
#include "global.h"
#include "log.h"
#include "logapi.h"
#include <alarmapi.h>

static void homing_set_state();
static void homing_is_all_homed();
static void abs_homing_do_home(AXIS_TYPE axis_type);
static bool abs_homing_is_axis_homed(AXIS_TYPE axis_type);
static bool abs_homing_is_axis_canceled(AXIS_TYPE axis_type);
static GNC_RES abs_homing_tp_init(AXIS_TYPE axis_type);
static GNC_RES abs_cancel_tp_init(AXIS_TYPE axis_type);
static double abs_homing_tp_pinc(AXIS_TYPE axis_type);
static void abs_homing_tp_update(VELPLAN_TYPE velplan_type, double cycle_time, AXIS_TYPE axis_type);
static bool homing_check_res(GNC_RES res);

void handle_homing_cmd() {
    homing_set_state();
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        switch (par_axis[i].home_way) {
            case ABSOLUTE_AXIS_HOMING:
                abs_homing_do_home(i);
                break;
            case RELATIVE_AXIS_HOMING:
                // 预留：相对式回零
                // relative_homing_do_home();
                break;
        }
        // 更新速度参数
        var_axis[i].cmd_vel = free_traj_data[i].v;
    }
    homing_is_all_homed();
}

static void abs_homing_do_home(AXIS_TYPE axis_type) {
    var_axis[axis_type].cmd_pos = var_axis[axis_type].cmd_pos + abs_homing_tp_pinc(axis_type);
    // printf("axis_type:%d, act_pos:%f, cmd_pos:%f\n", axis_type, var_axis[axis_type].act_pos, var_axis[axis_type].cmd_pos);
}

static void homing_is_all_homed() {
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        switch (par_axis[i].home_way) {
            case ABSOLUTE_AXIS_HOMING:
                if (homing_tp[i].homing_state != ABS_EXEC_IDLE) return;
                break;
            case RELATIVE_AXIS_HOMING:
                break;
        }
    }
    var_chan[0].is_homing = false;
    var_chan[0].is_moving = false;
}

static void homing_set_state() {
    GNC_RES res = GNC_CAL_SUCCESS;
    switch (sys->nc_event) {
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // log_system("[home] 回零处理复位");
            LOG_OUTPUT(INFO, "回零处理复位");
            for (int i = 0; i < SYS_AXES_NUM; i++) {
                if (homing_tp[i].homing_state == ABS_EXEC_HOMING) {
                    if (abs_homing_is_axis_homed(i)) {
                        homing_tp[i].homing_state = ABS_EXEC_IDLE;
                        break;
                    }
                    // 创建减速tp
                    res = abs_cancel_tp_init(i);
                    CHECK_RES(homing_check_res(res));
                    homing_tp[i].homing_state = ABS_EXEC_CANCELING;
                    var_chan[0].is_resetting = true;
                }
            }
            break;

        case SYS_EVENT_X_POS_PRESS:
            // log_system("[home] 回零处理X按下");
            LOG_OUTPUT(INFO, "回零处理X按下");
            // todo：复位暂停的过程是否可以再次按下X重新进行回零
            if (sys->is_switching_mode) break; // 换挡模式下，不允许操作
            if (homing_tp[AXIS_X].homing_state == ABS_EXEC_IDLE) {
                if (abs_homing_is_axis_homed(AXIS_X)) {
                    homing_tp[AXIS_X].homing_state = ABS_EXEC_IDLE;
                    break;
                }
                // 创建回零TP
                res = abs_homing_tp_init(AXIS_X);
                CHECK_RES(homing_check_res(res));
                homing_tp[AXIS_X].homing_state = ABS_EXEC_HOMING;
                var_chan[0].is_moving = true;
                var_chan[0].is_homing = true;
            }
            break;

        case SYS_EVENT_Y_POS_PRESS:
            // log_system("[home] 回零处理Y按下");
            LOG_OUTPUT(INFO, "回零处理Y按下");
            if (sys->is_switching_mode) break;
            if (homing_tp[AXIS_Y].homing_state == ABS_EXEC_IDLE) {
                if (abs_homing_is_axis_homed(AXIS_Y)) {
                    homing_tp[AXIS_Y].homing_state = ABS_EXEC_IDLE;
                    break;
                }
                res = abs_homing_tp_init(AXIS_Y);
                CHECK_RES(homing_check_res(res));
                homing_tp[AXIS_Y].homing_state = ABS_EXEC_HOMING;
                var_chan[0].is_moving = true;
                var_chan[0].is_homing = true;
            }
            break;

        case SYS_EVENT_Z_POS_PRESS:
            // log_system("[home] 回零处理Z按下");
            LOG_OUTPUT(INFO, "回零处理Z按下");
            if (sys->is_switching_mode) break;
            if (homing_tp[AXIS_Z].homing_state == ABS_EXEC_IDLE) {
                if (abs_homing_is_axis_homed(AXIS_Z)) {
                    homing_tp[AXIS_Z].homing_state = ABS_EXEC_IDLE;
                    break;
                }
                res = abs_homing_tp_init(AXIS_Z);
                CHECK_RES(homing_check_res(res));
                homing_tp[AXIS_Z].homing_state = ABS_EXEC_HOMING;
                var_chan[0].is_moving = true;
                var_chan[0].is_homing = true;
            }
            break;

        case SYS_EVENT_RERUN:
        case SYS_EVENT_PAUSE:
        default:
            // log_system("[home] 回零不支持次操作(%d)", sys->nc_event);
            LOG_OUTPUT(WARN, "回零不支持次操作 %d", sys->nc_event);
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

static bool homing_check_res(GNC_RES res) {
    switch (res) {
        case GNC_CAL_SUCCESS:
        case GNC_OPERATOR_SUCCESS:
            return true;
        case GNC_DATA_NULL:
        case GNC_PARAM_UNVALID:
        case GNC_CAL_ERROR:
        case GNC_OPERATOR_ERROR:
        default:
            homing_tp_reset(homing_tp);
            var_chan[0].is_estop = true;
            var_chan[0].is_resetting = false;
            var_chan[0].is_moving = false;
            var_chan[0].is_homing = false;
            // log_system("[home] 回零错误(%d)", res);
            LOG_OUTPUT(ERROR, "回零错误 %d", res);
            setAlarm(ALARM_CH, ALARM_ERROR, 1);
            return false;
    }
}

static double abs_homing_tp_pinc(AXIS_TYPE axis_type) {
    double s = 0;
    switch (homing_tp[axis_type].homing_state) {
        case ABS_EXEC_IDLE:
            memset(&free_traj_data[axis_type], 0, sizeof(TRAJ_PARAMS));
            break;

        case ABS_EXEC_HOMING:
            // 检查是否完成回零
            if (abs_homing_is_axis_homed(axis_type)) {
                homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                var_axis[axis_type].is_homef = true;
                break;
            }
            // 进行回零
            abs_homing_tp_update(T_SHAPE, par_ncu.cycle, axis_type);
            s = homing_tp[axis_type].target_distance;
            break;

        case ABS_EXEC_CANCELING:
            // 检查是否完成减速
            if (abs_homing_is_axis_canceled(axis_type)) {
                homing_tp[axis_type].homing_state = ABS_EXEC_IDLE;
                var_chan[0].is_resetting = false;
                break;
            }
            // 进行回零
            abs_homing_tp_update(T_SHAPE, par_ncu.cycle, axis_type);
            s = homing_tp[axis_type].target_distance;
            break;

       default:
           // error
           break;
    };
    if (homing_tp[axis_type].homing_direction == AXIS_HOMING_NEGATIVE) s = -s;
    return s;
};

static GNC_RES abs_homing_tp_init(AXIS_TYPE axis_type) {
    GNC_RES res = GNC_CAL_SUCCESS;
    homing_tp[axis_type].homing_block->length = fabs(par_axis[axis_type].home_crds - var_axis[axis_type].act_pos);            // 设置的x轴零点与当前位置之间的距离
    homing_tp[axis_type].homing_block->start_vel = fabs(var_axis[axis_type].act_vel);                                         // 当前速度
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, par_ncu.cycle); // 速度控制方式：T型、S型、SIN型
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if (par_axis[axis_type].home_crds > var_axis[axis_type].act_pos)
        homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE; // 回零移动方向
    else
        homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    var_chan[0].is_homing = true;
    if (res != GNC_CAL_SUCCESS){
        // log_system("[home] 速度计算错误(%d)", res);
        LOG_OUTPUT(ERROR, "速度计算错误 %d", res);
    }
    return res;
}

static GNC_RES abs_cancel_tp_init(AXIS_TYPE axis_type) {
    GNC_RES res = GNC_CAL_SUCCESS;
    homing_tp[axis_type].homing_block->start_vel = fabs(var_axis[axis_type].act_vel); // 当前速度
    // 注意：速度、加速度、加加速度和距离均为非负数，请校验
    homing_tp[axis_type].homing_block->length
        = vp_cal_decel_dist(T_SHAPE, fabs(var_axis[axis_type].act_vel), 0, homing_tp[axis_type].homing_block->max_acc, homing_tp[axis_type].homing_block->max_jerk); // 减速距离
    // 注意：velplan_type 只有T型、S型、SIN型 三种类型
    res = vp_cal_vel_profile(T_SHAPE, homing_tp[axis_type].homing_block, homing_tp[axis_type].homing_profile, par_ncu.cycle); // 速度控制方式：T型、S型、SIN型
    homing_tp[axis_type].interp_cycle = 1;
    homing_tp[axis_type].accum_distance = 0;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length; // 单个轴的零点距离
    if (par_axis[axis_type].home_crds > var_axis[axis_type].act_pos)
        homing_tp[axis_type].homing_direction = AXIS_HOMING_POSITIVE; // 回零移动方向
    else
        homing_tp[axis_type].homing_direction = AXIS_HOMING_NEGATIVE;
    if (res != GNC_CAL_SUCCESS){
        // log_system("[home] 速度计算错误(%d)", res);
        LOG_OUTPUT(ERROR, "速度计算错误 %d", res);
    }
    return res;
}

static void abs_homing_tp_update(VELPLAN_TYPE velplan_type, double cycle_time, AXIS_TYPE axis_type) {
    vp_cal_traj_data(velplan_type, homing_tp[axis_type].homing_profile, homing_tp[axis_type].interp_cycle, &free_traj_data[axis_type], cycle_time);
    homing_tp[axis_type].current_distance = homing_tp[axis_type].target_distance;
    homing_tp[axis_type].target_distance = free_traj_data[axis_type].s - homing_tp[axis_type].accum_distance;
    homing_tp[axis_type].accum_distance = free_traj_data[axis_type].s;
    homing_tp[axis_type].remain_distance = homing_tp[axis_type].homing_block->length - homing_tp[axis_type].accum_distance;
    homing_tp[axis_type].interp_cycle += 1;
}

// 检查轴是否都已回零
static bool abs_homing_is_axis_homed(AXIS_TYPE axis_type) {
    if (fabs(par_axis[axis_type].home_crds - var_axis[axis_type].act_pos) < HOME_TOL) return true;
    return false;
}

// 检查轴是否都已取消回零
static bool abs_homing_is_axis_canceled(AXIS_TYPE axis_type) {
    if (homing_tp[axis_type].interp_cycle > homing_tp[axis_type].homing_profile->t_cycle) return true;
    return false;
}

GNC_RES homing_tp_init(HOMING_TP* homing_tp) {
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        homing_tp[i].homing_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
        if(homing_tp[i].homing_block == NULL) return GNC_DATA_NULL;
        homing_tp[i].homing_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
        if(homing_tp[i].homing_profile == NULL) return GNC_DATA_NULL;
        homing_tp[i].homing_block->type = LINE;
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
        homing_tp[i].homing_block->max_vel = par_axis[i].rapid_vel * var_chan[0].rapid_override;
        homing_tp[i].homing_block->feed_vel = par_axis[i].rapid_vel;
        homing_tp[i].homing_block->max_acc = par_axis[i].rapid_acc;
        homing_tp[i].homing_block->max_jerk = par_axis[i].rapid_jerk;
        homing_tp[i].homing_block->end_vel = 0;
        homing_tp[i].homing_direction = AXIS_HOMING_POSITIVE;
    }
    return GNC_OPERATOR_SUCCESS;
}

void homing_tp_reset(HOMING_TP* homing_tp) {
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        homing_tp[i].homing_state = ABS_EXEC_IDLE;
    }
    var_chan[0].is_homing = false;
}
