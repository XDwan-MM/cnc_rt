#include "emcmot.h"
#include "global.h"
#include <hmisysapi.h>
#include "egear_ratio.h"
#include "mcode_queue.h"
#include "ferror_control.h"
#include "pathplan.h"
#include "rt_sem.h"
#include "alarm.h"
#include "jog.h"
#include "handwheel.h"
#include "homing.h"
#include "mem.h"
#include "spl_control.h"
#include "stored_stroke_check.h"
#include "parser.h"
#include "hmisysapi.h"
#include "master.h"
#include <alarmapi.h>
#include <math.h>
#include "rt_data.h"
#include "pitch_comp.h"
#include "comp_data.h"
#include "debug_mode.h"
#include "master.h"
#include <hmisysapi.h>

static void check_for_fault();
static void set_operating_mode();
static void spindle_control();
static void get_cmd_pos();
static void update_status();
static void output_to_hal();
static void read_from_hal();
static void check_for_soft_limit();
static void compute_error_comp();
static void calcul_mac_cmd_pos();

static int32_t x_cmd_Pulses, y_cmd_Pulses, z_cmd_Pulses;
static int32_t x_target_pos, y_target_pos, z_target_pos;
char* mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "回零", "空闲"};

// 轨迹规划
void emcmotTrajPlan() {
    // todo：什么时候op_flag=0，为0应统一关闭线程，报错退出系统
    if (op_flag == 0) {
        return;
    }

    // 遍历每个通道
    for (int i = 0; i < var_sys[VAR_SYS_CHAN_NUM].value.i; i++) {
        
        // 当前活动通道
        setVarSysInt(VAR_SYS_ACTIVE_CHAN, i, 0);
        
        if(rt_data->nc_event == SYS_EVENT_RESET)
        {
            // 复位状态：2.外部复位信号ERS 4.按下MDI上的RESET键
            nc_set_resetting();
            setVarChanInt(0, VAR_CHAN_IS_RESETTING, 1, 0);
            // 复位生效参数
            Axis_Limit_Manager_Init();
            // 清除跟随误差标志
            reset_ferror_control();
            // 清除警告
            cleanAllAlarm(ALARM_TYPE_ALL, ALARM_WARNING);
        }

        // 检查系统是否有异常
        check_for_fault();

        // 读取抽象层数据
        read_from_hal();

        // 检查跟随误差
        check_ferror_fault();

        // 判断运动模式
        set_operating_mode();

        // 主轴控制
        spindle_control();

        // 根据运动模式计算运动位置
        get_cmd_pos();

        // 计算mac_cmd_pos
        calcul_mac_cmd_pos();

        // 误差补偿
        compute_error_comp();

        //更新轴状态
        Axis_Status_Update();

        // 预先检查插补位置是否超软限位
        check_for_soft_limit();

        // 更新抽象层数据
        output_to_hal();

        // 上传运动状态数据
        update_status();

        // 清除事件
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}

// 路径规划
void emcmotPathPlan() {
    // 遍历多个通道
    for (int i = 0; i < var_sys[VAR_SYS_CHAN_NUM].value.i; i++) {
        if (rt_get_chan_state(i, CHAN_IS_PARSER_RERUN)){
            // LOG(LOG_LEVEL_INFO, "初始化译码器");
            parser_init(rt_data->current_mode);
            rt_set_chan_state(i, CHAN_IS_FIRST_RUN, true);      // 通知预处理第一次获取pre_pos
            rt_set_chan_state(i, CHAN_IS_PARSER_RERUN, false);
        }
        // 解释器译码
        parser_run_mode();
        // 预处理，添加block
        chan_add_block();
    }
}

static void set_operating_mode() {
    // 急停拍下时不允许切换模式
    if (rt_get_sys_state(SYS_IS_ESTOP_PRESS)) return;
    // 如果没有切换，不进行操作
    if (rt_data->current_mode == rt_data->target_mode) {
        rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
        return;
    }
    rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
    // 根据切换的加工模式和当前加工模式，判断是否可以切换
    switch (rt_data->current_mode) {
        case MODE_IDLE:
            rt_data->current_mode = rt_data->target_mode;
            break;
        case MODE_JOG:
            // 当前点动速度为零,才能切模式成功
            if (fabs(jog_tp[AXIS_X].curr_vel) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Y].curr_vel) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Z].curr_vel) < SPEED_THRESHOLD)
            {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            }
            break;
        case MODE_EDIT:
        case MODE_MPG:
            rt_data->current_mode = rt_data->target_mode;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            break;
        case MODE_HOMING:
            if (!var_chan[0][VAR_CHAN_IS_HOMING].value.i) {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            }
            break;
        case MODE_MDI:
        case MODE_MEM:
        case MODE_DNC:
            if(!var_chan[0][VAR_CHAN_IS_RUNNING].value.i){
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            }
            break;
        default:
            break;
    }
}

// 主轴控制
static void spindle_control(){
    spindle_set_state();
    if(spindle.mode != DRV_MODE_DISABLED)
    {
        spindle_cal_speed();
    }
}

// 不同加工模式的运动控制
static void get_cmd_pos() {
    switch (rt_data->current_mode) {
        case MODE_EDIT: // 编辑
            handle_edit_cmd();
            break;
        case MODE_MDI: // MDI
        case MODE_MEM: // 记忆
        case MODE_DNC:
            handle_auto_cmd();
            break;
        case MODE_HOMING: // 回零
            handle_homing_cmd();
            break;
        case MODE_JOG: // 点动
            handle_jog_cmd();
            break;
        case MODE_MPG: // 手轮
            handle_mpg_cmd();
            break;
        case MODE_IDLE: // 空闲状态
        default: // error
            break;
    }
}

static void check_for_fault() {
    if(rt_get_sys_state(SYS_IS_ESTOP_PRESS)){
        handle_estop_cmd();
    }
    //　检查系统报警和内部急停标志
    int alarm_count = 0;
    getAlarmCount(ALARM_TYPE_ALL, ALARM_ERROR, &alarm_count);
    if(alarm_count > 0 || rt_data->sys_state[SYS_IS_ESTOP_ON]){
        rt_data->sys_state[SYS_IS_ALARMING] = true;
        // 除急停外的其他报警，取消准备就绪信号
        if(!rt_data->sys_state[SYS_IS_ESTOP_ON]) rt_set_sys_state(SYS_IS_READY, false);
        handle_alarm_cmd();
    }
    if(alarm_count <= 0){
        rt_data->sys_state[SYS_IS_ALARMING] = false;
        rt_set_sys_state(SYS_IS_READY, true);
    }
}

static void read_from_hal() {
    // 读取实际位置
    #ifdef DEBUG_MODE_ENABLED
    rt_data->axis_data[AXIS_X].motor_act_pos =  rt_data->axis_data[AXIS_X].motor_cmd_pos;
    rt_data->axis_data[AXIS_Y].motor_act_pos =  rt_data->axis_data[AXIS_Y].motor_cmd_pos;
    rt_data->axis_data[AXIS_Z].motor_act_pos =  rt_data->axis_data[AXIS_Z].motor_cmd_pos;
    #else
    Hal_Driver_Get_Actual_Pos(&x_axis_driver, &x_target_pos);
    rt_data->axis_data[AXIS_X].motor_act_pos = pulses_to_position(x_target_pos, par_axis[AXIS_X]);
    Hal_Driver_Get_Actual_Pos(&y_axis_driver, &y_target_pos);
    rt_data->axis_data[AXIS_Y].motor_act_pos = pulses_to_position(y_target_pos, par_axis[AXIS_Y]);
    Hal_Driver_Get_Actual_Pos(&z_axis_driver, &z_target_pos);
    rt_data->axis_data[AXIS_Z].motor_act_pos = pulses_to_position(z_target_pos, par_axis[AXIS_Z]);
    #endif

    // 第一次运行，初始化位置
    if (!update_pos_flag) {
        for (int i = 0; i < SYS_AXES_NUM; i++)
        {
            rt_data->axis_data[i].wcs_cmd_pos_cur = rt_data->axis_data[i].motor_act_pos;
            rt_data->axis_data[i].wcs_cmd_pos_pre = rt_data->axis_data[i].motor_act_pos;
            rt_data->axis_data[i].mac_cmd_pos = rt_data->axis_data[i].motor_act_pos;
            rt_data->axis_data[i].motor_cmd_pos = rt_data->axis_data[i].motor_act_pos;
            rt_data->axis_data[i].fb_pos_no_comp = rt_data->axis_data[i].motor_act_pos;
        }
        update_pos_flag = 1;
    }
}

static void output_to_hal() {
    // 发送给驱动器
    x_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_X].motor_cmd_pos, par_axis[AXIS_X]);
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_cmd_Pulses);
    y_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_Y].motor_cmd_pos, par_axis[AXIS_Y]);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_cmd_Pulses);
    z_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_Z].motor_cmd_pos, par_axis[AXIS_Z]);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_cmd_Pulses);
}

static void compute_error_comp() 
{
    double pitch_comp_val[3];
    double backlash_comp_val[3];
    double perp_comp_val[3];

    // Sync params from shared memory to library-compatible structs
    // 暂时是立马同步补偿参数，后续改成复位生效应将函数移到复位逻辑．
    sync_comp_params();

    // TODO: check_and_rebuild_pitch_tables() 脏标记检测（预留）
    // 后续 HMI 侧 par_table[] 接口就绪后，加 PAR_CMP_PITCH_TABLE_DIRTY 枚举并实现

    bool is_rapid = false;
    if(cycle_tp->plan_block->type == RAPID || rt_data->sys_state[SYS_IS_RAPID])
    {
        is_rapid = true;
    }

    // 构建 all_axis_pos[] 供垂直度等跨轴补偿使用
    double all_axis_pos[3];
    for (int i = 0; i < 3; i++) {
        all_axis_pos[i] = rt_data->axis_data[i].mac_cmd_pos;
    }

    for (int i = 0; i < 3; i++) {
        double vel_cmd = var_axis[i][VAR_AXIS_CMD_VEL].value.f;
        double mac_cmd_pos = rt_data->axis_data[i].mac_cmd_pos;

        // 螺距误差补偿值计算 (计算出来的补偿值 单位为:mm)
        pitch_comp_val[i] = calculate_pitch_compensation(
            &pitch_comp_state[i], &par_comp_lib[i], &pitch_comp_table[i],
            mac_cmd_pos, vel_cmd);

        // 反向间隙补偿值计算 (计算出来的补偿值 单位为:mm)
        backlash_comp_val[i] = calculate_backlash_compensation(
            &par_comp_lib[i], &backlash_comp_state[i],
            vel_cmd, is_rapid);

        // 垂直度误差补偿值计算 (计算出来的补偿值 单位为:mm)
        perp_comp_val[i] = calculate_perpendicularity_total(
            &par_comp_lib[i], &perp_comp_state[i],
            i, all_axis_pos, vel_cmd);

        // TODO: 直线度补偿值计算 (预留)
        // straight_comp_val = calculate_straightness_compensation(...);

        double total_comp = pitch_comp_val[i] + backlash_comp_val[i] + perp_comp_val[i];

        // 当前周期插补+补偿后位置
        rt_data->axis_data[i].motor_cmd_pos = mac_cmd_pos + total_comp;
        // 反馈位置（去除补偿）
        rt_data->axis_data[i].fb_pos_no_comp = rt_data->axis_data[i].motor_act_pos - total_comp;

        rt_data->axis_data[i].pitch_comp_val    = pitch_comp_val[i];
        rt_data->axis_data[i].backlash_comp_val = backlash_comp_val[i];
        rt_data->axis_data[i].perp_comp_val     = perp_comp_val[i];

        
    }
    // printf(
    //     "Pos_nc%f, Pos_c%f, vel_cmd%f, luobu%f, bl%f\n",
    //     rt_data->axis_data[0].mac_cmd_pos,
    //     rt_data->axis_data[0].motor_cmd_pos,
    //     var_axis[0][VAR_AXIS_CMD_VEL].value.f,
    //     rt_data->axis_data[0].pitch_comp_val,
    //     rt_data->axis_data[0].backlash_comp_val
    // );
}

static void calcul_mac_cmd_pos()
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        // 机床锁住或者轴拆卸，不下发位置
        if (!rt_data->sys_state[SYS_IS_MAC_LOCK] && !rt_data->axis_data[i].axis_status.is_disable)
        {
            rt_data->axis_data[i].mac_cmd_pos += rt_data->axis_data[i].wcs_cmd_pos_cur - rt_data->axis_data[i].wcs_cmd_pos_pre;
        }
        // 上一周期插补位置
        rt_data->axis_data[i].wcs_cmd_pos_pre = rt_data->axis_data[i].wcs_cmd_pos_cur;
    }
}

static void check_for_soft_limit()
{
    for (int var = 0; var < 3; ++var)
    {
        if (runtime[var].is_first_run)
        {
            int levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, RESET_CHEAK_USED);
            runtime[var].temp_config = runtime[var].config;
            runtime[var].limit_pos = rt_data->axis_data[var].mac_cmd_pos;
            if (!levels)
            {
                runtime[var].limit_type = NO_LIMIT_TRIGGERED;
            }
            runtime[var].is_first_run = 0;
        }

        if (rt_data->axis_data[var].axis_status.is_moving)
        {
            if ((runtime[var].limit_type == 0) ||
                (!rt_data->axis_data[var].axis_status.is_decel_done))
            {
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, SOFT_CHEAK_USED);
                runtime[var].limit_pos = rt_data->axis_data[var].mac_cmd_pos;
            }
            switch (runtime[var].limit_type)
            {
            case LEVEL1_FWD_LIMIT:
                runtime[var].temp_config.level1.positive_limit = runtime[var].limit_pos;
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            case LEVEL1_REV_LIMIT:
                runtime[var].temp_config.level1.negative_limit = runtime[var].limit_pos;
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            case LEVEL2_FWD_LIMIT:
                if (runtime[var].config.level2_mode == 0)
                {
                    runtime[var].temp_config.level2.positive_limit = runtime[var].limit_pos;
                }
                else
                {
                    runtime[var].temp_config.level2.negative_limit = runtime[var].limit_pos;
                }
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            case LEVEL2_REV_LIMIT:
                if (runtime[var].config.level2_mode == 0)
                {
                    runtime[var].temp_config.level2.negative_limit = runtime[var].limit_pos;
                }
                else
                {
                    runtime[var].temp_config.level2.positive_limit = runtime[var].limit_pos;
                }
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            case LEVEL3_FWD_LIMIT:
                runtime[var].temp_config.level3.negative_limit = runtime[var].limit_pos;
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            case LEVEL3_REV_LIMIT:
                runtime[var].temp_config.level3.positive_limit = runtime[var].limit_pos;
                runtime[var].active_levels = Check_Axis_Limit_Violation(&runtime[var], rt_data->axis_data[var].mac_cmd_pos, ALARM_CHEAK_USED);
                break;
            default:
                break;
            }
        }
    }
}

// 更新运动状态数据
static void update_status() {
    // 更新var变量
    shm_var_update();
    // 更新复位状态
    nc_set_resetted();
}

