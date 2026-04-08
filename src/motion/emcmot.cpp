#include "emcmot.h"
#include "global.h"
#include <hmisysapi.h>
#include "egear_ratio.h"
#include "mcode_queue.h"
#include "ferror_control.h"
#include "pathplan.h"
#include "rt_sem.h"
#include "spindle.h"
#include "alarm.h"
#include "rapid.h"
#include "jog.h"
#include "handwheel.h"
#include "homing.h"
#include "mem.h"
#include "spindle_test.h"
#include "stored_stroke_check.h"
#include "parser.h"
#include "hmisysapi.h"
#include "master.h"
#include <alarmapi.h>
#include <math.h>
#include "rt_data.h"

static void check_for_fault();
static void set_operating_mode();
static void spindle_control();
static void get_cmd_pos();
static void update_status();
static void check_ferror_fault();
static void output_to_hal();
static void read_from_hal();
static void check_for_soft_limit();
static void compute_screw_comp();

static int32_t x_cmd_Pulses, y_cmd_Pulses, z_cmd_Pulses;
static int32_t x_target_pos, y_target_pos, z_target_pos;
char* mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "快速点动", "回零", "报警", "空闲"};

// 轨迹规划
void emcmotTrajPlan() {
    // todo：什么时候op_flag=0，为0应统一关闭线程，报错退出系统
    if (op_flag == 0) {
        return;
    }

    // 遍历每个通道
    for (int i = 0; i < var_sys[VAR_SYS_CHAN_NUM].value.i; i++) {
        
        if(rt_data->nc_event == SYS_EVENT_RESET)
        {
            #ifndef DEBUG_MODE_ENABLED
            Master_Reset();
            #endif
            //复位生效参数
            Axis_Limit_Manager_Init();
            cleanAllAlarm(ALARM_TYPE_ALL, ALARM_WARNING);
        }

        // 当前活动通道
        setVarSysInt(VAR_SYS_ACTIVE_CHAN, i, 0);

        // 检查系统是否有异常
        check_for_fault();

        // 读取抽象层数据
        read_from_hal();

        // 判断运动模式
        set_operating_mode();

        // 临时：主轴控制
        spindle_control();

        // 根据运动模式计算运动位置
        get_cmd_pos();

        // 误差补偿
        compute_screw_comp();

        // 预先检查插补位置是否超软限位
        check_for_soft_limit();

        // 检查跟随误差
        check_ferror_fault();

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
            HmiSysData hmiSysData;
            #ifdef DEBUG_MODE_ENABLED
            char ncFilePath[512];
            snprintf(ncFilePath, sizeof(ncFilePath), "%s/%s", PROJECT_DIR, "config/NC/c.nc");
            parser_init(ncFilePath);
            #else
            getHmiSysDataPtr(&hmiSysData);
            parser_init(hmiSysData.ncFilePath);
            #endif
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
    // 如果没有切换，不进行操作
    if (rt_data->current_mode == rt_data->target_mode) {
        rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
        return;
    }
    // 根据切换的加工模式和当前加工模式，判断是否可以切换
    switch (rt_data->current_mode) {
        case MODE_IDLE:
            rt_data->current_mode = rt_data->target_mode;
            break;
        case MODE_JOG:
            // 当前点动速度为零,才能切模式成功
            if (fabs(jog_tp[AXIS_X].curr_vel - 0.0) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Y].curr_vel - 0.0) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Z].curr_vel - 0.0) < SPEED_THRESHOLD)
            {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[rt_data->current_mode]);
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
                break;
            }
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
            break;
        case MODE_EDIT:
        case MODE_MPG:
            rt_data->current_mode = rt_data->target_mode;
            // log_system("[emcmot] 系统模式切换为(%s)", mode_names[rt_data->current_mode]);
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            break;
        case MODE_RAPID_JOG:
            // 当前快速点动速度为零,才能切模式成功
            if (fabs(jog_tp[AXIS_X].curr_vel - 0.0) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Y].curr_vel - 0.0) < SPEED_THRESHOLD
            && fabs(jog_tp[AXIS_Z].curr_vel - 0.0) < SPEED_THRESHOLD)
            {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[rt_data->current_mode]);
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
                break;
            }
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
            rt_data->current_mode = rt_data->target_mode;
            break;
        case MODE_HOMING:
            if (!var_chan[0][VAR_CHAN_IS_HOMING].value.i) {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[rt_data->current_mode]);
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
                break;
            }else{
                // 回零中切换模式不能生效
                rt_data->target_mode = rt_data->current_mode;
                setAlarm(ALARM_SY, ALARM_WARNING, 0);
                // log_system("[emcmot] 回零中不能切换模式，请先按复位后切换");
                LOG(LOG_LEVEL_WARN, "回零中不能切换模式，请先按复位后切换");
                break;
            }
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
            break;

        case MODE_MDI:
        case MODE_MEM:
        case MODE_DNC:
            // 除了插补G指令，需判断其他G和M指令是否完成，才能切换模式
            if (!var_chan[0][VAR_CHAN_IS_RUNNING].value.i) {
                rt_data->current_mode = rt_data->target_mode;
                rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[rt_data->current_mode]);
                LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
                break;
            } else{
                // 加工下切换模式不能生效
                rt_data->target_mode = rt_data->current_mode;
                setAlarm(ALARM_SY, ALARM_WARNING, 1);
                // log_system("[emcmot] 自动加工下不能切换模式，请先按复位后切换");
                LOG(LOG_LEVEL_WARN, "自动加工下不能切换模式，请先按复位后切换");
                break;
            }
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
            break;
        default:
            break;
    }
}

// 主轴控制
static void spindle_control(){
    // if(rt_data->sys_state[SYS_IS_ESTOP_ON]) return;
//    switch (rt_data->current_mode){
//        case MODE_JOG:
//        case MODE_MPG:
//        case MODE_EDIT:
//            emcSpindleControl();
//            break;
//        case MODE_MEM:
//        case MODE_MDI:
//        case MODE_DNC:
//            emcmcodedeal();
//            break;
//        default:
//            break;
//    }
    spindle_set_state();
    if(spindle.mode == DRV_MODE_CSV)
    {
        spindle_cal_speed();
    }
}

// 不同加工模式的运动控制
static void get_cmd_pos() {
    switch (rt_data->current_mode) {
        case MODE_IDLE: // 空闲状态
            break;
        case MODE_EDIT: // 编辑
            break;
        case MODE_HOMING: // 回零
            handle_homing_cmd();
            break;
        case MODE_MDI: // MDI
        case MODE_MEM: // 自动
        case MODE_DNC:
            handle_run_cmd();
            break;
        case MODE_JOG: // 点动
            handle_jog_cmd();
            break;
        case MODE_RAPID_JOG: // 快速点动
            handle_jog_rapid_cmd();
            break;
        case MODE_MPG: // 手轮
            handle_mpg_cmd();
            break;
        default: // error
            break;
    }
}

static void check_for_fault() {
    if(rt_data->sys_state[SYS_IS_ESTOP_ON]){
        setAlarm(ALARM_SY, ALARM_ERROR, 0);
        // 重置各个模式
        mem_handle_estop();
        homing_handle_estop();
        jog_handle_estop();
        if(rt_data->nc_event != SYS_EVENT_NONE || rt_data->target_mode != rt_data->current_mode){
            //　不允许操作
            rt_data->nc_event = SYS_EVENT_NONE;
            // 不允许切换模式
            rt_data->target_mode = rt_data->current_mode;
            LOG(LOG_LEVEL_ERR, "急停报警下不允许操作，请检查问题！");
        }
    }
    //　检查系统报警
    int alarm_count = 0;
    getAlarmCount(ALARM_TYPE_ALL, ALARM_ERROR, &alarm_count);
    if(alarm_count > 0){
        rt_data->sys_state[SYS_IS_ALARMING] = true;
        handle_alarm_cmd();
    }
}

// 检查轴跟随误差错误
static void check_ferror_fault()
{
    // 急停拍下情况、(空闲、编辑状态)时不检查跟随误差，避免上电无运动误报
    if (rt_data->sys_state[SYS_IS_ESTOP_ON] || rt_data->current_mode == MODE_IDLE ||
        rt_data->current_mode == MODE_EDIT)
    {
        return;
    }
    /* 指令速度不为0时，检测跟随误差 */
    // x轴
    if (fabs(var_axis[AXIS_X][VAR_AXIS_CMD_VEL].value.f - 0.0) > SPEED_THRESHOLD)
    {
        // 指令速度 指令位置 反馈位置
        double vel_cmd_x = var_axis[AXIS_X][VAR_AXIS_CMD_VEL].value.f;
        double pos_cmd_x = rt_data->axis_data[AXIS_X].motor_cmd_pos;
        double pos_fb_x = rt_data->axis_data[AXIS_X].motor_act_pos;
        // 计算跟随误差
        updata_axis_ferror(&ferror_control[AXIS_X], par_axis[AXIS_X], pos_cmd_x, pos_fb_x, vel_cmd_x);
        // 跟随误差报警处理
        if (ferror_control[AXIS_X].error_flag)
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 100);
            if (res == -8)
                return;
            // 上述代码逻辑是让下面LOG只打印一次；返回值-8：表示该报警已存在。
            LOG(LOG_LEVEL_ERR, "X轴跟随误差过大");
        }
    }
    // y轴
    if (fabs(var_axis[AXIS_Y][VAR_AXIS_CMD_VEL].value.f - 0.0) > SPEED_THRESHOLD)
    {
        // 指令速度 指令位置 反馈位置
        double vel_cmd_y = var_axis[AXIS_Y][VAR_AXIS_CMD_VEL].value.f;
        double pos_cmd_y = rt_data->axis_data[AXIS_Y].motor_cmd_pos;
        double pos_fb_y = rt_data->axis_data[AXIS_Y].motor_act_pos;
        // 计算跟随误差
        updata_axis_ferror(&ferror_control[AXIS_Y], par_axis[AXIS_Y], pos_cmd_y, pos_fb_y, vel_cmd_y);
        // 跟随误差报警处理
        if (ferror_control[AXIS_Y].error_flag)
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 101);
            if (res == -8)
                return;
            LOG(LOG_LEVEL_ERR, "Y轴跟随误差过大");
        }
    }
    // z轴
    if (fabs(var_axis[AXIS_Z][VAR_AXIS_CMD_VEL].value.f - 0.0) > SPEED_THRESHOLD)
    {
        // 指令速度 指令位置 反馈位置
        double vel_cmd_z = var_axis[AXIS_Z][VAR_AXIS_CMD_VEL].value.f;
        double pos_cmd_z = rt_data->axis_data[AXIS_Z].motor_cmd_pos;
        double pos_fb_z = rt_data->axis_data[AXIS_Z].motor_act_pos;
        // 计算跟随误差
        updata_axis_ferror(&ferror_control[AXIS_Z], par_axis[AXIS_Z], pos_cmd_z, pos_fb_z, vel_cmd_z);
        // 跟随误差报警处理
        if (ferror_control[AXIS_Z].error_flag)
        {
            int res = setAlarm(ALARM_AX, ALARM_ERROR, 102);
            if (res == -8)
                return;
            LOG(LOG_LEVEL_ERR, "Z轴跟随误差过大");
        }
    }
}

static void read_from_hal() {
    // 读取实际位置
    Hal_Driver_Get_Actual_Pos(&x_axis_driver, &x_target_pos);
    rt_data->axis_data[AXIS_X].motor_act_pos = pulses_to_position(x_target_pos, par_axis[AXIS_X]);
    Hal_Driver_Get_Actual_Pos(&y_axis_driver, &y_target_pos);
    rt_data->axis_data[AXIS_Y].motor_act_pos = pulses_to_position(y_target_pos, par_axis[AXIS_Y]);
    Hal_Driver_Get_Actual_Pos(&z_axis_driver, &z_target_pos);
    rt_data->axis_data[AXIS_Z].motor_act_pos = pulses_to_position(z_target_pos, par_axis[AXIS_Z]);

    // 人机交互显示的位置值，要减去误差补偿值.
    var_axis[AXIS_X][VAR_AXIS_ACT_POS_EX].value.f = rt_data->axis_data[AXIS_X].motor_act_pos - 0;
    var_axis[AXIS_Y][VAR_AXIS_ACT_POS_EX].value.f = rt_data->axis_data[AXIS_Y].motor_act_pos - 0;
    var_axis[AXIS_Z][VAR_AXIS_ACT_POS_EX].value.f = rt_data->axis_data[AXIS_Z].motor_act_pos - 0;

    // 第一次运行，初始化位置
    if (!update_pos_flag) {
        var_axis[AXIS_X][VAR_AXIS_CMD_POS_EX].value.f = rt_data->axis_data[AXIS_X].motor_act_pos;
        var_axis[AXIS_Y][VAR_AXIS_CMD_POS_EX].value.f = rt_data->axis_data[AXIS_Y].motor_act_pos;
        var_axis[AXIS_Z][VAR_AXIS_CMD_POS_EX].value.f = rt_data->axis_data[AXIS_Z].motor_act_pos;
        update_pos_flag = 1;
    }
}

static void output_to_hal() {
    x_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_X].motor_cmd_pos, par_axis[AXIS_X]);
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_cmd_Pulses);
    y_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_Y].motor_cmd_pos, par_axis[AXIS_Y]);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_cmd_Pulses);
    z_cmd_Pulses = position_to_pulses(rt_data->axis_data[AXIS_Z].motor_cmd_pos, par_axis[AXIS_Z]);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_cmd_Pulses);
}

static void compute_screw_comp() {
    double pitch_comp_val_x, pitch_comp_val_y, pitch_comp_val_z;
    double perp_comp_val_x, perp_comp_val_y, perp_comp_val_z;

    // 螺距误差补偿值计算 (计算出来的补偿值 单位为:mm)
    pitch_comp_val_x = 0;
    pitch_comp_val_y = 0;
    pitch_comp_val_z = 0;

    // 垂直度误差补偿值计算 (计算出来的补偿值 单位为:mm)
    perp_comp_val_x = 0;
    perp_comp_val_y = 0;
    perp_comp_val_z = 0;

    // 开始补偿
    rt_data->axis_data[AXIS_X].motor_cmd_pos = var_axis[AXIS_X][VAR_AXIS_CMD_POS_EX].value.f + pitch_comp_val_x + perp_comp_val_x;
    rt_data->axis_data[AXIS_Y].motor_cmd_pos = var_axis[AXIS_Y][VAR_AXIS_CMD_POS_EX].value.f + pitch_comp_val_y + perp_comp_val_y;
    rt_data->axis_data[AXIS_Z].motor_cmd_pos = var_axis[AXIS_Z][VAR_AXIS_CMD_POS_EX].value.f + pitch_comp_val_z + perp_comp_val_z;

    // 输出给VAR变量后面用
    rt_data->axis_data[AXIS_X].pitch_comp_val = pitch_comp_val_x;
    rt_data->axis_data[AXIS_Y].pitch_comp_val = pitch_comp_val_y;
    rt_data->axis_data[AXIS_Z].pitch_comp_val = pitch_comp_val_z;

    rt_data->axis_data[AXIS_X].perp_comp_val = perp_comp_val_x;
    rt_data->axis_data[AXIS_Y].perp_comp_val = perp_comp_val_y;
    rt_data->axis_data[AXIS_Z].perp_comp_val = perp_comp_val_z;
}

static void check_for_soft_limit() {
    runtime[0].current_position = rt_data->axis_data[AXIS_X].motor_cmd_pos;
    runtime[1].current_position = rt_data->axis_data[AXIS_Y].motor_cmd_pos;
    runtime[2].current_position = rt_data->axis_data[AXIS_Z].motor_cmd_pos;
    runtime[0].active_levels = Check_Axis_Limit_Violation(runtime[0], true);
    runtime[1].active_levels = Check_Axis_Limit_Violation(runtime[1], true);
    runtime[2].active_levels = Check_Axis_Limit_Violation(runtime[2], true);
}

// 更新运动状态数据
static void update_status() {
    Axis_Status_Update();
    shm_var_update();
}

