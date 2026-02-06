#include "emcmot.h"
#include "global.h"
#include "log.h"
#include "egear_ratio.h"
#include "mcode_queue.h"
#include "ferror_control.h"
#include "pathplan.h"
#include "rt_sem.h"
#include "spindle.h"
#include "alarm.h"
#include "rapid.h"
#include "jog.h"
#include "mpg.h"
#include "homing.h"
#include "mem.h"
#include "spindle.h"
#include "stored_stroke_check.h"
#include "parser.h"
#include "hmisysapi.h"
#include "logapi.h"
#include <alarmapi.h>
#include <math.h>

static void check_for_fault();
static void set_operating_mode();
static void handle_system_event();
static void spindle_control();
static void get_cmd_pos();
static void update_status();
static void check_ferror_fault();
static void output_to_hal();
static void read_to_hal();
static void check_for_soft_limit();
static void compute_screw_comp();

static int32_t x_cmd_Pulses, y_cmd_Pulses, z_cmd_Pulses;
static int32_t x_target_pos, y_target_pos, z_target_pos;
static int preop_flag = 0;
static int first_flag = 0;
char* mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "快速点动", "回零", "报警", "空闲"};

// 轨迹规划
void emcmotTrajPlan() {
    if (op_flag == 0) {
        return;
    }

    // 遍历每个通道
    for (int i = 0; i < var_sys.chan_num; i++) {
        // 当前活动通道
        var_sys.active_chan = i;

        // 读取抽象层数据
        read_to_hal();

        // 检查系统是否有异常
        check_for_fault();

        // 判断运动模式
        set_operating_mode();

        // 处理事件
        // handle_system_event();

        // 临时：主轴控制
        spindle_control();

        // 根据运动模式计算运动位置
        get_cmd_pos();

        // 误差补偿
        compute_screw_comp();

        // 预先检查插补位置是否超软限位
        check_for_soft_limit();

        // 更新抽象层数据
        output_to_hal();

        // 上传运动状态数据
        update_status();
    }
}

// 路径规划
void emcmotPathPlan() {
    if (sys->is_parse_running){
        HmiSysData hmiSysData;
        #ifdef DEBUG_MODE_ENABLED
        char ncFilePath[512];
        snprintf(ncFilePath, sizeof(ncFilePath), "%s/%s", PROJECT_DIR, "config/NC/a.nc");
        parser_init(ncFilePath);
        #else
        getHmiSysDataPtr(&hmiSysData);
        parser_init(hmiSysData.ncFilePath);
        #endif
        sys->is_first_run = true;   // 通知预处理第一次获取pre_pos
        sys->is_parse_running = false;
    }
    // 遍历多个通道
    for (int i = 0; i < var_sys.chan_num; i++) {
        // 解释器译码
        parser_run_mode();
        // 预处理，添加block
        chan_add_block();
    }
}

static void set_operating_mode() {
    // 如果没有切换，不进行操作
    if (sys->current_mode == sys->target_mode) {
        sys->is_switching_mode = false;
        return;
    }
    // 检查是否急停
    if (var_chan[0].is_estop) {
        return;
    }
    // 根据切换的加工模式和当前加工模式，判断是否可以切换
    switch (sys->current_mode) {
        case MODE_IDLE:
            sys->current_mode = sys->target_mode;
            var_chan[var_sys.active_chan].mode = sys->current_mode;
            break;
        case MODE_JOG:
            // 当前点动速度为零,才能切模式成功
            if (fabs(var_axis[AXIS_X].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD 
            && fabs(var_axis[AXIS_Y].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD 
            && fabs(var_axis[AXIS_Z].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD) 
            {
                sys->current_mode = sys->target_mode;
                sys->is_switching_mode = false;
                var_chan[var_sys.active_chan].mode = sys->current_mode;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
                LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
                break;
            }
            sys->nc_event = SYS_EVENT_RESET;
            sys->is_switching_mode = true;
            break;
        case MODE_EDIT:
        case MODE_MPG:
            sys->current_mode = sys->target_mode;
            var_chan[var_sys.active_chan].mode = sys->current_mode;
            // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
            LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
            break;
        case MODE_RAPID_JOG:
            // 当前快速点动速度为零,才能切模式成功
            if (fabs(var_axis[AXIS_X].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD 
            && fabs(var_axis[AXIS_Y].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD 
            && fabs(var_axis[AXIS_Z].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD) 
            {
                sys->current_mode = sys->target_mode;
                sys->is_switching_mode = false;
                var_chan[var_sys.active_chan].mode = sys->current_mode;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
                LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
                break;
            }
            sys->nc_event = SYS_EVENT_RESET;
            sys->is_switching_mode = true;
            sys->current_mode = sys->target_mode;
            break;
        case MODE_ALARM:
            sys->current_mode = sys->target_mode;
            var_chan[var_sys.active_chan].mode = sys->current_mode;
            // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
            LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
            break;
        case MODE_HOMING:
            if (!var_chan[0].is_homing) {
                sys->current_mode = sys->target_mode;
                sys->is_switching_mode = false;
                var_chan[var_sys.active_chan].mode = sys->current_mode;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
                LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
                break;
            }
            sys->nc_event = SYS_EVENT_RESET;
            sys->is_switching_mode = true;
            break;

        case MODE_MDI:
        case MODE_MEM:
        case MODE_DNC:
            if (mem_exec_state == EXEC_IDLE) {
                sys->current_mode = sys->target_mode;
                sys->is_switching_mode = false;
                var_chan[var_sys.active_chan].mode = sys->current_mode;
                // log_system("[emcmot] 系统模式切换为(%s)", mode_names[sys->current_mode]);
                LOG_OUTPUT(INFO, "系统模式切换为 %s", mode_names[sys->current_mode]);
                break;
            } else if (mem_exec_state == EXEC_RUNNING) {
                // 加工下切换模式不能生效
                sys->target_mode = sys->current_mode;
                // 自动加工过程中不能切换模式，应先暂停
                // log_system("[emcmot] 自动加工下不能切换模式，请先按进给保持后切换");
                LOG_OUTPUT(INFO, "自动加工下不能切换模式，请先按进给保持后切换");
                break;
            }
            sys->nc_event = SYS_EVENT_RESET;
            sys->is_switching_mode = true;
            break;
        default:
            break;
    }
}

static void handle_system_event(){
    switch (sys->nc_event)
    {
    case SYS_EVENT_RESET:
    case SYS_EVENT_PAUSE:
        break;
    default:
        break;
    }
}

// 主轴控制
static void spindle_control(){
    if(var_chan[0].is_estop) return;
    switch (sys->current_mode){
        case MODE_JOG:
        case MODE_MPG:
        case MODE_EDIT:
            emcSpindleControl();
            break;
        case MODE_MEM:
        case MODE_MDI:
        case MODE_DNC:
            emcmcodedeal();
            break;
        default:
            break;
    }
}

// 不同加工模式的运动控制
static void get_cmd_pos() {
    switch (sys->current_mode) {
        case MODE_IDLE: // 空闲状态
            break;
        case MODE_EDIT: // 编辑
            break;
        case MODE_ALARM: // 系统报错
            handle_alarm_cmd();
            break;
        case MODE_HOMING: // 回零
            // handle_homing_cmd();
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
    //　检查系统报警
    int alarm_count = 0;
    getAlarmCount(ALARM_TYPE_ALL, ALARM_LEVEL_ALL, &alarm_count);
    if(alarm_count > 0){
        var_chan[0].is_estop = true;
        sys->target_mode = sys->current_mode;
        sys->current_mode = MODE_ALARM;
        var_chan[var_sys.active_chan].mode = sys->current_mode;
    }
    // 检查跟随误差
    check_ferror_fault();
}

// 检查轴跟随误差错误
static void check_ferror_fault() {
    // 指令位置
    double pos_cmd_x = var_axis[AXIS_X].motor_cmd_pos;
    double pos_cmd_y = var_axis[AXIS_Y].motor_cmd_pos;
    double pos_cmd_z = var_axis[AXIS_Z].motor_cmd_pos;
    // 反馈位置
    double pos_fb_x = var_axis[AXIS_X].motor_act_pos;
    double pos_fb_y = var_axis[AXIS_Y].motor_act_pos;
    double pos_fb_z = var_axis[AXIS_Z].motor_act_pos;
    // 指令速度
    double vel_cmd_x = var_axis[AXIS_X].cmd_vel;
    double vel_cmd_y = var_axis[AXIS_Y].cmd_vel;
    double vel_cmd_z = var_axis[AXIS_Z].cmd_vel;

    updata_axis_ferror(&var_axis[AXIS_X].ferror_control, &par_axis[AXIS_X], pos_cmd_x, pos_fb_x, vel_cmd_x);
    updata_axis_ferror(&var_axis[AXIS_Y].ferror_control, &par_axis[AXIS_Y], pos_cmd_y, pos_fb_y, vel_cmd_y);
    updata_axis_ferror(&var_axis[AXIS_Z].ferror_control, &par_axis[AXIS_Z], pos_cmd_z, pos_fb_z, vel_cmd_z);

    if (var_axis[AXIS_X].ferror_control.error_flag) {
        // setAlarm(ALARM_CH, ALARM_ERROR, 4);
        // log_system("[ferror] Ｘ轴跟随误差报警");
        // LOG_OUTPUT(ERROR, "Ｘ轴跟随误差报警");
    }
    if (var_axis[AXIS_Y].ferror_control.error_flag) {
        // setAlarm(ALARM_CH, ALARM_ERROR, 5);
        // log_system("[ferror] Ｙ轴跟随误差报警");
        // LOG_OUTPUT(ERROR, "Ｙ轴跟随误差报警");
    }
    if (var_axis[AXIS_Z].ferror_control.error_flag) {
        // setAlarm(ALARM_CH, ALARM_ERROR, 6);
        // log_system("[ferror] Ｚ轴跟随误差报警");
        // LOG_OUTPUT(ERROR, "Ｚ轴跟随误差报警");
    }
}

static void read_to_hal() {
    // 读取实际位置
    Hal_Driver_Get_Actual_Pos(&x_axis_driver, &x_target_pos);
    var_axis[AXIS_X].motor_act_pos = pulses_to_position(x_target_pos, &par_axis[AXIS_X]);
    Hal_Driver_Get_Actual_Pos(&y_axis_driver, &y_target_pos);
    var_axis[AXIS_Y].motor_act_pos = pulses_to_position(y_target_pos, &par_axis[AXIS_Y]);
    Hal_Driver_Get_Actual_Pos(&z_axis_driver, &z_target_pos);
    var_axis[AXIS_Z].motor_act_pos = pulses_to_position(z_target_pos, &par_axis[AXIS_Z]);

    // 人机交互显示的位置值，要减去误差补偿值.
    var_axis[AXIS_X].act_pos = var_axis[AXIS_X].motor_act_pos - 0;
    var_axis[AXIS_Y].act_pos = var_axis[AXIS_Y].motor_act_pos - 0;
    var_axis[AXIS_Z].act_pos = var_axis[AXIS_Z].motor_act_pos - 0;

    // 第一次运行，初始化位置
    if (!preop_flag) {
        var_axis[AXIS_X].cmd_pos = var_axis[AXIS_X].motor_act_pos;
        var_axis[AXIS_Y].cmd_pos = var_axis[AXIS_Y].motor_act_pos;
        var_axis[AXIS_Z].cmd_pos = var_axis[AXIS_Z].motor_act_pos;
        var_axis[AXIS_X].motor_act_pos = var_axis[AXIS_X].cmd_pos;
        var_axis[AXIS_Y].motor_act_pos = var_axis[AXIS_Y].cmd_pos;
        var_axis[AXIS_Z].motor_act_pos = var_axis[AXIS_Z].cmd_pos;
        preop_flag = 1;
    }
}

static void output_to_hal() {
    x_cmd_Pulses = position_to_pulses(var_axis[AXIS_X].motor_cmd_pos, &par_axis[AXIS_X]);
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_cmd_Pulses);
    y_cmd_Pulses = position_to_pulses(var_axis[AXIS_Y].motor_cmd_pos, &par_axis[AXIS_Y]);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_cmd_Pulses);
    z_cmd_Pulses = position_to_pulses(var_axis[AXIS_Z].motor_cmd_pos, &par_axis[AXIS_Z]);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_cmd_Pulses);
}

STATIC void compute_screw_comp() {
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
    var_axis[AXIS_X].motor_cmd_pos = var_axis[AXIS_X].cmd_pos + pitch_comp_val_x + perp_comp_val_x;
    var_axis[AXIS_Y].motor_cmd_pos = var_axis[AXIS_Y].cmd_pos + pitch_comp_val_y + perp_comp_val_y;
    var_axis[AXIS_Z].motor_cmd_pos = var_axis[AXIS_Z].cmd_pos + pitch_comp_val_z + perp_comp_val_z;

    // 输出给VAR变量后面用
    var_axis[AXIS_X].pitch_comp_val = pitch_comp_val_x;
    var_axis[AXIS_Y].pitch_comp_val = pitch_comp_val_y;
    var_axis[AXIS_Z].pitch_comp_val = pitch_comp_val_z;

    var_axis[AXIS_X].perp_comp_val = perp_comp_val_x;
    var_axis[AXIS_Y].perp_comp_val = perp_comp_val_y;
    var_axis[AXIS_Z].perp_comp_val = perp_comp_val_z;
}

static void check_for_soft_limit() {
    var_axis[AXIS_X].runtime.current_position = var_axis[AXIS_X].motor_cmd_pos;
    var_axis[AXIS_Y].runtime.current_position = var_axis[AXIS_Y].motor_cmd_pos;
    var_axis[AXIS_Z].runtime.current_position = var_axis[AXIS_Z].motor_cmd_pos;
    var_axis[AXIS_X].runtime.active_levels = Check_Axis_Limit_Violation(var_axis[AXIS_X].runtime);
    var_axis[AXIS_Y].runtime.active_levels = Check_Axis_Limit_Violation(var_axis[AXIS_Y].runtime);
    var_axis[AXIS_Z].runtime.active_levels = Check_Axis_Limit_Violation(var_axis[AXIS_Z].runtime);
    if(var_axis[AXIS_X].runtime.active_levels || var_axis[AXIS_Y].runtime.active_levels || var_axis[AXIS_Z].runtime.active_levels){
        // setAlarm(ALARM_CH, ALARM_ERROR, 7);
        // log_system("[limit] 软限位超差");
        // LOG_OUTPUT(ERROR, "软限位超差");
    }
}

// 更新运动状态数据
static void update_status() {
    var_chan[0].act_feedrate = var_chan[0].cmd_feedrate;
    var_chan[0].act_acc = var_chan[0].cmd_acc;
    var_chan[0].act_jerk = var_chan[0].cmd_jerk;
    var_chan[0].act_spdl_speed = var_chan[0].act_spdl_speed;
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        var_axis[i].act_vel = var_axis[i].cmd_vel;
    }
    shm_var_update();
    rt_par_update();
}

