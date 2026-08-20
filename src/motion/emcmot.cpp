#include "emcmot.h"
#include "alarm.h"
#include "auto.h"
#include "backlash_comp.h"
#include "comp_data.h"
#include "debug_mode.h"
#include "egear_ratio.h"
#include "ferror_control.h"
#include "global.h"
#include "handwheel.h"
#include "hmisysapi.h"
#include "homing.h"
#include "jog.h"
#include "master.h"
#include "parser.h"
#include "pathplan.h"
#include "pitch_comp.h"
#include "rt_data.h"
#include "rt_sem.h"
#include "spl_control.h"
#include "stored_stroke_check.h"
#include "straightness_comp.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>

static void chan_do_period();
static void smx_do_period();
static void axis_write_out();
static void check_for_fault();
static void set_operating_mode();
static void spindle_control();
static void get_cmd_pos();
static void update_status();
static void output_to_hal();
static void read_from_hal();
static void compute_error_comp();
static void calcul_mac_cmd_pos();
static void manual_unset_sys_state();
static void reset_clean_warning();
static void rebuild_axis_positions_from_feedback();

#ifdef UNIT_TEST
void emcmot_test_rebuild_axis_positions_from_feedback()
{
    rebuild_axis_positions_from_feedback();
    update_pos_flag = 1;
}
#endif

static int32_t x_cmd_Pulses, y_cmd_Pulses, z_cmd_Pulses;
static int32_t x_target_pos, y_target_pos, z_target_pos;
const char *mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "回零", "空闲"};

// 轨迹规划
void emcmotTrajPlan()
{
    // todo：什么时候op_flag=0，为0应统一关闭线程，报错退出系统
    if (op_flag == 0)
    {
        return;
    }
    // 通道周期状态机
    chan_do_period();
    // 运动同步控制
    smx_do_period();
    // 轴输出与伺服处理
    axis_write_out();
    // 清除事件
    rt_data->nc_event = SYS_EVENT_NONE;
}

// 路径规划
void emcmotPathPlan()
{
    // 遍历多个通道
    for (int i = 0; i < var_sys[VAR_SYS_CHAN_NUM].value.i; i++)
    {
        if (rt_get_chan_state(i, CHAN_IS_PARSER_RERUN))
        {
            // LOG(LOG_LEVEL_INFO, "初始化译码器");
            parser_init(rt_data->current_mode);
            rt_set_chan_state(i, CHAN_IS_FIRST_RUN, true);
            rt_set_chan_state(i, CHAN_IS_PARSER_RERUN, false);
        }
        // 解释器译码
        parser_run_mode();
        // 添加block
        chan_add_block();
        // 预处理
        // pretreat_block();
        // 一致性处理
        // unif_block();
    }
}

// 通道周期状态机
static void chan_do_period()
{
    // 遍历每个通道
    for (int i = 0; i < var_sys[VAR_SYS_CHAN_NUM].value.i; i++)
    {
        // 当前活动通道
        setVarSysInt(VAR_SYS_ACTIVE_CHAN, i, 0);
        // 复位清除警告
        reset_clean_warning();
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
    }
}

// 运动同步控制
static void smx_do_period()
{
    // 刚性攻丝同步（进给 = 转速 × 螺距：F = S × Pitch）
    // 主轴位置 Z = Pitch × (θ/360°)
    // double z_vel = spl_data.current_angle * cycle_tp->plan_block->screw_data.screw_offset;
    // rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur += z_vel * (double)par_ncu[PAR_NCU_CYCLE].value.val_uint /
    // TIME_US_2_S;

    // 主轴同步

    // 重叠控制
}

// 轴输出与伺服处理
static void axis_write_out()
{
    // 计算mac_cmd_pos
    calcul_mac_cmd_pos();
    // 误差补偿
    compute_error_comp();
    // 更新轴状态
    Axis_Status_Update();
    // 预先检查插补位置是否超软限位
    check_for_soft_limit();
    // 更新抽象层数据
    output_to_hal();
    // 上传运动状态数据
    update_status();
}

static void set_operating_mode()
{
    // 急停拍下时不允许切换模式
    if (rt_get_sys_state(SYS_ESTOP_INPUT))
        return;
    // 如果没有切换，不进行操作
    if (rt_data->current_mode == rt_data->target_mode)
    {
        rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
        return;
    }
    rt_data->sys_state[SYS_IS_SWITCHING_MODE] = true;
    // 根据切换的加工模式和当前加工模式，判断是否可以切换
    switch (rt_data->current_mode)
    {
    case MODE_IDLE:
        rt_data->current_mode = rt_data->target_mode;
        break;
    case MODE_EDIT:
    case MODE_MPG:
        rt_data->current_mode = rt_data->target_mode;
        LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
        break;
    case MODE_JOG:
        // 当前点动速度为零,才能切模式成功
        if (fabs(jog_tp[AXIS_X].curr_vel) < SPEED_THRESHOLD && fabs(jog_tp[AXIS_Y].curr_vel) < SPEED_THRESHOLD &&
            fabs(jog_tp[AXIS_Z].curr_vel) < SPEED_THRESHOLD)
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
        }
        break;
    case MODE_HOMING:
        if (!var_chan[0][VAR_CHAN_IS_HOMING].value.i)
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
        }
        break;
    case MODE_MDI:
    case MODE_MEM:
    case MODE_DNC:
        if (!var_chan[0][VAR_CHAN_IS_RUNNING].value.i)
        {
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
static void spindle_control()
{
    spindle_set_state();
    if (spindle.mode != DRV_MODE_DISABLED)
    {
        spindle_cal_speed();
    }
#ifdef DEBUG_MODE_ENABLED
    setVarChanInt(0, VAR_CHAN_S_CODE, (int32_t)spl_data.current_speed, 0);
#else
    setVarChanInt(0, VAR_CHAN_S_CODE, spindle.actual_vel, 0);
#endif
}

// 不同加工模式的运动控制
static void get_cmd_pos()
{
    // todo：临时，机械锁住第一阶段，不支持手动模式
    manual_unset_sys_state();
    switch (rt_data->current_mode)
    {
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
    default:        // error
        break;
    }
}

static void check_for_fault()
{
    if (rt_get_sys_state(SYS_ESTOP_INPUT))
    {
        handle_estop_cmd();
    }
    // 检查系统报警(去除用户PLC报警)和内部急停标志
    int alarm_count0 = 0;
    int alarm_count1 = 0;
    int alarm_count2 = 0;
    int alarm_count3 = 0;
    int alarm_count4 = 0;
    int alarm_count5 = 0;
    int alarm_count6 = 0;
    int alarm_count7 = 0;
    int alarm_count8 = 0;
    getAlarmCount(ALARM_SY, ALARM_ERROR, &alarm_count0);
    getAlarmCount(ALARM_CH, ALARM_ERROR, &alarm_count1);
    getAlarmCount(ALARM_AX, ALARM_ERROR, &alarm_count2);
    getAlarmCount(ALARM_SV, ALARM_ERROR, &alarm_count3);
    getAlarmCount(ALARM_PC, ALARM_ERROR, &alarm_count4);
    getAlarmCount(ALARM_DV, ALARM_ERROR, &alarm_count5);
    getAlarmCount(ALARM_PS, ALARM_ERROR, &alarm_count6);
    getAlarmCount(ALARM_HM, ALARM_ERROR, &alarm_count7);
    getAlarmCount(ALARM_ETHCAT_SV, ALARM_ERROR, &alarm_count8);

    int alarm_count = alarm_count0 + alarm_count1 + alarm_count2 + alarm_count3 + alarm_count4 + alarm_count5 +
                      alarm_count6 + alarm_count7 + alarm_count8;

    if (alarm_count > 0 || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        rt_data->sys_state[SYS_IS_ALARMING] = true;
        // 除急停外的其他报警，取消准备就绪信号
        if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
            rt_set_sys_state(SYS_IS_READY, false);
        handle_alarm_cmd();
    }
    if (alarm_count <= 0)
    {
        rt_data->sys_state[SYS_IS_ALARMING] = false;
        rt_set_sys_state(SYS_IS_READY, true);
    }
}

static void read_from_hal() {
    // 读取实际位置
    #ifdef DEBUG_MODE_ENABLED
    rt_data->axis_data[AXIS_X].motor_act_pos = rt_data->axis_data[AXIS_X].motor_cmd_pos;
    rt_data->axis_data[AXIS_Y].motor_act_pos = rt_data->axis_data[AXIS_Y].motor_cmd_pos;
    rt_data->axis_data[AXIS_Z].motor_act_pos = rt_data->axis_data[AXIS_Z].motor_cmd_pos;
    #else
    Hal_Driver_Get_Actual_Pos(&x_axis_driver, &x_target_pos);
    rt_data->axis_data[AXIS_X].motor_act_pos = pulses_to_position(&rt_data->axis_data[AXIS_X], x_target_pos);
    Hal_Driver_Get_Actual_Pos(&y_axis_driver, &y_target_pos);
    rt_data->axis_data[AXIS_Y].motor_act_pos = pulses_to_position(&rt_data->axis_data[AXIS_Y], y_target_pos);
    Hal_Driver_Get_Actual_Pos(&z_axis_driver, &z_target_pos);
    rt_data->axis_data[AXIS_Z].motor_act_pos = pulses_to_position(&rt_data->axis_data[AXIS_Z], z_target_pos);
    #endif
    // 电机脉冲
    rt_data->axis_data[AXIS_X].enc_cntr = x_target_pos;
    rt_data->axis_data[AXIS_Y].enc_cntr = y_target_pos;
    rt_data->axis_data[AXIS_Z].enc_cntr = z_target_pos;
    // 读取实际速度
    rt_data->axis_data[AXIS_X].act_vel = x_axis_driver.actual_vel;
    rt_data->axis_data[AXIS_Y].act_vel = y_axis_driver.actual_vel;
    rt_data->axis_data[AXIS_Z].act_vel = z_axis_driver.actual_vel;
    rt_data->chan_data[0].act_vel = sqrt(pow(x_axis_driver.actual_vel, 2) + pow(y_axis_driver.actual_vel, 2) + pow(z_axis_driver.actual_vel, 2));

    if (!update_pos_flag)
    {
        // 报警复位后用最新带补偿反馈，重建未补偿坐标基准。
        rebuild_axis_positions_from_feedback();
        update_pos_flag = 1;
    }
}

static void output_to_hal()
{
    // 发送给驱动器
    x_cmd_Pulses = position_to_pulses(&rt_data->axis_data[AXIS_X], rt_data->axis_data[AXIS_X].motor_cmd_pos);
    Hal_Driver_Set_Target_Pos(&x_axis_driver, x_cmd_Pulses);
    y_cmd_Pulses = position_to_pulses(&rt_data->axis_data[AXIS_Y], rt_data->axis_data[AXIS_Y].motor_cmd_pos);
    Hal_Driver_Set_Target_Pos(&y_axis_driver, y_cmd_Pulses);
    z_cmd_Pulses = position_to_pulses(&rt_data->axis_data[AXIS_Z], rt_data->axis_data[AXIS_Z].motor_cmd_pos);
    Hal_Driver_Set_Target_Pos(&z_axis_driver, z_cmd_Pulses);
}

static void compute_error_comp()
{
    double pitch_comp_val[3] = {0.0};
    double backlash_comp_val[3] = {0.0};
    double straightness1_comp_val[3] = {0.0};
    double straightness2_comp_val[3] = {0.0};

    // todo 事件驱动：仅在 HMI 下发补偿事件时同步参数/重建表
    comp_handle_event(rt_data->nc_event);

    bool is_rapid = false;
    if (cycle_tp->plan_block->type == RAPID || rt_data->sys_state[SYS_IS_RAPID])
    {
        is_rapid = true;
    }

    // 构建 all_axis_pos[] 供跨轴补偿使用
    double all_axis_pos[3];
    for (int i = 0; i < 3; i++)
    {
        all_axis_pos[i] = rt_data->axis_data[i].mac_cmd_pos;
    }

    for (int i = 0; i < 3; i++)
    {
        double mac_cmd_pos = rt_data->axis_data[i].mac_cmd_pos;
        // 螺距误差补偿值计算(计算出来的补偿值 单位为:mm)
        pitch_comp_val[i] = calcul_pitch_comp_val(&pitch_comp_state[i],
                                                  &par_comp_lib[i],
                                                  &pitch_comp_table[i],
                                                  mac_cmd_pos);
        // 反向间隙补偿值计算(计算出来的补偿值 单位为:mm)
        backlash_comp_val[i] = calcul_backlash_comp_val(&par_comp_lib[i],
                                                        &backlash_comp_state[i],
                                                        mac_cmd_pos,
                                                        is_rapid);
        // 第一组直线度误差补偿值计算
        straightness1_comp_val[i] = 0.0;
        // straightness1_comp_val[i] = calcul_straightness1_comp_val(&par_comp_lib[i],
        //                                                           &straightness1_comp_state[i],
        //                                                           i,
        //                                                           all_axis_pos,
        //                                                           &straightness1_comp_table[i]);

        // 第二组直线度误差补偿值计算
        straightness2_comp_val[i] = 0.0;
        // straightness2_comp_val[i] = calcul_straightness2_comp_val(&par_comp_lib[i],
        //                                                           &straightness2_comp_state[i],
        //                                                           i,
        //                                                           all_axis_pos,
        //                                                           &straightness2_comp_table[i]);

        // 补偿值同步到全局误差补偿变量中
        rt_data->axis_data[i].pitch_comp_val = pitch_comp_val[i];
        rt_data->axis_data[i].backlash_comp_val = backlash_comp_val[i];
        rt_data->axis_data[i].straightness1_comp_val = straightness1_comp_val[i];
        rt_data->axis_data[i].straightness2_comp_val = straightness2_comp_val[i];

        rt_data->axis_data[i].total_comp_cal = rt_data->axis_data[i].pitch_comp_val +
                                               rt_data->axis_data[i].backlash_comp_val +
                                               rt_data->axis_data[i].straightness1_comp_val +
                                               rt_data->axis_data[i].straightness2_comp_val;

        // 下发电机位置 (当前周期插补位置 + 总补偿值)
        rt_data->axis_data[i].motor_cmd_pos = mac_cmd_pos + rt_data->axis_data[i].total_comp_cal;

        // 反馈位置（去除补偿）
        rt_data->axis_data[i].fb_pos_no_comp = rt_data->axis_data[i].motor_act_pos -
                                               rt_data->axis_data[i].total_comp_cal;
    }
    // 螺补
    // printf("pitch_comp_val%f\n",rt_data->axis_data[AXIS_X].pitch_comp_val);
    // printf("mac_cmd_pos_x %f motor_act_pos %f driver_actual_pos %d total_comp_cal %f chazhi %f motor_cmd_pos %f\n",
    //        rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //        rt_data->axis_data[AXIS_X].motor_act_pos,
    //        x_axis_driver.actual_pos,
    //        rt_data->axis_data[AXIS_X].total_comp_cal,
    //        rt_data->axis_data[AXIS_X].motor_act_pos - rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //        rt_data->axis_data[AXIS_X].motor_cmd_pos);
#ifdef ENABLE_COMP_DATA_LOG
    {
        static FILE *comp_log_file = NULL;
        static int comp_log_cycle = 0;

        if (comp_log_file == NULL)
        {
            comp_log_file = fopen(PROJECT_DIR "/../comp_data_log.csv", "w");
        }

        if (comp_log_file != NULL)
        {
            fprintf(comp_log_file,
                    "%d,%.10f,%.10f\n",
                    comp_log_cycle,
                    rt_data->axis_data[0].mac_cmd_pos,
                    pitch_comp_val[0]);
            fflush(comp_log_file);
            comp_log_cycle++;
        }
    }
#endif
}

static void calcul_mac_cmd_pos()
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        // 机床锁住或者轴拆卸，不下发位置
        if (!rt_data->sys_state[SYS_IS_MAC_LOCK_READY] && !rt_data->axis_data[i].axis_status.is_disable)
        {
            rt_data->axis_data[i].mac_cmd_pos += (rt_data->axis_data[i].wcs_cmd_pos_cur -
                                                  rt_data->axis_data[i].wcs_cmd_pos_pre);
        }
        // 上一周期插补位置
        rt_data->axis_data[i].wcs_cmd_pos_pre = rt_data->axis_data[i].wcs_cmd_pos_cur;
    }
}

// 更新运动状态数据
static void update_status()
{
    // 更新var变量
    shm_var_update();
    // 更新复位状态
    nc_set_resetted();
}

static void reset_clean_warning()
{
    if (rt_data->nc_event == SYS_EVENT_HMI_RESET)
    {
        // 4.按下MDI上的RESET键
        nc_set_resetting(HMI_RESET);
        // 复位生效参数
        Axis_Limit_Manager_Init();
        // 清除警告提示
        cleanAllAlarm(ALARM_SY, ALARM_WARNING);
        cleanAllAlarm(ALARM_CH, ALARM_WARNING);
        cleanAllAlarm(ALARM_AX, ALARM_WARNING);
        cleanAllAlarm(ALARM_SV, ALARM_WARNING);
        cleanAllAlarm(ALARM_DV, ALARM_WARNING);
        cleanAllAlarm(ALARM_PS, ALARM_WARNING);
        cleanAllAlarm(ALARM_HM, ALARM_WARNING);
        cleanAllAlarm(ALARM_ETHCAT_SV, ALARM_WARNING);
    }
}

static void manual_unset_sys_state()
{
    if (rt_data->current_mode == MODE_MDI || rt_data->current_mode == MODE_MEM || rt_data->current_mode == MODE_DNC)
        return;
    // todo：手动模式暂不支持机械锁住
    if (rt_data->sys_state[SYS_IS_MAC_LOCK_READY])
        rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
    if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON])
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = false;
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_MAC_LOCK_ON:
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_WARN, "手动模式暂不支持机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = false;
        break;
    case SYS_EVENT_BLOCK_SKIP_ON:
        rt_set_sys_state(SYS_IS_BLOCK_SKIP, true);
        break;
    case SYS_EVENT_BLOCK_SKIP_OFF:
        rt_set_sys_state(SYS_IS_BLOCK_SKIP, false);
        break;
    default:
        break;
    }
}

static void rebuild_axis_positions_from_feedback()
{
    // motor_act_pos 是带补偿反馈；wcs_cmd_pos_*、mac_cmd_pos 和 fb_pos_no_comp 是未补偿机床坐标。
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {

        // 暂时这样做，后续等待断电保持
        rt_data->axis_data[i].motor_cmd_pos = rt_data->axis_data[i].motor_act_pos;
        double uncompensated_pos;
        uncompensated_pos = rt_data->axis_data[i].motor_act_pos - rt_data->axis_data[i].total_comp_cal;
        rt_data->axis_data[i].mac_cmd_pos = uncompensated_pos;
        rt_data->axis_data[i].wcs_cmd_pos_cur = uncompensated_pos;
        rt_data->axis_data[i].wcs_cmd_pos_pre = uncompensated_pos;
        rt_data->axis_data[i].fb_pos_no_comp = uncompensated_pos;
    }
    // 暂时这样做，后续等待断电保持
    // 　补偿状态机
    for (int axis = 0; axis < 3; axis++)
    {
        // 螺距补偿
        // pitch_comp_seqlist_init(&pitch_comp_table[axis]);
        // pitch_comp_init(&pitch_comp_state[axis], 0.001);
        pitch_comp_state[axis].last_pos_cmd = rt_data->axis_data[axis].mac_cmd_pos;
        // 反向间隙
        // backlash_comp_init(&backlash_comp_state[axis]);
        backlash_comp_state[axis].last_pos_cmd = rt_data->axis_data[axis].mac_cmd_pos;
        // 直线度补偿
        // straightness_comp_seqlist_init(&straightness1_comp_table[axis]);
        // straightness_comp_seqlist_init(&straightness2_comp_table[axis]);
        // straightness_comp_init(&straightness1_comp_state[axis], 0.001);
        // straightness_comp_init(&straightness2_comp_state[axis], 0.001);
    }
}
