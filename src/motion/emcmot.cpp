#include "emcmot.h"
#include "alarm.h"
#include "auto.h"
#include "backlash_comp.h"
#include "comp_data.h"
#include "debug_mode.h"
#include "ferror_control.h"
#include "global.h"
#include "handwheel.h"
#include "hmisysapi.h"
#include "ref.h"
#include "rt_hal_adapter.h"
#include "jog.h"
#include "parser.h"
#include "pathplan.h"
#include "pitch_comp.h"
#include "rt_data.h"
#include "rerun.h"
#include "reset.h"
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
static void get_cmd_pos();
static void update_status();
static void output_to_hal();
static void read_from_hal();
static void compute_error_comp();
static void calcul_mac_cmd_pos();
static void reset_clean_warning();
static void rebuild_axis_positions_from_feedback();
static void handle_rigid_return_event();

#ifdef UNIT_TEST
void emcmot_test_rebuild_axis_positions_from_feedback()
{
    rebuild_axis_positions_from_feedback();
    update_pos_flag = 1;
}
#endif

const char *mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "回零", "空闲"};

// 轨迹规划
void emcmotTrajPlan()
{
    // 主站未进入OP或当前周期通信失败时，跳过本周期运动计算。
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
        pretreat_block();
        // 一致性处理
        unif_block();
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
        spindle_control_update();
        // 根据运动模式计算运动位置
        get_cmd_pos();
    }
}

// 运动同步控制
static void smx_do_period()
{ 
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
    NC_MODE previous_mode = rt_data->current_mode;

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
        rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
        break;
    case MODE_MPG:
        rt_data->current_mode = rt_data->target_mode;
        rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
        LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
        rerun_update_when_switching();
        break;
    case MODE_JOG:
        // 当前点动速度为零,才能切模式成功
        if (fabs(jog_get_axis_velocity(AXIS_X)) < JOG_SPEED_THRESHOLD &&
            fabs(jog_get_axis_velocity(AXIS_Y)) < JOG_SPEED_THRESHOLD &&
            fabs(jog_get_axis_velocity(AXIS_Z)) < JOG_SPEED_THRESHOLD)
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            rerun_update_when_switching();
        }
        break;
    case MODE_HOMING:
        if (ref_tp[0].ref_state == REF_ABS_EXEC_IDLE && ref_tp[1].ref_state == REF_ABS_EXEC_IDLE && ref_tp[2].ref_state == REF_ABS_EXEC_IDLE)
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            rerun_update_when_switching();
        }
        break;
    case MODE_MDI:
    case MODE_MEM:
    case MODE_DNC:
        if (cycle_tp->exec_state == EXEC_IDLE)
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            rerun_update_when_switching();
        }
        break;
    case MODE_EDIT:
        rt_data->current_mode = rt_data->target_mode;
        LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
        break;
    default:
        break;
    }

    // 切换完成后进入手轮模式时重建手轮位置基准：
    // 离开手轮模式后其他模式可能已经移动过机床，不能沿用上一次手轮模式的规划位置。
    if (previous_mode != MODE_MPG && rt_data->current_mode == MODE_MPG)
    {
        handwheel_resync_position();
    }
}

// 不同加工模式的运动控制
static void get_cmd_pos()
{
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
        handle_ref_cmd();
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

    int alarm_count = alarm_count0 + alarm_count1 + alarm_count2 + alarm_count3 + alarm_count4 + alarm_count5 + alarm_count6 +
                      alarm_count7 + alarm_count8;

    // 伺服驱动报警下清除回参考点信号
    if(alarm_count3 > 0){
        reset_ref_state();
    }
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

static void read_from_hal()
{
    /* HAL 给的就是用户单位（mm）——脉冲换算与 enc_off 语义都由 HAL 完成，
     * 这里不再调 pulses_to_position()，也不重复加减 enc_off。 */
    double pos = 0.0;
    bool feed_positions_valid = true;
    if (rt_hal_axis_pos(RT_HAL_AXIS_X, &pos) == HAL_OK)
        rt_data->axis_data[AXIS_X].motor_act_pos = pos;
    else feed_positions_valid = false;
    if (rt_hal_axis_pos(RT_HAL_AXIS_Y, &pos) == HAL_OK)
        rt_data->axis_data[AXIS_Y].motor_act_pos = pos;
    else feed_positions_valid = false;
    if (rt_hal_axis_pos(RT_HAL_AXIS_Z, &pos) == HAL_OK)
        rt_data->axis_data[AXIS_Z].motor_act_pos = pos;
    else feed_positions_valid = false;
    const bool spindle_position_valid =
        rt_hal_axis_pos(RT_HAL_AXIS_SP, &pos) == HAL_OK;
    if (spindle_position_valid)
        rt_data->axis_data[RT_HAL_AXIS_SP].motor_act_pos = pos;


    /* enc_cntr：HAL 不提供原始反馈计数，置 0。仅用于显示，不参与保护判断。 */
    rt_data->axis_data[AXIS_X].enc_cntr = 0;
    rt_data->axis_data[AXIS_Y].enc_cntr = 0;
    rt_data->axis_data[AXIS_Z].enc_cntr = 0;
    rt_data->axis_data[AXIS_SP].enc_cntr = 0;

    /* 进给轴读 HAL 换算后的 mm/min；主轴仍读 rpm。 */
    const RtHalAxisSlot feed_slot[3] = {RT_HAL_AXIS_X, RT_HAL_AXIS_Y, RT_HAL_AXIS_Z};
    const int feed_axis[3] = {AXIS_X, AXIS_Y, AXIS_Z};
    for (int i = 0; i < 3; ++i) {
        double mm_min = 0.0;
        if (rt_hal_axis_speed_mm_min(feed_slot[i], &mm_min) != HAL_OK) mm_min = 0.0;
        rt_data->axis_data[feed_axis[i]].act_vel = mm_min;
    }
    /* 主轴：HalCSpindleStatus.actual_speed，rpm（HAL 已按 speed_unit 换算）。 */
    double spd = 0.0;
    if (rt_hal_spindle_speed(RT_HAL_AXIS_SP, &spd) == HAL_OK)
        rt_data->axis_data[AXIS_SP].act_vel = spd;
    /* 进给轴实际合成速度（不含主轴）：三轴 mm/min 的欧氏模。纯显示量，不参与保护判断。 */
    {
        const double vx = rt_data->axis_data[AXIS_X].act_vel;
        const double vy = rt_data->axis_data[AXIS_Y].act_vel;
        const double vz = rt_data->axis_data[AXIS_Z].act_vel;
        rt_data->chan_data[0].act_vel = sqrt(vx * vx + vy * vy + vz * vz);
    }

    if (!update_pos_flag && feed_positions_valid &&
        (!spl_context.reset_stop_csp || spindle_position_valid))
    {
        // 报警复位后用最新带补偿反馈，重建未补偿坐标基准。
        rebuild_axis_positions_from_feedback();
        handwheel_resync_position();
        update_pos_flag = 1;
    }
}

static void output_to_hal()
{
    /* 以用户单位（mm）发布命令，只放进适配层的按拍缓冲；真正的写入由 PDO 线程
     * 在 commit 前完成。台账未清时适配层会拒收，此处有意忽略返回值。 */
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_X, rt_data->axis_data[AXIS_X].motor_cmd_pos);
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_Y, rt_data->axis_data[AXIS_Y].motor_cmd_pos);
    (void)rt_hal_request_axis_pos(RT_HAL_AXIS_Z, rt_data->axis_data[AXIS_Z].motor_cmd_pos);
    /* 刚性攻丝：把插补算的主轴绝对角度发给驱动器（主轴在 CSP 位置环里跟随）。
     * 只在"当前块是刚性攻丝"时发 —— 别的时候主轴是速度模式（CSV），写位置会被 HAL 拒。
     * 角度语义见 interpolate_tap()：从本段起点算起、跨圈累计、反向为负。 */
    if (cycle_tp->plan_block->type == RIGID_TAP || cycle_tp->plan_block->type == TAP_RETURN)
        (void)rt_hal_request_axis_pos(RT_HAL_AXIS_SP, rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur);
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
        pitch_comp_val[i] = calcul_pitch_comp_val(&pitch_comp_state[i], &par_comp_lib[i], &pitch_comp_table[i], mac_cmd_pos);
        // 反向间隙补偿值计算(计算出来的补偿值 单位为:mm)
        backlash_comp_val[i] = calcul_backlash_comp_val(&par_comp_lib[i], &backlash_comp_state[i], mac_cmd_pos, is_rapid);
        // 第一组直线度误差补偿值计算
        straightness1_comp_val[i] = calcul_straightness1_comp_val(&par_comp_lib[i],
                                                                  &straightness1_comp_state[i],
                                                                  i,
                                                                  all_axis_pos,
                                                                  &straightness1_comp_table[i]);
        // 第二组直线度误差补偿值计算
        straightness2_comp_val[i] = calcul_straightness2_comp_val(&par_comp_lib[i],
                                                                  &straightness2_comp_state[i],
                                                                  i,
                                                                  all_axis_pos,
                                                                  &straightness2_comp_table[i]);
        // 补偿值同步到全局误差补偿变量中
        rt_data->axis_data[i].pitch_comp_val = pitch_comp_val[i];
        rt_data->axis_data[i].backlash_comp_val = backlash_comp_val[i];
        rt_data->axis_data[i].straightness1_comp_val = straightness1_comp_val[i];
        rt_data->axis_data[i].straightness2_comp_val = straightness2_comp_val[i];

        rt_data->axis_data[i].total_comp_cal = rt_data->axis_data[i].pitch_comp_val + rt_data->axis_data[i].backlash_comp_val +
                                               rt_data->axis_data[i].straightness1_comp_val +
                                               rt_data->axis_data[i].straightness2_comp_val;

        // 下发电机位置 (当前周期插补位置 + 总补偿值)
        rt_data->axis_data[i].motor_cmd_pos = mac_cmd_pos + rt_data->axis_data[i].total_comp_cal;

        // 反馈位置（去除补偿）
        rt_data->axis_data[i].fb_pos_no_comp = rt_data->axis_data[i].motor_act_pos - rt_data->axis_data[i].total_comp_cal;
    }
    // 直线度补偿值
    // printf("straightness %f %f %.10f %.10f mac_x%f motor_cmd%f\n",
    //        all_axis_pos[1],
    //        all_axis_pos[2],
    //        rt_data->axis_data[AXIS_X].straightness1_comp_val,
    //        rt_data->axis_data[AXIS_X].straightness2_comp_val,
    //        rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //        rt_data->axis_data[AXIS_X].motor_cmd_pos);
    // 螺补
    // printf("mac_cmd_pos%f pitch_comp_val%f motor_cmd_pos%f \n",
    //        rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //     //    rt_data->axis_data[AXIS_Y].mac_cmd_pos,
    //     //    rt_data->axis_data[AXIS_Z].mac_cmd_pos,
    //        rt_data->axis_data[AXIS_X].pitch_comp_val,
    //     //    rt_data->axis_data[AXIS_Y].pitch_comp_val,
    //     //    rt_data->axis_data[AXIS_Z].pitch_comp_val,
    //        rt_data->axis_data[AXIS_X].motor_cmd_pos
    //     //    rt_data->axis_data[AXIS_Y].motor_cmd_pos,
    //     //    rt_data->axis_data[AXIS_Y].motor_cmd_pos,
    //     );

    // printf("mac_cmd_pos_x%f total_comp_x%f motor_cmd_pos_x%f motor_act_pos_x%f driver_act_pos_x%d\n ",
    //        rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //        rt_data->axis_data[AXIS_X].total_comp_cal,
    //        rt_data->axis_data[AXIS_X].motor_cmd_pos,
    //        rt_data->axis_data[AXIS_X].motor_act_pos,
    //        x_axis_driver.actual_pos);
    // // 编码器
    // printf("x_axis_driver.actual_pos %d\n", x_axis_driver.actual_pos);
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
            fprintf(comp_log_file, "%d,%.10f,%.10f\n", comp_log_cycle, rt_data->axis_data[0].mac_cmd_pos, pitch_comp_val[0]);
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
        if (!rt_data->sys_state[SYS_IS_MAC_LOCK_READY] && !rt_data->axis_data[i].status.is_disable)
        {
            rt_data->axis_data[i].mac_cmd_pos += (rt_data->axis_data[i].wcs_cmd_pos_cur - rt_data->axis_data[i].wcs_cmd_pos_pre);
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
        // 复位生效存储型限位功能的参数参数
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


static void rebuild_axis_positions_from_feedback()
{
    // motor_act_pos 是带补偿反馈；wcs_cmd_pos_*、mac_cmd_pos 和 fb_pos_no_comp 是未补偿机床坐标。
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        rt_data->axis_data[i].motor_cmd_pos = rt_data->axis_data[i].motor_act_pos;
        rt_data->axis_data[i].motor_act_pos_pre = rt_data->axis_data[i].motor_act_pos;
        double uncompensated_pos;
        // printf("total_comp_cal %f\n", rt_data->axis_data[i].total_comp_cal);
        uncompensated_pos = rt_data->axis_data[i].motor_act_pos - rt_data->axis_data[i].total_comp_cal;
        rt_data->axis_data[i].mac_cmd_pos = uncompensated_pos;
        rt_data->axis_data[i].wcs_cmd_pos_cur = uncompensated_pos;
        rt_data->axis_data[i].wcs_cmd_pos_pre = uncompensated_pos;
        rt_data->axis_data[i].fb_pos_no_comp = uncompensated_pos;
    }

    for (int axis = 0; axis < 3; axis++)
    {
        double mac_cmd_pos = rt_data->axis_data[axis].mac_cmd_pos;

        pitch_comp_state[axis].last_pos_cmd = mac_cmd_pos;
        pitch_comp_state[axis].pos_initialized = true;
        backlash_comp_state[axis].last_pos_cmd = mac_cmd_pos;
        backlash_comp_state[axis].pos_initialized = true;

        for (int snapshot_axis = 0; snapshot_axis < 3; snapshot_axis++)
        {
            double snapshot_pos = rt_data->axis_data[snapshot_axis].mac_cmd_pos;
            straightness1_comp_state[axis].last_axis_pos[snapshot_axis] = snapshot_pos;
            straightness2_comp_state[axis].last_axis_pos[snapshot_axis] = snapshot_pos;
        }
        straightness1_comp_state[axis].pos_initialized = true;
        straightness2_comp_state[axis].pos_initialized = true;
    }
}
