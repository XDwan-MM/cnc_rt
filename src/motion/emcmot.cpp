#include "rt_drive_control.h"
#include "motion_csp_stop.h"
#include "emcmot.h"
#include "alarm.h"
#include "auto.h"
#include "backlash_comp.h"
#include "comp_data.h"
#include "debug_mode.h"
#include "ferror_control.h"
#include "global.h"
#include "mpg.h"
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
#include "spindle_control.h"
#include "stored_stroke_check.h"
#include "straightness_comp.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

static void update_spindle_position_permission(void);
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
static void rebuild_axis_positions_from_feedback(bool spindle_valid);
static void rebuild_axis_position_from_feedback(int axis);

#ifdef UNIT_TEST
void emcmot_test_rebuild_axis_positions_from_feedback()
{
    rebuild_axis_positions_from_feedback(true);
    update_pos_flag = 1;
}
#endif

const char *mode_names[] = {"编辑", "AUTO", "MDI", "DNC", "手轮", "手动", "回零", "空闲"};

// 轨迹规划
void emcmotTrajPlan()
{
    // 主站未进入OP或当前周期通信失败时，跳过本周期运动计算。
    if (__atomic_load_n(&op_flag, __ATOMIC_ACQUIRE) == 0)
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
        // 公共停机先锁定停车执行及坐标保护，再更新反馈。
        reset_prepare_motion();
        read_from_hal();
        // 检查跟随误差
        check_ferror_fault();
        // 判断运动模式
        set_operating_mode();
        // 主轴控制
        spindle_control_update();
        motion_csp_stop_update();
        update_spindle_position_permission();
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
        if (is_ref_ready_to_switch())
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
        if (is_auto_ready_to_switch())
        {
            rt_data->current_mode = rt_data->target_mode;
            rt_data->sys_state[SYS_IS_SWITCHING_MODE] = false;
            LOG(LOG_LEVEL_INFO, "系统模式切换为 %s", mode_names[rt_data->current_mode]);
            rerun_update_when_switching();
        }
        break;
    case MODE_EDIT:
        rt_data->current_mode = rt_data->target_mode;
        rerun_update_when_switching();
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
    /* 主轴：RtHalSpindleStatus.actual_speed，rpm（HAL 已按 speed_unit 换算）。 */
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

    /* 公共复位协调在这里保护 CSP 停车的活动坐标基准。
     * 仅进给轴的恢复沿用既有策略；CSP 重建还要求下使能确认之后主轴反馈仍然有效。 */
    if (!update_pos_flag && feed_positions_valid && reset_position_rebuild_allowed())
    {
        // 报警复位后用最新反馈，重建未补偿坐标基准。
        rebuild_axis_positions_from_feedback(spindle_position_valid);
        handwheel_resync_position();
        update_pos_flag = 1;
        if(spindle_position_valid)reset_spindle_position_rebuilt();
    }
    else if(spindle_position_valid && reset_spindle_position_required()) {
        rebuild_axis_position_from_feedback(AXIS_SP);
        reset_spindle_position_rebuilt();
    }
}

/* XYZ 位置请求是"暂存成功即返回"，HAL 的真正拒绝要下一拍快照才可见。
 * 没有这条诊断时，表现为"G 代码在等位置、轴一动不动、也没有报警"。
 * 每个轴只在错误码变化时打一条，避免刷屏。 */
static void report_axis_position_rejection(int axis, RtHalAxisSlot slot, int stage_rc)
{
    static int s_last[3] = {0, 0, 0};
    static int s_wait_code[3] = {0, 0, 0};
    static unsigned s_wait_cycles[3] = {0, 0, 0};
    HalAxisSnapshot a{};   /* 值初始化：快照失败时也能安全打印 */
    int code = stage_rc;
    int category = stage_rc ? HAL_ISSUE_POSITION : 0;

    if (axis < 0 || axis > 2) return;
    if (rt_hal_axis_snapshot(slot, &a) == HAL_OK) {
        if (!code && a.request_rejection.category == HAL_ISSUE_POSITION &&
            a.request_rejection.code) {
            code = (int)a.request_rejection.code;
            category = HAL_ISSUE_POSITION;
        }
        if (!code && a.execution_error.category == HAL_ISSUE_POSITION &&
            a.execution_error.code) {
            code = (int)a.execution_error.code;
            category = HAL_ISSUE_POSITION;
        }
    } else if (!code) {
        return;
    }
    /* 轴还没使能、或使能/模式动作正占用时的拒绝，是复位与上使能流程的正常现象
     * （HAL 明确返回 NOT_ENABLED / BUSY / MODE_MISMATCH）。这类拒绝在复位过程中会在
     * 262→8→0 之间来回跳，按“码变化”打 ERROR 就会一直刷屏。这里限频成 WARN：
     * 同一等待原因首次一条，之后每 1000 拍一条；轴已使能仍被拒才是真故障，保持 ERROR。 */
    {
        const int not_enabled = (a.drive_state != HAL_DRIVE_OPERATION_ENABLED);
        const int expected = not_enabled &&
            (code == HAL_ERROR_NOT_ENABLED || code == HAL_ERROR_BUSY ||
             code == HAL_ERROR_MODE_MISMATCH);
        if (expected) {
            if (code != s_wait_code[axis]) {
                s_wait_code[axis] = code;
                s_wait_cycles[axis] = 0;
            } else if ((++s_wait_cycles[axis] % 1000u) != 0u) {
                return;
            }
            LOG(LOG_LEVEL_WARN,
                "轴槽 %d 位置请求等待使能/模式（正常过渡，已限频）：code=%d drive=%d "
                "mode=%d/%d block=0x%x cmd=%.4f act=%.4f",
                (int)slot, code, (int)a.drive_state,
                (int)a.actual_mode, (int)a.target_mode,
                (unsigned)a.motion_block_reasons, a.command_pos, a.actual_pos);
            return;
        }
        s_wait_code[axis] = 0;
    }
    if (code == s_last[axis]) return;
    s_last[axis] = code;
    if (code) {
        LOG(LOG_LEVEL_ERR,
            "轴槽 %d 位置请求未生效：code=%d category=%d drive=%d mode=%d/%d "
            "block=0x%x cmd=%.4f act=%.4f",
            (int)slot, code, category, (int)a.drive_state,
            (int)a.actual_mode, (int)a.target_mode,
            (unsigned)a.motion_block_reasons, a.command_pos, a.actual_pos);
    }
}

static void output_to_hal()
{
    /* 以用户单位（mm）发布命令，只放进适配层的按拍缓冲；真正的写入由 PDO 线程
     * 在 commit 前完成。暂存被拒或 HAL 异步拒绝都会在这里留下记录。 */
    {
        const int rc=rt_hal_request_axis_pos(RT_HAL_AXIS_X, rt_data->axis_data[AXIS_X].motor_cmd_pos);
        report_axis_position_rejection(AXIS_X, RT_HAL_AXIS_X, rc);
    }
    {
        const int rc=rt_hal_request_axis_pos(RT_HAL_AXIS_Y, rt_data->axis_data[AXIS_Y].motor_cmd_pos);
        report_axis_position_rejection(AXIS_Y, RT_HAL_AXIS_Y, rc);
    }
    {
        const int rc=rt_hal_request_axis_pos(RT_HAL_AXIS_Z, rt_data->axis_data[AXIS_Z].motor_cmd_pos);
        report_axis_position_rejection(AXIS_Z, RT_HAL_AXIS_Z, rc);
    }
    /* 主轴目标沿用已对齐的角度坐标基准；XYZ 仍走原有的电机位置/补偿通路。
     * 停车输出优先。 */
    if(motion_csp_stop_handle_position_output() || !motion_spindle_position_active())return;
    const int rc=rt_hal_request_axis_pos(RT_HAL_AXIS_SP,
        rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur);
    if(rc!=HAL_OK) {
        motion_spindle_release_position();
        (void)alarm_spindle_request_torque(false);
        alarm_report_hal_error(rc);
    }
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
    double pitch_pos[SYS_AXES_NUM] = {0};
    double fir_ma_pos[SYS_AXES_NUM] = {0};
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        // 机床锁住或者轴拆卸，不更新机械位置
        if (!rt_data->sys_state[SYS_IS_MAC_LOCK_READY] && !rt_data->axis_data[i].status.is_disable)
        {
            pitch_pos[i] = (rt_data->axis_data[i].wcs_cmd_pos_cur - rt_data->axis_data[i].wcs_cmd_pos_pre);
        }
        // 上一周期插补位置
        rt_data->axis_data[i].wcs_cmd_pos_pre = rt_data->axis_data[i].wcs_cmd_pos_cur;
        // 指令速度平滑
        fir_ma_pos[i] = fir_ma_update(&filter_ma[i], pitch_pos[i]);
        // 更新指令位置
        rt_data->axis_data[i].mac_cmd_pos += fir_ma_pos[i];
        // 计算轴平滑速度
        rt_data->axis_data[i].fir_vel = fir_ma_pos[0] / rt_data->ncu_data.cycle;
    }
    // 计算通道指令平滑速度
    rt_data->chan_data[0].fir_vel = sqrt(pow(fir_ma_pos[0] / rt_data->ncu_data.cycle, 2) + pow(fir_ma_pos[1] / rt_data->ncu_data.cycle, 2) + pow(fir_ma_pos[2] / rt_data->ncu_data.cycle, 2));
    // if(cycle_tp->exec_state == EXEC_RUNNING){
    //     printf("FIR %f %f %f, CMD %f %f %f\n", 
    //     rt_data->axis_data[AXIS_X].mac_cmd_pos,
    //     rt_data->axis_data[AXIS_Y].mac_cmd_pos,
    //     rt_data->axis_data[AXIS_Z].mac_cmd_pos,
    //     rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur,
    //     rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur,
    //     rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur);
    // }
}

// 更新运动状态数据
static void update_status()
{
    // 更新var变量
    shm_var_update();
    /* 复位收口在本拍的 HAL 报警轮询之后统一协调。 */
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


static void rebuild_axis_position_from_feedback(int i)
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

/* 供驱动层在“重新上使能前”调用：单轴重建 + 手轮基准同步。
 * 与复位路径的 rebuild_axis_positions_from_feedback() 保持同一套语义，
 * 但不影响其他轴（其他轴可能已经在使能状态）。 */
void emcmot_rebuild_axis_position_from_feedback(int axis)
{
    if (axis < 0 || axis >= SYS_AXES_NUM) return;
    rebuild_axis_position_from_feedback(axis);
    handwheel_resync_position();
}

static void rebuild_axis_positions_from_feedback(bool spindle_valid)
{
    // 进给轴恢复不依赖主轴位置反馈是否可用。
    for(int i=0;i<SYS_AXES_NUM;++i) {
        if(i!=AXIS_SP || spindle_valid)rebuild_axis_position_from_feedback(i);
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
/* 只负责主轴位置的交接。位置规划与下发走这条运动通路；
 * 这里的许可既不是另一个控制器，也不等于硬件使能意图。 */
static bool s_position_active;
bool motion_spindle_position_active(void) { return s_position_active; }
void motion_spindle_release_position(void) { s_position_active=false; }

bool motion_spindle_take_position(double target)
{
    HalAxisSnapshot a;
    if(!isfinite(target) || rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&a)!=HAL_OK ||
       (a.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION))!=
       (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION) ||
       a.drive_state!=HAL_DRIVE_OPERATION_ENABLED || a.actual_mode!=HAL_AXIS_MODE_CSP ||
       !(a.command_valid&HAL_COMMAND_POSITION) || !isfinite(a.command_pos) ||
       !isfinite(a.actual_pos) || fabs(a.command_pos-target)>1e-6 ||
       !alarm_spindle_request_torque(true)) {
        LOG(LOG_LEVEL_ERR, "主轴位置控制接管失败：target=%.6f", target);
        return false;
    }
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=target;
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_pre=target;
    s_position_active=true;
    return true;
}
static void update_spindle_position_permission(void)
{
    if(!s_position_active)return;
    HalAxisSnapshot a;
    if(!reset_spindle_business_allowed() || rt_data->sys_state[SYS_ESTOP_INPUT] ||
       rt_data->sys_state[SYS_IS_ESTOP_ACTIVE]) {
        s_position_active=false;return;
    }
    int rc=rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&a);
    if(rc==HAL_OK && a.request_rejection.category==HAL_ISSUE_POSITION)
        rc=a.request_rejection.code;
    if(rc==HAL_OK && a.execution_error.category==HAL_ISSUE_POSITION)
        rc=a.execution_error.code;
    if(rc!=HAL_OK ||
       (a.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION))!=
       (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_POSITION) ||
       a.drive_state!=HAL_DRIVE_OPERATION_ENABLED || a.actual_mode!=HAL_AXIS_MODE_CSP) {
        s_position_active=false;
        (void)alarm_spindle_request_torque(false);
        alarm_report_hal_error(rc!=HAL_OK?rc:HAL_ERROR_STATE);
    }
}
/* M19/M29 准备 CSV 时的“等待类”拒绝会每拍复发（例如主轴还在减速），
 * 这里限频：同一原因首次打一条 WARN，之后每 1000 拍（约 1 s）打一条；
 * 原因变化或准备成功时重新计数。真正的错误仍走 LOG_LEVEL_ERR 立即打印。 */
static int s_prep_wait_reason;
static unsigned s_prep_wait_cycles;
static void prep_wait_log(int reason, const char* fmt, ...)
{
    if(reason!=s_prep_wait_reason) {
        s_prep_wait_reason=reason;
        s_prep_wait_cycles=0;
    } else if((++s_prep_wait_cycles%1000u)!=0u) {
        return;
    }
    char text[192];
    va_list ap;
    va_start(ap,fmt);
    vsnprintf(text,sizeof(text),fmt,ap);
    va_end(ap);
    LOG(LOG_LEVEL_WARN,"%s",text);
}
int motion_spindle_prepare_velocity(void)
{
    HalAxisSnapshot a;
    if(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&a)!=HAL_OK ||
       (a.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_SPEED))!=
       (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE|HAL_FEEDBACK_SPEED) ||
       a.actual_mode!=HAL_AXIS_MODE_CSP || !isfinite(a.actual_speed_rpm)) {
        LOG(LOG_LEVEL_ERR,"M19前准备CSV被拒：快照/反馈无效 rc=%d",(int)a.actual_mode);
        return -1;
    }
    if(motion_csp_stop_pending()) {
        prep_wait_log(1,"M19前准备CSV被拒：CSP停车在途");
        return 0;
    }
    if(cycle_tp && cycle_tp->exec_state!=EXEC_IDLE &&
       (cycle_tp->exec_cmd==DO_TAP_CMD || cycle_tp->exec_cmd==DO_MOVE_CMD ||
        cycle_tp->exec_cmd==DO_CHOP_CMD || cycle_tp->exec_cmd==DO_SKIP_CMD)) {
        prep_wait_log(2,"M19前准备CSV被拒：运动未空闲 cmd=%d state=%d",
            (int)cycle_tp->exec_cmd,(int)cycle_tp->exec_state);
        return 0;
    }
    if(fabs(a.actual_speed_rpm)>1.0) {
        prep_wait_log(3,"M19前准备CSV被拒：主轴未停稳 rpm=%.3f",a.actual_speed_rpm);
        return 0;
    }
    if(a.drive_state==HAL_DRIVE_OPERATION_ENABLED) {
        if(!(a.feedback_valid&HAL_FEEDBACK_POSITION) || !(a.command_valid&HAL_COMMAND_POSITION) ||
           !isfinite(a.command_pos) || !isfinite(a.actual_pos)) {
            LOG(LOG_LEVEL_ERR,"M19前准备CSV被拒：位置反馈/指令无效");
            return -1;
        }
        if(fabs(a.command_pos-a.actual_pos)>10.0) {
            prep_wait_log(4,"M19前准备CSV被拒：位置误差过大 cmd=%.4f act=%.4f err=%.4f",
                a.command_pos,a.actual_pos,fabs(a.command_pos-a.actual_pos));
            return 0;
        }
    } else if(a.drive_state!=HAL_DRIVE_READY && a.drive_state!=HAL_DRIVE_SWITCH_ON_DISABLED) {
        LOG(LOG_LEVEL_ERR,"M19前准备CSV被拒：驱动状态=%d",(int)a.drive_state);
        return -1;
    }
    s_prep_wait_reason=0;
    s_position_active=false;
    return 1;
}

#ifdef UNIT_TEST
void emcmot_test_spindle_position_period(void)
{
    update_spindle_position_permission();
    output_to_hal();
}
#endif
