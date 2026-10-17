#include "spl_control.h"
#include "spl_orient.h"
#include "rt_hal_adapter.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <hmisysapi.h>
#include <math.h>
#include <parapi.h>
#include <regapi.h>
#include <string.h>

#define SPINDLE_CONTROL_PERIOD_SECONDS (0.001)
#define SPINDLE_SPEED_EPSILON          (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON    (1.0)
#define SPINDLE_ACTUAL_SPEED_EPSILON   (1.0)
#define SPINDLE_CSP_POSITION_EPSILON   (10.0)
#define SPINDLE_M29_STABLE_CYCLES      (10U)
#define SPINDLE_SPEED_ARRIVAL_TOL      (0.05)
#define SPINDLE_SPEED_ARRIVAL_MIN_TOL  (5.0)
#define SPINDLE_ALARM_MISSING_SPEED    (20)
#define SPINDLE_ALARM_ORIENT_TIMEOUT   (2100)
#define ALARM_RESULT_ALREADY_EXISTS    (-8)
#define SPINDLE_ORIENT_TIMEOUT_CYCLES  (10000U)
#define SPINDLE_M29_MODE_TIMEOUT_CYCLES (10000U)

spindle_context_t spl_context;

/*
 * 主轴控制的职责边界：
 * 1. 本文件只维护 CNC/PLC 的主轴逻辑状态和速度规划；
 * 2. 速度目标通过 hal_driver_t->target_vel 交给驱动器 HAL；
 * 3. 驱动器实际使能、模式和状态字由 driver.c 状态机维护；
 * 4. M3/M4 的 FIN 只能在真实驱动器到速后产生，不能只依据软件规划器；
 * 5. M19 定向由驱动器收到 0x040F 后自行减速，定向完成依据状态字 bit10。
 */

/* ==========================================================================
 * 1. 内部函数声明
 * ========================================================================== */

static bool spindle_stop_is_required(const spindle_context_t *context);
static void spindle_update_command_event(void);
static void spindle_process_event(spindle_context_t *context);
static void spindle_handle_stop(spindle_context_t *context);
static void spindle_handle_rotation(spindle_context_t *context,
                                    spindle_state_t state);
static bool spindle_validate_start_request(const spindle_context_t *context);
static bool spindle_report_missing_speed_alarm(const char *message);
static double spindle_get_selected_speed(const spindle_context_t *context);

/* ---- HAL 访问器（方案 B）：按需现查 HAL 快照，不留持久镜像 ----
 * HAL 没有的（target_vel / target_pos / req_mode）才是 RT 本地量（s_spl_*）；
 * 状态机内部的（DS402 状态、控制字、模式切换进度）由 HAL 独占，上层只看结果。 */
static int spl_drv_enabled(void) {
    HalCAxisStatus st;
    return rt_hal_axis_status(RT_HAL_AXIS_SP, &st) == HAL_OK ? st.enabled : 0;
}
static int spl_drv_mode(void) {
    HalCSpindleStatus ss;
    return rt_hal_spindle_status(RT_HAL_AXIS_SP, &ss) == HAL_OK ? ss.mode : -1;
}
static double spl_drv_actual_vel(void) {
    double v = 0.0;
    return rt_hal_spindle_speed(RT_HAL_AXIS_SP, &v) == HAL_OK ? v : 0.0;
}
static double spl_drv_actual_pos(void) {
    HalCSpindleStatus ss;
    return rt_hal_spindle_status(RT_HAL_AXIS_SP, &ss) == HAL_OK ? ss.position_deg : 0.0;
}
static uint16_t spl_drv_status_word(void) {
    HalCAxisStatus st;
    return rt_hal_axis_status(RT_HAL_AXIS_SP, &st) == HAL_OK ? st.raw_status : 0;
}
static int spl_drv_orient_state(void) {
    int32_t v = HAL_ORIENT_NONE;
    return rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &v) == HAL_OK ? (int)v : HAL_ORIENT_NONE;
}
/* 状态字 0x21 = Switch On Disabled：带电但没使能。 */
static int spl_drv_switch_on_disabled(void) {
    return (spl_drv_status_word() & 0x006FU) == 0x0021U;
}
/* HAL 的释放是把控制字交回状态机并重新使能（不是退回 0x21），
 * 故"释放完成"看定向状态是否退回 NONE。 */
static int spl_drv_orient_inactive(void) {
    return spl_drv_orient_state() == HAL_ORIENT_NONE;
}

/* RT 本地量：HAL 没有对应的查询接口。 */
static double s_spl_target_vel;
static double s_spl_target_pos;
static int    s_spl_req_mode = HAL_SPINDLE_CSV;

/* M29 交回 M3/M5/复位后继续观察 PLC 清理过程 */
static unsigned s_m29_trace_tail;
static bool s_m29_trace_start;

void spindle_m29_plc_trace_tick(void)
{
    static const struct { RegType type; int idx; int bit; } channels[] = {
        {REG_F,76,3}, {REG_G,61,0}, {REG_R,560,0}, {REG_R,560,1},
        {REG_R,560,2}, {REG_R,500,3}, {REG_G,70,5}, {REG_G,70,4},
        {REG_R,1,0}, {REG_G,4,3}, {REG_F,7,0}, {REG_R,643,5},
        {REG_K,1,3}, {REG_R,284,0}, {REG_R,284,1}, {REG_R,1000,3},
        {REG_R,108,4}, {REG_A,0,5}, {REG_R,212,4}, {REG_F,1,1},
        {REG_F,45,0}, {REG_F,0,6}, {REG_R,500,5}, {REG_R,500,7},
        {REG_R,999,5}, {REG_R,282,0}, {REG_R,282,1}, {REG_R,640,4},
        {REG_X,0,3}
    };
    static int previous[sizeof(channels) / sizeof(channels[0])];
    static bool initialized;
    static int previous_phase;
    static bool previous_active;
    static uint32_t sample;
    int values[sizeof(channels) / sizeof(channels[0])];
    ++sample;
    if (!spl_context.m29_active && !s_m29_trace_tail) return;
    for (unsigned i = 0; i < sizeof(channels) / sizeof(channels[0]); ++i) {
        bool value = false;
        values[i] = getRegBit(channels[i].type, channels[i].idx,
                             channels[i].bit, &value) == 0 ? (int)value : -1;
    }
    if (spl_context.m29_active) s_m29_trace_tail = 2000;
    if (s_m29_trace_tail && (s_m29_trace_start || !initialized ||
        previous_phase != (int)spl_context.m29_phase ||
        previous_active != spl_context.m29_active ||
        memcmp(previous, values, sizeof(values)))) {
        LOG(LOG_LEVEL_INFO,
            "M29 PLC变化 sample=%u phase=%d active=%d "
            "F76.3=%d G61.0=%d R560.0=%d R560.1=%d R560.2=%d R500.3=%d "
            "G70.5=%d G70.4=%d R1.0=%d G4.3=%d F7.0=%d R643.5=%d "
            "K1.3=%d R284.0=%d R284.1=%d R1000.3=%d R108.4=%d A0.5=%d "
            "R212.4=%d F1.1=%d F45.0=%d F0.6=%d R500.5=%d R500.7=%d "
            "R999.5=%d R282.0=%d R282.1=%d R640.4=%d X0.3=%d",
            (unsigned)sample, spl_context.m29_phase, spl_context.m29_active,
            values[0], values[1], values[2], values[3], values[4], values[5],
            values[6], values[7], values[8], values[9], values[10], values[11],
            values[12], values[13], values[14], values[15], values[16], values[17],
            values[18], values[19], values[20], values[21], values[22], values[23],
            values[24], values[25], values[26], values[27], values[28]);
    }
    /* M29 入口补打一次持久驱动故障（首次报警可能早于本次采集） */
    static int last_axis[RT_HAL_AXIS_COUNT][6];
    static bool axes_initialized;
    for (int slot = 0; slot < RT_HAL_AXIS_COUNT; ++slot) {
        HalCAxisStatus status = {0};
        const int rc = rt_hal_axis_status((RtHalAxisSlot)slot, &status);
        const int current[6] = {
            rt_data->axis_data[slot].status.is_error,
            rt_data->axis_data[slot].status.is_disable,
            rc, rc == HAL_OK ? status.raw_status : -1,
            rc == HAL_OK ? status.error_code_valid : -1,
            rc == HAL_OK ? status.error_code : -1
        };
        if (s_m29_trace_start || !axes_initialized ||
            memcmp(last_axis[slot], current, sizeof(current))) {
            LOG(LOG_LEVEL_INFO,
                "M29故障源 sample=%u slot=%d is_error=%d disabled=%d rc=%d "
                "status_word=%d error_valid=%d error_code=%d F0.6=%d F45.0=%d",
                (unsigned)sample, slot, current[0], current[1], current[2],
                current[3], current[4], current[5], values[21], values[20]);
            memcpy(last_axis[slot], current, sizeof(current));
        }
    }
    axes_initialized = true;
    if (s_m29_trace_tail) --s_m29_trace_tail;
    s_m29_trace_start = false;
    memcpy(previous, values, sizeof(values));
    previous_phase = spl_context.m29_phase;
    previous_active = spl_context.m29_active;
    initialized = true;
}

/* ---- 模式切换：HAL 内部走完"减速 -> 去使能 -> 写 6060 -> 等 6061 -> 重新使能"，
 * 上层只留一个完成闩锁，表达"请求下去了没有、跟上没有"。 ---- */
#define SPL_MS_IDLE      0
#define SPL_MS_SWITCHING 1
#define SPL_MS_DONE      2
#define SPL_MS_ERROR     3
static int     s_spl_ms = SPL_MS_IDLE;
static int32_t s_spl_ms_rc = HAL_OK;

static void spl_ms_poll(void) {
    if (s_spl_ms == SPL_MS_SWITCHING && spl_drv_mode() == (int)s_spl_req_mode)
        s_spl_ms = SPL_MS_DONE;
}
/* 每次调用顺带推进闩锁，调用顺序就不影响判断。 */
static int spl_ms_switching(void) { spl_ms_poll(); return s_spl_ms == SPL_MS_SWITCHING; }
static int spl_ms_done(void)      { spl_ms_poll(); return s_spl_ms == SPL_MS_DONE; }
static int32_t spl_ms_result(void) { return s_spl_ms_rc; }
/* 取消遗留切换（复位/急停）。HAL 侧没有可取消的东西，清闩锁即可。 */
static void spl_ms_reset(void) { s_spl_ms = SPL_MS_IDLE; s_spl_ms_rc = HAL_OK; }

static int spl_ms_request(HalSpindleMode mode) {
    /* 反馈可能已匹配，但 HAL 里还排着反向请求，所以每次都往下发 */
    s_spl_ms_rc = rt_hal_request_spindle_mode(RT_HAL_AXIS_SP, mode);
    if (s_spl_ms_rc == HAL_OK) {
        s_spl_req_mode = mode;
        s_spl_ms = spl_drv_mode() == (int)mode ? SPL_MS_DONE : SPL_MS_SWITCHING;
    } else {
        s_spl_ms = SPL_MS_ERROR;
    }
    return s_spl_ms_rc;
}

/* 使能声明：这里只设"意愿位"，不直接写 HAL。
 * 主轴若自己写 HAL 会与 hal_axis_intent_poll() 抢同一字段，后者每拍无条件 enable(1)、
 * 后写赢，会导致主轴的去使能全部失效（M5 停稳仍带力矩、切换失败也不下使能，且会取消在飞复位）。
 * 统一由 poll 合并，急停与轴屏蔽永远优先。 */
static void spl_drv_enable(void)  { rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = true; }
static void spl_drv_disable(void) { rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = false; }

static double spindle_calculate_target_velocity(const spindle_context_t *context);
static void spindle_update_driver_control(void);
static void spindle_update_velocity(void);
static void spindle_update_planner_state(spindle_context_t *context);
static bool spindle_is_velocity_arrived(void);
static bool spindle_stop_is_complete(void);
static bool spindle_velocity_feedback_blocked(void);
static void spindle_finish_stop_if_ready(void);
static void spindle_disable_driver(void);
static void spindle_disable_driver_before_operation(void);
static bool spindle_is_rotating(void);
static void spindle_publish_status(void);
static void spindle_report_mcode_complete(void);
static void spindle_update_orient_state(void);
static void spindle_sync_planner_to_actual(void);
static void spindle_publish_rigid_state(void);
static void spindle_apply_deferred_start(void);
static void spindle_update_m29(void);
static void spindle_update_m29_feedback(void);
static bool spindle_reset_or_estop_active(void);
static void spindle_log_m29_csp_trace(void);
static void spindle_update_m29_no_orient(void);
static bool spindle_m29_no_orient_preparing(void) {
    return spl_context.m29_phase >= SPINDLE_M29_NO_ORIENT_WAIT_CSV &&
           spl_context.m29_phase <= SPINDLE_M29_NO_ORIENT_ALIGNING;
}

/* ==========================================================================
 * 2. 外部函数定义
 * ========================================================================== */

/**
 * @brief 执行一个主轴控制周期。
 *
 * 控制周期按固定顺序完成安全检查、事件处理、定向更新、驱动器控制
 * 和状态发布。主轴控制相关的驱动器细节由本模块统一管理。
 */
void spindle_control_update(void)
{
    bool release_orient_event;

    if (spindle_m29_no_orient_preparing()) {
        spindle_update_m29_no_orient();
        spindle_publish_status();
        spindle_publish_rigid_state();
        return;
    }

    release_orient_event =
        (spl_context.spl_event == SPINDLE_EVENT_RELEASE_ORIENT);

    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        (spl_drv_status_word() & 0x0008U) != 0U)
    {
        // printg("85\n");
        /* 急停解除后必须等待新的 M3/M4，不能恢复急停前的旧速度。 */
        spl_context.reset_requires_start = true;
        spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        spl_context.start_request_pending = false;
        spl_context.orient_request_pending = false;
        spl_context.orient_release_pending = false;
        spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
        spl_context.orient_elapsed_cycles = 0U;
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
        spl_context.m29_active = false;
        spl_context.m29_phase = SPINDLE_M29_IDLE;
        spl_context.csv_switch_pending = false;
        spl_context.deferred_start_pending = false;
        /* 安全事件取消遗留模式切换，免得闸门一直卡在"正在切换"。 */
        spl_ms_reset();
        spl_orient_abort();
    }

    if ((rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
        (nc_reset != NULL && nc_reset->is_resetting))
    {
        spl_context.reset_requires_start = true;
        spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        // printg("103\n");
        spl_context.start_request_pending = false;
        spl_context.deferred_start_pending = false;

        /*
         * 复位必须能够中止正在进行或已经完成的定向。PLC 的 G70.6
         * 下降沿通常会同时送入 RELEASE_ORIENT，但复位事件优先级更高
         * 时不能让下面的 STOP 覆盖该释放请求；没有下降沿时也要有
         * 兜底释放，避免状态永久停在 SPINDLE_ORIENT_DONE。
         */
        if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
            spl_context.spl_event == SPINDLE_EVENT_ORIENT ||
            spl_context.orient_request_pending)
        {
            // printg("115\n");
            spl_context.spl_event = SPINDLE_EVENT_RELEASE_ORIENT;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            release_orient_event = true;
        }
        /* 复位结束当前 M19 握手，新的 M3/M4 可以重新取得主轴控制权。 */
        spl_context.orient_hold_stop = false;
        spl_context.m19_active = false;
        spl_context.m29_active = false;
        spl_context.m29_phase = SPINDLE_M29_IDLE;
        spl_context.csv_switch_pending = false;
        /* HMI RESET 同样取消未完成的模式切换。 */
        spl_ms_reset();
    }

    if (spindle_stop_is_required(&spl_context) && !release_orient_event)
    {
        // printg("126\n");
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else if (!release_orient_event)
    {
        /* 复位后的旧 CW/CCW 事件由下面的专用过滤逻辑处理。 */
        spindle_update_command_event();
    }

    if (spl_context.reset_requires_start &&
        !spl_context.start_request_pending &&
        (spl_context.spl_event == SPINDLE_EVENT_CW ||
         spl_context.spl_event == SPINDLE_EVENT_CCW))
    {
        // printg("146\n");
        /* 丢弃复位前遗留的 PLC/方向边沿，直到新的 M3/M4 到来。 */
        spl_context.spl_event = SPINDLE_EVENT_NONE;
    }

    /* 事件处理只负责改变逻辑/驱动器目标；定向完成和速度规划在后续阶段更新。 */
    spindle_process_event(&spl_context);
    /* 直接读取 driver.c 的定向状态，并映射到 CNC/PLC 的 orient_phase。 */
    spindle_update_orient_state();
    /* M29 consumes the DONE phase on the following cycle, allowing the
     * NC/PLC handshake to observe orient_phase == DONE first. */
    spindle_update_m29();
    spindle_update_m29_feedback();
    /* 根据安全状态、定向状态和速度状态决定使能、模式及 60FF 目标值。 */
    spindle_update_driver_control();
    /* 将软件速度目标发布到 HMI 变量，供显示和诊断使用。 */
    spindle_publish_status();
    /* 刚性攻丝方式的状态（F76.3 的来源）每拍跟着 M29 阶段走。 */
    spindle_publish_rigid_state();
}

/* 「刚性攻丝方式中」的状态来源：M29 接管主轴起置起，M03/M04/M05、复位/急停或 M29
 * 出错时落下（这些路径都会把 m29_phase 清回 IDLE/ERROR）。
 * 这里只维护状态；F76.3 由 plc_f_signal_handler 发布（SYS_IS_RIGID）。
 * 必须跟随 M29 阶段，否则 SYS_IS_RIGID 无人写、F76.3 恒 0。 */
static void spindle_publish_rigid_state(void)
{
    rt_data->sys_state[SYS_IS_RIGID] =
        (spl_context.m29_phase != SPINDLE_M29_IDLE &&
         spl_context.m29_phase != SPINDLE_M29_ERROR);
}

/**
 * @brief 初始化主轴上下文中的面板启动速度。
 *
 * 其余成员由全局静态存储区保证初值为零；运行过程中，CNC/PLC 事件
 * 再分别填充 command_speed、command_state 和各类握手标志。
 */
void spl_context_init(spindle_context_t* spl_ctx)
{
    spl_ctx->mcp_target_speed = rt_data->axis_data[AXIS_SP].spl_default_speed;
}

void spindle_request_m29(void)
{
    s_m29_trace_tail = 2000;
    s_m29_trace_start = true;
    spindle_clear_velocity_arrival();   /* 新 M29：清旧到速完成 */
    /* M29 本身是复位后的新主轴操作请求，允许重新取得驱动器使能。 */
    spl_context.reset_requires_start = false;
    spl_context.start_request_pending = true;
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_ORIENTING;
    spl_context.m19_active = false;
    spl_context.orient_hold_stop = true;
    spindle_clear_velocity_arrival();   /* 清旧到速反馈 + F45.3 */
    /* M29 是新的主轴操作请求，取消排队中的 M3/M4。 */
    spl_context.deferred_start_pending = false;
    spl_context.spl_event = SPINDLE_EVENT_ORIENT;
    spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
    spl_context.command_speed = 0.0;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.start_request_pending = false;
}

/* 不能用读取失败时回零的便捷访问器判断停稳。 */
static bool spindle_m29_read_feedback(HalCAxisStatus *axis, HalCSpindleStatus *spindle)
{
    const int axis_rc = rt_hal_axis_status(RT_HAL_AXIS_SP, axis);
    const int spindle_rc = rt_hal_spindle_status(RT_HAL_AXIS_SP, spindle);
    if (axis_rc != HAL_OK || spindle_rc != HAL_OK) {
        LOG(LOG_LEVEL_ERR, "M29无预定向反馈不可读：axis_rc=%d spindle_rc=%d",
            axis_rc, spindle_rc);
        return false;
    }
    if (!axis->speed_valid || !axis->position_valid ||
        !isfinite(axis->actual_speed_rpm) || !isfinite(axis->actual_pos) ||
        !isfinite(axis->command_pos) || !isfinite(spindle->position_deg)) {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向反馈无效：speed_valid=%d position_valid=%d "
            "rpm=%.3f actual_pos=%.3f command_pos=%.3f spindle_pos=%.3f",
            axis->speed_valid, axis->position_valid, axis->actual_speed_rpm,
            axis->actual_pos, axis->command_pos, spindle->position_deg);
        return false;
    }
    return true;
}

bool spindle_request_m29_no_orient(void)
{
    HalCAxisStatus axis;
    HalCSpindleStatus spindle;
    int32_t orient;
    const bool stop_required = spindle_stop_is_required(&spl_context);
    const bool mode_switching = spl_ms_switching();
    if (stop_required || rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->axis_data[AXIS_SP].status.is_disable ||
        rt_data->axis_data[AXIS_SP].status.is_error ||
        spl_context.m29_active || mode_switching || spl_context.csv_switch_pending ||
        spl_context.orient_phase != SPINDLE_ORIENT_IDLE || spl_context.orient_request_pending) {
        LOG(LOG_LEVEL_ERR,
            "M29无预定向入口拒绝：stop=%d estop=%d disable=%d error=%d "
            "active=%d switching=%d csv_pending=%d orient_phase=%d orient_pending=%d",
            stop_required, rt_data->sys_state[SYS_ESTOP_INPUT],
            rt_data->axis_data[AXIS_SP].status.is_disable,
            rt_data->axis_data[AXIS_SP].status.is_error, spl_context.m29_active,
            mode_switching, spl_context.csv_switch_pending,
            spl_context.orient_phase, spl_context.orient_request_pending);
        return false;
    }
    const int orient_rc = rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient);
    if (orient_rc != HAL_OK || orient != HAL_ORIENT_NONE) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：HAL定向状态 rc=%d state=%d",
            orient_rc, orient_rc == HAL_OK ? orient : -1);
        return false;
    }
    if (!spindle_m29_read_feedback(&axis, &spindle))
        return false;
    if ((axis.raw_status & 0x0008U) || (axis.error_code_valid && axis.error_code)) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：主轴故障 status=0x%04X error=0x%04X",
            (unsigned)axis.raw_status, (unsigned)axis.error_code);
        return false;
    }
    if (!isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) ||
        rt_data->axis_data[AXIS_SP].max_feed_acc <= 0.0) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：主轴进给加速度无效 %.3f",
            rt_data->axis_data[AXIS_SP].max_feed_acc);
        return false;
    }
    if (spindle.mode != HAL_SPINDLE_CSV && spindle.mode != HAL_SPINDLE_CSP &&
        fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：未知模式%d下主轴仍在转 %.1f rpm",
            spindle.mode, axis.actual_speed_rpm);
        return false;
    }
    /* CSP 运动由插补所有，不能把正在走的目标截成当前位置。 */
    if (spindle.mode == HAL_SPINDLE_CSP &&
        (fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON ||
         fabs(axis.command_pos - axis.actual_pos) > SPINDLE_CSP_POSITION_EPSILON)) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：CSP未停稳或位置未跟随");
        return false;
    }
    if (spindle.mode == HAL_SPINDLE_CSP &&
        rt_hal_request_axis_pos(RT_HAL_AXIS_SP, axis.actual_pos) != HAL_OK) {
        LOG(LOG_LEVEL_ERR, "M29无预定向入口拒绝：CSP位置保持请求失败");
        return false;
    }

    s_m29_trace_tail = 2000;
    s_m29_trace_start = true;
    spl_context.m29_active = true;
    spl_context.m29_phase =
        spindle.mode == HAL_SPINDLE_CSV || spindle.mode == HAL_SPINDLE_CSP ?
        SPINDLE_M29_NO_ORIENT_STOPPING : SPINDLE_M29_NO_ORIENT_WAIT_CSV;
    spl_context.m29_stop_stable_cycles = 0;
    spl_context.orient_elapsed_cycles = 0;
    spl_context.reset_requires_start = false;
    spl_context.m19_active = false;
    spl_context.orient_hold_stop = true;
    spl_context.orient_release_pending = false;
    spl_context.start_request_pending = false;
    spl_context.deferred_start_pending = false;
    spl_context.spl_event = SPINDLE_EVENT_NONE;
    spl_context.command_state = spl_context.spl_state = SPINDLE_STOP;
    spl_context.command_speed = 0.0;
    spl_context.vel_planner.current_vel = axis.actual_speed_rpm;
    spl_context.vel_planner.target_vel = 0.0;
    spl_context.vel_planner.enable = false;
    s_spl_target_pos = axis.actual_pos;
    spindle_clear_velocity_arrival();
    spl_ms_reset();
    LOG(LOG_LEVEL_INFO, "M29无预定向：初始模式=%d，准备停稳后切CSP", spindle.mode);
    return true;
}

static void spindle_m29_no_orient_fail(void)
{
    spl_context.m29_phase = SPINDLE_M29_ERROR;
    spl_context.m29_active = false;
    spl_context.reset_requires_start = true;
    spl_context.start_request_pending = false;
    spl_context.deferred_start_pending = false;
    spl_context.command_state = spl_context.spl_state = SPINDLE_STOP;
    spl_context.command_speed = 0.0;
    spl_context.vel_planner.enable = false;
    spl_context.vel_planner.target_vel = 0.0;
    s_spl_target_vel = 0.0;
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
    spindle_clear_velocity_arrival();
    spl_ms_reset();
    spl_drv_disable();
    setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
    LOG(LOG_LEVEL_ERR, "M29无预定向准备中止：停止/联锁、反馈或请求异常、等待超时");
}

static void spindle_update_m29_no_orient(void)
{
    HalCAxisStatus axis;
    HalCSpindleStatus spindle;
    int32_t orient;
    if (spindle_stop_is_required(&spl_context) || rt_data->sys_state[SYS_ESTOP_INPUT] ||
        rt_data->axis_data[AXIS_SP].status.is_disable ||
        rt_data->axis_data[AXIS_SP].status.is_error ||
        !spindle_m29_read_feedback(&axis, &spindle) ||
        !isfinite(rt_data->axis_data[AXIS_SP].max_feed_acc) ||
        rt_data->axis_data[AXIS_SP].max_feed_acc <= 0.0 ||
        (axis.raw_status & 0x0008U) || (axis.error_code_valid && axis.error_code) ||
        rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &orient) != HAL_OK || orient != HAL_ORIENT_NONE ||
        spl_context.spl_event == SPINDLE_EVENT_ORIENT ||
        spl_context.spl_event == SPINDLE_EVENT_RELEASE_ORIENT ||
        ++spl_context.orient_elapsed_cycles >= SPINDLE_M29_MODE_TIMEOUT_CYCLES) {
        spindle_m29_no_orient_fail();
        return;
    }
    /* PLC 的方向边沿不能接管准备流程；不产生 M19 完成或 F45.7。 */
    spl_context.spl_event = SPINDLE_EVENT_NONE;
    spl_drv_enable();
    if (spl_context.m29_phase == SPINDLE_M29_NO_ORIENT_WAIT_CSV) {
        if (s_spl_ms == SPL_MS_IDLE && spl_ms_request(HAL_SPINDLE_CSV) != HAL_OK) {
            spindle_m29_no_orient_fail();
            return;
        }
        if (!spl_ms_done() || spindle.mode != HAL_SPINDLE_CSV) return;
        spl_context.m29_phase = SPINDLE_M29_NO_ORIENT_STOPPING;
    }
    if (spl_context.m29_phase == SPINDLE_M29_NO_ORIENT_STOPPING) {
        if (spindle.mode == HAL_SPINDLE_CSV) {
            spl_context.vel_planner.max_acc = rt_data->axis_data[AXIS_SP].max_feed_acc;
            spl_vel_planner_update(&spl_context.vel_planner, SPINDLE_CONTROL_PERIOD_SECONDS);
            s_spl_target_vel = spl_context.vel_planner.current_vel;
            if (rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, fabs(s_spl_target_vel),
                    s_spl_target_vel > 0 ? 1 : s_spl_target_vel < 0 ? -1 : 0) != HAL_OK) {
                spindle_m29_no_orient_fail();
                return;
            }
        }
        if (fabs(axis.actual_speed_rpm) <= SPINDLE_ACTUAL_ZERO_EPSILON &&
            (spindle.mode == HAL_SPINDLE_CSP ||
             fabs(spl_context.vel_planner.current_vel) <= SPINDLE_SPEED_EPSILON))
            ++spl_context.m29_stop_stable_cycles;
        else spl_context.m29_stop_stable_cycles = 0;
        if (spl_context.m29_stop_stable_cycles >= SPINDLE_M29_STABLE_CYCLES) {
            if (spl_ms_request(HAL_SPINDLE_CSP) != HAL_OK) {
                spindle_m29_no_orient_fail();
                return;
            }
            spl_context.m29_phase = SPINDLE_M29_NO_ORIENT_SWITCHING;
            spl_context.orient_elapsed_cycles = 0;
        }
        return;
    }
    if (spl_context.m29_phase == SPINDLE_M29_NO_ORIENT_SWITCHING) {
        if (!spl_ms_done() || spindle.mode != HAL_SPINDLE_CSP || !axis.enabled ||
            (axis.raw_status & 0x006fU) != 0x0027U ||
            fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON) return;
        /* HAL 切 CSP 时预置实际位置。这里只平移坐标，不命令转回机械零度；
         * 下一拍确认设零反馈后才写零位置，避免设零被拒时发出旧坐标的零目标。 */
        if (rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0) != HAL_OK) {
            spindle_m29_no_orient_fail();
            return;
        }
        spl_context.m29_phase = SPINDLE_M29_NO_ORIENT_ALIGNING;
        spl_context.orient_elapsed_cycles = 0;
        spl_context.m29_stop_stable_cycles = 0;
        return;
    }
    const int setpos_result = rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP);
    if (setpos_result == HAL_ERROR_BUSY) return;
    if (setpos_result != HAL_OK) {
        spindle_m29_no_orient_fail();
        return;
    }
    if (spindle.mode != HAL_SPINDLE_CSP || !axis.enabled ||
        (axis.raw_status & 0x006fU) != 0x0027U ||
        fabs(axis.actual_speed_rpm) > SPINDLE_ACTUAL_ZERO_EPSILON ||
        fabs(axis.actual_pos) > SPINDLE_CSP_POSITION_EPSILON ||
        fabs(axis.command_pos) > SPINDLE_CSP_POSITION_EPSILON) {
        spl_context.m29_stop_stable_cycles = 0;
        return;
    }
    if (++spl_context.m29_stop_stable_cycles < SPINDLE_M29_STABLE_CYCLES) return;
    if (rt_hal_request_axis_pos(RT_HAL_AXIS_SP, 0.0) != HAL_OK) {
        spindle_m29_no_orient_fail();
        return;
    }
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur = 0.0;
    s_spl_target_pos = 0.0;
    spl_context.m29_phase = SPINDLE_M29_READY;
    LOG(LOG_LEVEL_INFO, "M29无预定向：CSP使能与起点对齐已确认，等待PLC握手");
}

bool spindle_m29_is_ready(void)
{
    /* 只看 READY 枚举不够（那只是本地状态机到位）：必须再核对实际模式切换结果、
     * 实际 CSP、使能状态与 PDO 状态字。 */
    if (spindle_stop_is_required(&spl_context))
        return false;
    if (!spl_context.m29_active || spl_context.reset_requires_start)
        return false;
    if (spl_context.m29_phase != SPINDLE_M29_READY)
        return false;
    if (!spl_ms_done())
        return false;
    if (spl_drv_mode() != HAL_SPINDLE_CSP)
        return false;
    if (!spl_drv_enabled())
        return false;
    return (spl_drv_status_word() & 0x006FU) == 0x0027U;
}

/* M29 卡住诊断用的只读快照：只查询，不改变任何状态（供 auto_mst 打印一行）。 */
void spindle_m29_diag_info(spindle_m29_diag_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->m29_phase = (int)spl_context.m29_phase;
    out->m29_active = spl_context.m29_active ? 1 : 0;
    out->orient_phase = (int)spl_context.orient_phase;
    out->orient_state = spl_drv_orient_state();
    out->ms_state = s_spl_ms;
    out->ms_result = (int)s_spl_ms_rc;
    out->ms_req_mode = (int)s_spl_req_mode;
    out->act_mode = spl_drv_mode();
    out->enabled = spl_drv_enabled();
    out->status_word = spl_drv_status_word();
    out->ready = spindle_m29_is_ready();
    out->planned_rpm = spl_context.vel_planner.current_vel;
    out->target_rpm = s_spl_target_vel;
    out->actual_rpm = spl_drv_actual_vel();
    out->target_deg = s_spl_target_pos;
    out->actual_deg = spl_drv_actual_pos();
}

void spindle_complete_m29(void)
{
    if (spl_context.m29_phase != SPINDLE_M29_READY)
    {
        return;
    }

    /* PLC has returned G4.3.  Keep CSP/Operation Enabled and the position
     * hold after the handshake; a subsequent M3/M4 explicitly releases this
     * READY state and requests the safe CSP -> CSV transition. */
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_READY;
    spl_context.orient_hold_stop = true;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.command_speed = 0.0;
    spindle_clear_velocity_arrival();   /* 清旧到速反馈 + F45.3 */
}

/* ==========================================================================
 * 3. 内部函数定义
 * ========================================================================== */
/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    /*
     * 此函数不直接启动驱动器，只比较“期望目标”和规划器目标，生成
     * CW/CCW/STOP 事件。定向期间禁止执行，防止旧的 S 值抢回控制权。
     */
    /* 定向流程拥有主轴控制权，不能被旧的 M3/M4 目标重新启动。 */
    if (spl_context.orient_request_pending ||
        spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
        spl_context.orient_hold_stop)
    {
        // // printg("187****\n");
        return;
    }

    /* command_speed 与 plc_speed 二选一，具体来源由 speed_source 决定。 */
    const double selected_speed = spindle_get_selected_speed(&spl_context);
    const double target_speed = isfinite(selected_speed) ?
        fabs(selected_speed) : 0.0;
    const spindle_state_t direction =
        (spl_context.command_state == SPINDLE_CCW) ? SPINDLE_CCW :
        (spl_context.command_state == SPINDLE_CW) ? SPINDLE_CW :
        spl_context.spl_state;
    const double signed_target =
        direction == SPINDLE_CCW ? -target_speed : target_speed;

    if (spl_context.reset_requires_start &&
        !spl_context.start_request_pending)
    {
        return;
    }

    /* 显式启动请求必须被消费，即使目标速度没变（重复同转速 M3/M4、M3/M4 S0）：
     * 命令入口已清完成反馈，这里若直接返回，请求会一直挂着，而 start_request_pending
     * 会一直挡住 F45.3 —— 主轴已在正常转速，PLC 却拿不到到速信号。
     * 只清请求、不恢复旧完成位；之后由 spindle_update_planner_state() 按实际反馈重新确认。 */
    if (spl_context.start_request_pending)
    {
        if (target_speed <= SPINDLE_SPEED_EPSILON)
        {
            /* M3/M4 S0 与 M5 等价：交给停止流程把命令字段归位并做停止判定。 */
            spl_context.spl_event = SPINDLE_EVENT_STOP;
            return;
        }
        if (fabs(signed_target - spl_context.vel_planner.target_vel) <=
            SPINDLE_SPEED_EPSILON)
        {
            spl_context.start_request_pending = false;
            return;
        }
    }

    if (fabs(signed_target - spl_context.vel_planner.target_vel) <=
        SPINDLE_SPEED_EPSILON)
    {
        return;
    }

    if (target_speed <= SPINDLE_SPEED_EPSILON)
    {
        spl_context.spl_event = SPINDLE_EVENT_STOP;
    }
    else if (direction == SPINDLE_CCW)
    {
        spl_context.spl_event = SPINDLE_EVENT_CCW;
    }
    else
    {
        spl_context.spl_event = SPINDLE_EVENT_CW;
    }
}

static void spindle_process_event(spindle_context_t *context)
{
    static bool stop_log_reported = false;

    /* spl_event 是单槽事件；本函数每周期消费一次并在末尾清空。 */
    switch (context->spl_event)
    {
    case SPINDLE_EVENT_STOP:
        /* G29.6 的停止边沿可能紧跟 G70.6 定向上升沿：定向已接管主轴后普通 STOP
         * 不能撤销使能，否则 HAL 等不到完成位（表现为定向超时）；释放/复位走
         * 专用 RELEASE_ORIENT 路径。 */
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            spl_drv_orient_state() != HAL_ORIENT_NONE ||
            context->spl_event == SPINDLE_EVENT_ORIENT)
        {
            break;
        }
        if (context->m29_phase == SPINDLE_M29_READY)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
        }
        spindle_handle_stop(context);
        if (!stop_log_reported)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴收到停止指令，当前速度=%f",
                context->vel_planner.current_vel);
            stop_log_reported = true;
        }
        break;
    case SPINDLE_EVENT_CW:
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            (context->orient_hold_stop &&
             context->m29_phase != SPINDLE_M29_READY))
            break;
        spindle_handle_rotation(context, SPINDLE_CW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_CCW:
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            (context->orient_hold_stop &&
             context->m29_phase != SPINDLE_M29_READY))
            break;
        spindle_handle_rotation(context, SPINDLE_CCW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_ORIENT:
        /*
         * 定向不是普通的 M5 停止：驱动器收到 0x040F 后会自行减速并
         * 完成定位。因此这里立即提交定向请求，不等待软件速度规划器
         * 归零；否则高速主轴可能永远等不到本地“零速”条件。
         */
        /* 定向接管主轴：清遗留启动/排队请求，命令字段归位到 STOP；"等待新启动"锁由
         * reset_requires_start=false 显式解除，不能靠 start_request_pending 兜底
         * （会挡住定向自己的完成判定）。 */
        spindle_clear_velocity_arrival();   /* 新指令：清旧到速完成 */
        context->start_request_pending = false;
        context->deferred_start_pending = false;
        context->command_state = SPINDLE_STOP;
        context->command_speed = 0.0;
        /* 显式 M19/M29 定向请求可以解除复位后的“等待新启动”锁。 */
        context->reset_requires_start = false;
        context->orient_request_pending = false;
        context->orient_release_pending = false;
        /* M19 在复位之后必须先重新进入“Operation Enabled”状态。
        -如果在这里调用 spindle_handle_stop()，就可能请求从 CSP 切换到 CSV，
        -并把定向请求覆盖成 target_state=READY_TO_SWITCH_ON。 */
        context->spl_state = SPINDLE_STOP;
        context->vel_planner.enable = false;
        context->vel_planner.target_vel = 0.0;
        s_spl_target_vel = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        spl_drv_enable();
        if (spl_orient_is_done())
        {
            /* 在开始一个新的请求之前，先等待 HAL 确认已经释放。
            -在定向模式之外，bit10 可能代表 CSV 速度已到达。 */
            const bool released = spl_orient_release();
            context->orient_phase = SPINDLE_ORIENT_RELEASING;
            context->orient_release_pending = !released;
            context->orient_request_pending = true;
            context->orient_elapsed_cycles = 0U;
            break;
        }
        /* 下面的轮询会等待使能（enable），每个周期排队一次直到被接受，
        然后在不重置其完成延时的情况下观察 HAL。 */
        context->orient_request_pending = true;
        context->orient_phase = SPINDLE_ORIENT_WAIT_ZERO;
        context->orient_elapsed_cycles = 0U;
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        /*
         * 释放既清除底层 0x040F 定向请求，也让普通主轴状态机重新接管。
         * 必须先清除 pending，避免本周期后续逻辑重新提交定向请求。
         */
        context->orient_request_pending = false;
        context->orient_release_pending = false;
        /* 释放/复位中止当前 M19，后续不应再为它产生完成反馈。 */
        context->m19_active = false;
        context->spl_state = SPINDLE_STOP;
        context->vel_planner.enable = false;
        context->vel_planner.target_vel = 0.0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        if (spl_orient_release())
        {
            context->orient_phase = SPINDLE_ORIENT_RELEASING;
            context->orient_release_pending = false;
            context->orient_elapsed_cycles = 0U;
        }
        else
        {
            /* 驱动器拒绝释放时保留阶段，后续周期继续重试。 */
            context->orient_release_pending = true;
        }
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }

    context->spl_event = SPINDLE_EVENT_NONE;
}

static void spindle_update_driver_control(void)
{
    bool velocity_control_ready = false;

    /* 安全条件优先级最高：急停或驱动器 Fault 时立即清零目标并请求下使能，
     * 否则急停释放后会把旧目标速度带回来。0x0008 是状态字的 Fault 位。 */
    if (spindle_reset_or_estop_active() ||
        (spl_drv_status_word() & 0x0008U) != 0U)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        spl_context.vel_planner.current_vel = 0.0;
        spl_context.vel_planner.active = false;
        /* 安全停机不是“到速”：禁止沿用急停前的 ARRIVED/FIN 状态。 */
        spl_context.spl_vel_state = SPINDLE_VEL_ZERO;
        s_spl_target_vel = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        /* 急停/Fault 不等待 CSP->CSV，直接清闩锁并请求下使能。 */
        spl_ms_reset();
        spl_context.csv_switch_pending = false;
        /* CSP 下没有速度目标可用，复位/急停先把期望模式扳回 CSV，
         * 免得下次 M3 还要再切一次。HAL 只记期望模式，急停中的轴不会因此被使能。 */
        if (spindle_reset_or_estop_active() &&
            (spl_drv_mode() == HAL_SPINDLE_CSP ||
             s_spl_req_mode == HAL_SPINDLE_CSP) &&
            !(s_spl_req_mode == HAL_SPINDLE_CSV && spl_ms_done()))
        {
            if (spl_ms_request(HAL_SPINDLE_CSV) != HAL_OK)
            {
                LOG(LOG_LEVEL_ERR,
                    "复位/急停后切回CSV失败：status_word=0x%04X",
                    spl_drv_status_word());
            }
        }
        spl_drv_disable();
        return;
    }

    /* 定向已经由驱动器减速，释放等待不能再次从旧规划速度执行 M5 斜坡。 */
    if (spl_context.m29_phase == SPINDLE_M29_RELEASING)
    {
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        s_spl_target_vel = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
        spl_drv_enable();
        return;
    }

    /* M29 已在定向释放阶段确认零速。切 CSP 及等待重新使能期间，
     * 不能调用普通 M5 收尾，否则零速会撤销 HAL 的使能请求。 */
    if (spl_context.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        s_spl_target_vel = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
        spl_drv_enable();
        return;
    }

    /* 模式切换期间不允许速度目标继续加速。 */
    if (spl_ms_switching())
    {
        /*
         * 模式切换期间禁止继续加速。若当前源模式是 CSV，仍让规划器
         * 输出受控减速速度；切换到其他模式时则直接保持速度目标为零。
         */
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        if (spl_drv_mode() == HAL_SPINDLE_CSV)
            spindle_update_velocity();
        else
            s_spl_target_vel = 0;
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_ERROR)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        s_spl_target_vel = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        if (spl_drv_mode() == HAL_SPINDLE_CSP)
            spl_ms_reset();
        spl_context.csv_switch_pending = false;
        spl_drv_disable();
        return;
    }

    /* 定向期间 0x040F 由 HAL 持续下发，这里不能写普通速度，否则两边打架。 */
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING)
        {
            /* 释放后 HAL 会把控制字交回状态机并**重新使能**，所以这里不能再去
             * 使能，否则下次 M19 会因为轴未使能而收不到定向请求。只清速度目标。 */
            (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        }

        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        s_spl_target_vel = 0;
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_READY)
    {
        spl_context.spl_state = SPINDLE_STOP;
        spl_context.vel_planner.enable = false;
        spl_context.vel_planner.target_vel = 0.0;
        /*
         * M29 完成后 CSP 必须保持“定向完成瞬间”的固定位置。
         * 不能在每个周期把目标位置追写成 actual_pos，否则目标会跟着
         * 电机一起移动，驱动器永远不会真正锁定定向位置；复位或后续
         * 模式切换时也会表现为主轴继续正转。
         *
         * 目标位置已经在 M29_RELEASING -> CSP 的切换入口保存，并由
         * Driver_Process_Mode_Switch 在 PREPARE_TARGET 阶段再次装载，
         * 因此 READY 阶段只保持该值，不再改写。
        */
        /* 攻丝期间的角度由**插补**给（emcmot 每拍把它下发给主轴），这里不再发位置：
         * 以前每拍重发一个固定角度，等于把主轴锁死，攻丝转不起来。
         * 这里只把插补的指令角镜像下来，供到位判据与日志使用。 */
        s_spl_target_pos = rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur;
        spindle_log_m29_csp_trace();
        spl_drv_enable();
        return;
    }

    if (spl_context.csv_switch_pending)
    {
        if (spl_drv_mode() == HAL_SPINDLE_CSP && !spl_ms_switching())
        {
            if (spl_ms_request(HAL_SPINDLE_CSV) != HAL_OK)
            {
                LOG(LOG_LEVEL_ERR,
                    "主轴CSP切换CSV请求失败，继续保持停止："
                    "status_word=0x%04X enabled=%d",
                    spl_drv_status_word(), spl_drv_enabled());
            }
            return;
        }

        if (spl_ms_done() && spl_drv_mode() == HAL_SPINDLE_CSV)
        {
            spindle_sync_planner_to_actual();
            spl_context.csv_switch_pending = false;
            if (spl_context.command_state == SPINDLE_STOP)
            {
                spl_context.spl_event = SPINDLE_EVENT_STOP;
            }
            else
            {
                spl_context.spl_event =
                    (spl_context.command_state == SPINDLE_CCW) ?
                    SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW;
            }
        }
        else if (spl_ms_result() != HAL_OK)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴CSP切换CSV失败，保留重试：status_word=0x%04X enabled=%d",
                spl_drv_status_word(), spl_drv_enabled());
            if (spl_ms_request(HAL_SPINDLE_CSV) != HAL_OK)
                spl_drv_disable();
        }
        return;
    }

    if (spl_drv_mode() == HAL_SPINDLE_CSV && spl_drv_enabled())
    {
        velocity_control_ready = true;
    }

    if (spl_context.spl_state == SPINDLE_STOP)
    {
        if (velocity_control_ready)
        {
            spindle_update_velocity();
        }
        else if (spl_drv_mode() < 0)    /* < 0 = 驱动器没报出工作模式。HAL 里 CSV == 0！ */
        {
            s_spl_target_vel = 0;
            spindle_finish_stop_if_ready();
        }
        else
        {
            /*
             * 已经不在 Operation Enabled 时，不能再做受控减速；只确保
             * 目标速度为零并等待实际速度归零，然后完成停止握手。
             */
            spindle_disable_driver_before_operation();
            /* 即使驱动器已经处于下使能状态，也要完成停止到速握手。 */
            spindle_finish_stop_if_ready();
        }
        return;
    }

    if (velocity_control_ready)
    {
        spindle_update_velocity();
        return;
    }
}

static void spindle_update_velocity(void)
{
    /*
     * 仅在驱动器处于 CSV 且允许速度控制时调用。规划器先计算 current_vel，
     * 再写入 HAL 缓存；EtherCAT 主循环负责把缓存发送到 60FF。
     */
    spl_vel_planner_t *planner = &spl_context.vel_planner;

    planner->max_acc = rt_data->axis_data[AXIS_SP].max_feed_acc;
    /* enable=false 时，规划器按最大减速度将 current_vel 拉回零。 */
    planner->enable = spindle_is_rotating() && !spl_ms_switching();
    planner->target_vel = 0.0;

    if (planner->enable)
    {
        planner->target_vel = spindle_calculate_target_velocity(&spl_context);
    }

    spindle_update_planner_state(&spl_context);

    /*
     * 规划器输出是本周期的目标速度。这里只更新 HAL 缓存，真正写入
     * EtherCAT 60FF 由主站发送周期统一完成。
     */
    s_spl_target_vel = (int32_t)planner->current_vel;
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP,
                                       fabs((double)s_spl_target_vel),
                                       s_spl_target_vel > 0 ? 1 :
                                       (s_spl_target_vel < 0 ? -1 : 0));
    spindle_finish_stop_if_ready();
}

static bool spindle_stop_is_required(const spindle_context_t *context)
{
    /* 汇总所有必须停止主轴的原因，供周期入口统一仲裁。 */
    bool stop_required = false;

    if ((spl_drv_status_word() & 0x0008U) != 0U)
    {
        return true;
    }

    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        stop_required = true;
    }
    if (rt_data->nc_event == SYS_EVENT_HMI_RESET)
    {
        stop_required = true;
    }
    if (nc_reset != NULL && nc_reset->is_resetting)
    {
        /* 复位事件只保持一个 NC 周期，复位流程期间必须持续停主轴。 */
        stop_required = true;
    }
    if (rt_data->sys_state[SYS_IS_ALARMING])
    {
        stop_required = true;
    }
    if (context->spl_event == SPINDLE_EVENT_STOP)
    {
        stop_required = true;
    }

    return stop_required;
}

static void spindle_handle_stop(spindle_context_t *context)
{
    spindle_clear_velocity_arrival();   /* 新指令：清旧到速完成 */
    /* M05 入口必须清掉遗留启动请求，否则定向释放交接时会把停止前排队的旧 M3/M4
     * 重新启动；命令字段也要归位到 STOP（停止完成判定依赖 command_state == spl_state）。 */
    context->start_request_pending = false;
    context->deferred_start_pending = false;
    context->command_state = SPINDLE_STOP;
    context->command_speed = 0.0;
    /*
     * 停止事件只撤销速度目标。普通 M5 保持驱动器可受控减速，急停/Fault
     * 则直接请求下使能；最终是否完成由 spindle_finish_stop_if_ready 判断。
     */
    context->spl_state = SPINDLE_STOP;
    context->vel_planner.enable = false;
    context->vel_planner.target_vel = 0.0;

    if (spl_drv_mode() == HAL_SPINDLE_CSP)
    {
        bool preserve_m29_request;

        /*
         * 复位/急停不能等待 CSP->CSV 的正常减速握手。CSP 下没有速度
         * 目标可供本模块执行受控减速，继续发起模式切换会把 target_pos
         * 锁在当前反馈位置并可能长期卡在切换状态，复位后主轴仍保持
         * 使能。此时立即清零目标并下使能，由驱动器安全状态机接管停车。
         */
        if (spindle_reset_or_estop_active())
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
            context->csv_switch_pending = false;
            context->command_state = SPINDLE_STOP;
            /* 复位必须取消尚未完成的 CSP/CSV 模式切换。 */
            spl_ms_reset();
            s_spl_target_pos = spl_drv_actual_pos();
            (void)rt_hal_request_axis_pos(RT_HAL_AXIS_SP, spl_drv_actual_pos());
            spl_drv_disable();
            (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
            return;
        }

        if (spl_ms_switching())
        {
            return;
        }
        preserve_m29_request = context->m29_active &&
                               context->spl_event == SPINDLE_EVENT_ORIENT;
        if (!preserve_m29_request)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
        }
        context->command_state = SPINDLE_STOP;
        if (spl_ms_request(HAL_SPINDLE_CSV) == HAL_OK)
        {
            context->csv_switch_pending = true;
        }
        else
        {
            spl_drv_disable();
        }
        return;
    }

    /* 普通 M5 保持 CSV、允许驱动器受控减速；急停/复位/驱动器 Fault 一律去使能。
     * 停止握手已完成的拍不能再重新使能（否则会把 spindle_finish_stop_if_ready() 刚下的
     * 去使能每拍盖回，主轴带力矩停着）；定向窗口 M19 与攻丝窗口 M29 READY 要带力矩。 */
    const bool stop_finished =
        spl_context.spl_state == SPINDLE_STOP &&
        spl_context.orient_phase == SPINDLE_ORIENT_IDLE &&
        spl_context.m29_phase != SPINDLE_M29_READY &&
        fabs(spl_context.vel_planner.current_vel) <= SPINDLE_SPEED_EPSILON &&
        fabs((double)spl_drv_actual_vel()) <= SPINDLE_ACTUAL_ZERO_EPSILON;
    if (!stop_finished && !spindle_reset_or_estop_active() &&
        (spl_drv_status_word() & 0x0008U) == 0U)
    {
        spl_drv_enable();
    }
    else
    {
        spl_drv_disable();
    }
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
}

static bool spindle_reset_or_estop_active(void)
{
    bool active;

    active = rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
             spl_context.reset_requires_start ||
             (rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
             (nc_reset != NULL && nc_reset->is_resetting);
    return active;
}

static void spindle_handle_rotation(spindle_context_t *context,
                                    spindle_state_t state)
{
    spindle_clear_velocity_arrival();   /* 新指令（M3/M4）：清旧到速完成 */
    if (spl_drv_mode() == HAL_SPINDLE_CSP && !spl_ms_switching())
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->command_state = state;
        context->command_speed = spindle_get_selected_speed(context);
        /* 新的 M3/M4 明确取得控制权，解除 M29 完成后的停止锁。 */
        context->orient_hold_stop = false;
        context->reset_requires_start = false;
        /* 新指令取消排队的老请求。 */
        context->start_request_pending = false;
        context->deferred_start_pending = false;
        /* 请求被拒（台账未清）也不能丢失启动请求，后续周期继续重试。 */
        (void)spl_ms_request(HAL_SPINDLE_CSV);
        context->csv_switch_pending = true;
        return;
    }

    if (context->m29_phase == SPINDLE_M29_READY)
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
    }

    /* 校验速度后切换逻辑方向、选择 CSV 模式，并允许驱动器重新使能。 */
    const double selected_speed = spindle_get_selected_speed(context);

    /* M3/M4 S0 与 M5 等价。 */
    if (isfinite(selected_speed) &&
        fabs(selected_speed) <= SPINDLE_SPEED_EPSILON)
    {
        spindle_handle_stop(context);
        return;
    }

    if (!spindle_validate_start_request(context))
    {
        return;
    }

    /*
     * 新 M3/M4 是急停/复位后的明确启动授权。清除 reset_requires_start
     * 后才允许使能主轴，防止旧的 PLC 方向信号自动恢复。
     */
    context->spl_state = state;
    if (context->reset_requires_start && !context->start_request_pending)
    {
        return;
    }
    context->reset_requires_start = false;
    /* 新指令取消排队的老请求。 */
    context->start_request_pending = false;
    context->deferred_start_pending = false;
    spindle_clear_velocity_arrival();   /* 清旧到速反馈 + F45.3 */
    context->spl_vel_state = SPINDLE_VEL_RAMP;
    /* HAL 只认实际模式（0x6061），故这里真的请求一次 CSV（不能只改本地镜像）；
     * 请求被拒 = 台账未清，返回值有意忽略。 */
    (void)spl_ms_request(HAL_SPINDLE_CSV);
    spl_drv_enable();

    // LOG(LOG_LEVEL_INFO,
    //     "主轴收到%s转指令，目标速度=%f",
    //     state == SPINDLE_CW ? "正" : "反",
    //     spindle_calculate_target_velocity(context));
}

static bool spindle_validate_start_request(const spindle_context_t *context)
{
    /* 防止没有有效 S 值时误启动主轴；报警只在首次出现时记录。 */
    const double selected_speed = spindle_get_selected_speed(context);
    const bool selected_speed_is_finite = isfinite(selected_speed);

    if (!selected_speed_is_finite)
    {
        return spindle_report_missing_speed_alarm("主轴速度值缺省或无效");
    }
    if (!isfinite(context->mcp_target_speed))
    {
        return spindle_report_missing_speed_alarm(
            "主轴速度值缺省，请先在MDI中执行M3/M4指令。");
    }
    if (fabs(context->mcp_target_speed) <= SPINDLE_SPEED_EPSILON)
    {
        return spindle_report_missing_speed_alarm(
            "主轴速度值缺省，请先在MDI中执行M3/M4指令。");
    }

    return true;
}

static bool spindle_report_missing_speed_alarm(const char *message)
{
    if (setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_MISSING_SPEED) ==
        ALARM_RESULT_ALREADY_EXISTS)
    {
        return false;
    }

    LOG(LOG_LEVEL_ERR, "%s", message);
    return false;
}

static double spindle_get_selected_speed(const spindle_context_t *context)
{
    /* G33.7 选择 PLC 速度，否则使用 CNC M3/M4 的 S 值。 */
    if (context->speed_source == SPINDLE_SPEED_SOURCE_PLC)
    {
        return context->plc_speed;
    }

    return context->command_speed;
}

static double spindle_calculate_target_velocity(const spindle_context_t *context)
{
    /* 应用主轴倍率和最大转速限制，并按 CW/CCW 输出带符号速度。 */
    const double max_speed = fabs(rt_data->axis_data[AXIS_SP].spdl_max_speed);
    const double override = var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100.0;
    double target_speed = fabs(spindle_get_selected_speed(context));

    if (context->speed_source == SPINDLE_SPEED_SOURCE_CNC)
    {
        target_speed *= fmax(0.0, override);
    }

    target_speed = fmin(max_speed, target_speed);
    if (context->spl_state == SPINDLE_CCW)
    {
        target_speed = -target_speed;
    }

    return target_speed;
}

static void spindle_update_planner_state(spindle_context_t *context)
{
    /* 软件规划器到位后仍需结合 606C 实际反馈，避免提前产生 FIN。 */
    const bool arrived = spl_vel_planner_update(&context->vel_planner, SPINDLE_CONTROL_PERIOD_SECONDS);

    if (!context->vel_planner.active)
    {
        context->spl_vel_state = SPINDLE_VEL_ZERO;
        return;
    }
    if (arrived)
    {
        /* 软件规划器到达目标不等于电机已经到速，必须继续检查 PDO 反馈。 */
        if (spindle_is_velocity_arrived())
        {
            context->spl_vel_state = SPINDLE_VEL_ARRIVED;
            spindle_report_mcode_complete();
        }
        else
        {
            context->spl_vel_state = SPINDLE_VEL_RAMP;
        }
        return;
    }

    context->spl_vel_state = SPINDLE_VEL_RAMP;
}

static bool spindle_is_velocity_arrived(void)
{
    /* 到速必须同时满足 CSV、Operation Enabled（状态字 0x27）、非零和误差带条件。 */
    if (spindle_velocity_feedback_blocked() || !spindle_is_rotating())
    {
        return false;
    }
    const double target = spl_context.vel_planner.current_vel;
    const double actual = (double)spl_drv_actual_vel();
    const double tolerance = fmax(SPINDLE_SPEED_ARRIVAL_MIN_TOL,
                                  fabs(target) * SPINDLE_SPEED_ARRIVAL_TOL);

    /*
     * 606C 实际速度和 60FF 目标速度必须来自同一 CSV 单位。除误差带外
     * 还要求实际速度非零，避免驱动器未使能或电机未转时误发 M3 FIN。
     * 状态字 0x006F == 0x0027（Operation Enabled）与 enabled 双查，
     * 避免 HAL 使能镜像滞后时用旧反馈误判到速。
     */
    if (!isfinite(target))
    {
        return false;
    }
    if (spl_drv_mode() != HAL_SPINDLE_CSV || !spl_drv_enabled() ||
        (spl_drv_status_word() & 0x006FU) != 0x0027U)
    {
        return false;
    }

    if (fabs(target) <= SPINDLE_SPEED_EPSILON ||
        fabs(actual) <= SPINDLE_ACTUAL_SPEED_EPSILON)
    {
        return false;
    }

    return fabs(actual - target) <= tolerance;
}

/* 驱动器交还速度控制权时只同步一次，普通规划周期不追写反馈。 */
static void spindle_sync_planner_to_actual(void)
{
    spl_context.vel_planner.current_vel = spl_drv_actual_vel();
    spl_context.vel_planner.target_vel = 0.0;
    spl_context.vel_planner.enable = false;
    spl_context.vel_planner.active =
        fabs(spl_context.vel_planner.current_vel) > SPINDLE_SPEED_EPSILON;
    spindle_clear_velocity_arrival();
}

static void spindle_update_orient_state(void)
{
    static bool release_status_log_active;
    static uint16_t last_release_status_word;
    static int last_release_enabled;
    static int last_release_orient_state;

    /*
     * 把 HAL 的定向进展映射到本文件的阶段状态，并在 M19 完成时产生代码握手。
     * HAL 只有 NONE / RUNNING / DONE 三态；RELEASING 是本地加的"等控制字真的交回"阶段。
     */

    /* 是否被接受由 HAL 给出，不直接看原始 bit10 */
    if (spl_context.orient_request_pending &&
        spl_context.orient_phase != SPINDLE_ORIENT_RELEASING)
    {
        const int state = spl_drv_orient_state();
        const bool accepted = state == HAL_ORIENT_RUNNING || state == HAL_ORIENT_DONE;
        if (accepted)
        {
            spl_context.orient_request_pending = false;
            spl_context.orient_phase = SPINDLE_ORIENT_RUNNING;
            spl_context.orient_elapsed_cycles = 0U;
        }
        else
        {
            if (spl_drv_enabled()) (void)spl_orient_request();
            if (++spl_context.orient_elapsed_cycles >=
                 SPINDLE_ORIENT_TIMEOUT_CYCLES)
            {
                LOG(LOG_LEVEL_ERR,
                    "主轴定向请求一直不被受理：status_word=0x%04X enabled=%d",
                    spl_drv_status_word(), spl_drv_enabled());
                setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
                spl_context.orient_request_pending = false;
                spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
                spl_context.m19_active = false;
                spl_context.orient_hold_stop = true;
                spl_context.reset_requires_start = true;
                spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
                spl_context.start_request_pending = false;
                spl_context.command_speed = 0.0;
            }
        }
    }

    /* 2) 已请求，等 HAL 把完成位置起来。 */
    if (spl_context.orient_phase == SPINDLE_ORIENT_RUNNING)
    {
        if (spl_orient_is_done())
        {
            spl_context.orient_phase = SPINDLE_ORIENT_DONE;
            spl_context.orient_elapsed_cycles = 0U;
        }
        else if (++spl_context.orient_elapsed_cycles >=
                 SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR, "主轴定向超时，自动请求释放");
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.m19_active = false;
            spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
            spl_context.orient_release_pending = true;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    /* 3) 待处理的释放请求（定向完成前的释放、复位、超时都走这里）。 */
    if (spl_context.orient_release_pending && spl_orient_release())
    {
        spl_context.orient_release_pending = false;
        if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE)
        {
            spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
        }
        spl_context.orient_elapsed_cycles = 0U;
    }

    /* 4) 释放收尾。HAL 的释放是"控制字交回状态机并重新使能"，不是退回
     *    Switch On Disabled，所以看的是定向状态已经退回 NONE。 */
    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING &&
        spl_drv_orient_inactive())
    {
        spl_context.orient_elapsed_cycles = 0U;
        release_status_log_active = false;
        if (spl_context.orient_request_pending) {
            /* 等释放信号已经发送到总线上之后，在下一个周期再重新发出请求. */
            spl_context.orient_phase = SPINDLE_ORIENT_WAIT_ZERO;
        } else {
            spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
            spl_context.orient_hold_stop = false;
            spindle_sync_planner_to_actual();
            spindle_apply_deferred_start();
        }
    }

    /* 5) 释放迟迟收不了尾就报警。这里不再退回去使能——HAL 那边已经重新使能了。 */
    if (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING)
    {
        const bool release_status_changed =
            !release_status_log_active ||
            last_release_status_word != spl_drv_status_word() ||
            last_release_enabled != spl_drv_enabled() ||
            last_release_orient_state != spl_drv_orient_state();

        if (release_status_changed)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴定向释放状态：status_word=0x%04X enabled=%d "
                "orient_state=%d actual_vel=%.0f",
                spl_drv_status_word(), spl_drv_enabled(),
                spl_drv_orient_state(), spl_drv_actual_vel());
            release_status_log_active = true;
            last_release_status_word = spl_drv_status_word();
            last_release_enabled = spl_drv_enabled();
            last_release_orient_state = spl_drv_orient_state();
        }

        if (++spl_context.orient_elapsed_cycles >= SPINDLE_ORIENT_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴定向释放超时：status_word=0x%04X enabled=%d "
                "orient_state=%d actual_vel=%.0f",
                spl_drv_status_word(), spl_drv_enabled(),
                spl_drv_orient_state(), spl_drv_actual_vel());
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
            spl_context.orient_hold_stop = true;
            spl_context.reset_requires_start = true;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            spl_context.start_request_pending = false;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }

    /* 6) M19 完成握手。 */
    if (spl_context.orient_phase == SPINDLE_ORIENT_DONE &&
        spl_context.m19_active)
    {
        spl_context.m19_active = false;
        spindle_report_mcode_complete();
    }
}

static void spindle_update_m29(void)
{
    if (!spl_context.m29_active)
    {
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_ORIENTING &&
        (spl_context.orient_phase == SPINDLE_ORIENT_RELEASING ||
         spl_context.orient_phase == SPINDLE_ORIENT_IDLE))
    {
        LOG(LOG_LEVEL_ERR,
            "M29主轴定向失败：orient_phase=%d status_word=0x%04X "
            "actual_vel=%.0f",
            spl_context.orient_phase, spl_drv_status_word(),
            spl_drv_actual_vel());
        spl_context.m29_phase = SPINDLE_M29_ERROR;
        spl_context.reset_requires_start = true;
        spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        spl_context.start_request_pending = false;
        spl_context.orient_hold_stop = true;
        spl_context.command_speed = 0.0;
        spl_context.command_state = SPINDLE_STOP;
        return;
    }

    if (spl_context.m29_phase == SPINDLE_M29_ORIENTING &&
        spl_context.orient_phase == SPINDLE_ORIENT_DONE)
    {
        if (spl_orient_release())
        {
            spl_context.m29_phase = SPINDLE_M29_RELEASING;
            spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
        }
        else
        {
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            spl_context.orient_hold_stop = true;
        }
    }

    /* 释放后 HAL 会重新使能，故判据是"定向已交回 + 主轴真的停了"，不是 Switch On Disabled；
     * 位置目标等 CSP 到位后再写（CSV 下的位置写会被 HAL 以 MODE_MISMATCH 拒收）。 */
    if (spl_context.m29_phase == SPINDLE_M29_RELEASING &&
        spl_drv_orient_inactive() &&
        fabs((double)spl_drv_actual_vel()) <= SPINDLE_ACTUAL_ZERO_EPSILON)
    {
        spindle_sync_planner_to_actual();
        s_spl_target_pos = spl_drv_actual_pos();
        LOG(LOG_LEVEL_INFO,
            "M29定向停止确认，切换CSP保持：actual_pos=%.0f actual_vel=%.0f "
            "status_word=0x%04X",
            spl_drv_actual_pos(), spl_drv_actual_vel(),
            spl_drv_status_word());
        if (spl_ms_request(HAL_SPINDLE_CSP) == HAL_OK)
        {
            spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
            spl_context.m29_phase = SPINDLE_M29_SWITCHING_CSP;
            spl_context.orient_elapsed_cycles = 0U;
        }
        else
        {
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            spl_context.orient_hold_stop = true;
        }
    }

    if (spl_context.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        if (spl_drv_mode() == HAL_SPINDLE_CSP && spl_drv_enabled())
        {
            spl_context.m29_phase = SPINDLE_M29_READY;
            /* 攻丝起点：把当前角度定为 0 度（插补算的是从本段起点起的绝对累计角
             * acc/螺距*360，必须与这里的零点对齐，否则第一个目标角会先跳回旧零点）。
             * 先设零、再写 0 度保持（适配层同拍按此顺序消费），此后角度由插补每拍下发。 */
            rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur = 0.0;
            s_spl_target_pos = 0.0;
            (void)rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0);
            (void)rt_hal_request_axis_pos(RT_HAL_AXIS_SP, 0.0);
        }
        else if (spl_ms_result() != HAL_OK)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP失败：status_word=0x%04X enabled=%d",
                spl_drv_status_word(), spl_drv_enabled());
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            spl_context.start_request_pending = false;
            spl_context.orient_hold_stop = true;
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
        }
        else if (++spl_context.orient_elapsed_cycles >=
                 SPINDLE_M29_MODE_TIMEOUT_CYCLES)
        {
            LOG(LOG_LEVEL_ERR,
                "M29切换CSP超时：status_word=0x%04X enabled=%d act_mode=%d",
                spl_drv_status_word(), spl_drv_enabled(), spl_drv_mode());
            setAlarm(ALARM_AX, ALARM_ERROR, SPINDLE_ALARM_ORIENT_TIMEOUT);
            spl_context.m29_phase = SPINDLE_M29_ERROR;
            spl_context.reset_requires_start = true;
            spindle_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            spl_context.start_request_pending = false;
            spl_context.orient_hold_stop = true;
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
            spl_context.orient_elapsed_cycles = 0U;
        }
    }
}

static void spindle_update_m29_feedback(void)
{
    static bool feedback_active;
    static uint32_t unsettled_cycles;
    static uint32_t stable_cycles;
    bool complete;
    double position_error;

    if (!spl_context.m29_active ||
        spl_context.m29_phase != SPINDLE_M29_READY)
    {
        stable_cycles = 0U;
        unsettled_cycles = 0U;
    }

    position_error = fabs((double)s_spl_target_pos -
                          (double)spl_drv_actual_pos());
    complete = spl_context.m29_active &&
               spl_context.m29_phase == SPINDLE_M29_READY &&
               spl_drv_mode() == HAL_SPINDLE_CSP &&
               spl_drv_enabled() &&
               fabs((double)spl_drv_actual_vel()) <=
                   SPINDLE_ACTUAL_ZERO_EPSILON &&
               position_error <= SPINDLE_CSP_POSITION_EPSILON;
    if (complete)
    {
        unsettled_cycles = 0U;
        stable_cycles++;
        if (stable_cycles < SPINDLE_M29_STABLE_CYCLES)
        {
                    return;
        }
        if (!feedback_active)
        {
            LOG(LOG_LEVEL_INFO,
                "M29定向及CSP切换完成：status_word=0x%04X "
                "act_mode=%d enabled=%d target_pos=%.0f actual_pos=%.0f "
                "actual_vel=%.0f position_error=%.0f",
                spl_drv_status_word(), spl_drv_mode(), spl_drv_enabled(),
                s_spl_target_pos, spl_drv_actual_pos(),
                spl_drv_actual_vel(), position_error);
            feedback_active = true;
        }
    }
    else
    {
        feedback_active = false;
        stable_cycles = 0U;
            if (spl_context.m29_active &&
            spl_context.m29_phase == SPINDLE_M29_READY &&
            (++unsettled_cycles % 1000U) == 0U)
        {
            LOG(LOG_LEVEL_WARN,
                "M29 CSP尚未稳定：target_pos=%.0f "
                "actual_pos=%.0f actual_vel=%.0f position_error=%.0f "
                "act_mode=%d enabled=%d status_word=0x%04X",
                s_spl_target_pos, spl_drv_actual_pos(),
                spl_drv_actual_vel(), position_error,
                spl_drv_mode(), spl_drv_enabled(),
                spl_drv_status_word());
        }
    }
}

/* 到速反馈的屏蔽条件：只要主轴控制权还没稳定交接，F45.3 必须保持低电平（spl_vel_state
 * 里残留 ARRIVED 也不能发布、不能据此完成停止握手）。条件只读，与发布顺序无关。 */
static bool spindle_velocity_feedback_blocked(void)
{
    if (spindle_stop_is_required(&spl_context) || spl_context.reset_requires_start)
    {
        return true;
    }
    /* 新请求尚未接管时，上一条指令的到速/零速都不能算作新指令的完成。 */
    if (spl_context.start_request_pending || spl_context.deferred_start_pending)
    {
        return true;
    }
    if (spl_context.spl_event != SPINDLE_EVENT_NONE)
    {
        return true;
    }
    if (spl_context.orient_request_pending || spl_context.orient_release_pending)
    {
        return true;
    }
    if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
        spl_context.orient_hold_stop)
    {
        return true;
    }
    if (spl_drv_orient_state() != HAL_ORIENT_NONE)
    {
        return true;
    }
    /* 模式切换（含 M29 的 CSP 保持）期间不做完成判定。 */
    if (spl_context.csv_switch_pending || spl_ms_switching())
    {
        return true;
    }
    if (spl_context.m29_active || spl_context.m29_phase != SPINDLE_M29_IDLE)
    {
        return true;
    }
    /* 命令与运行状态还没对上：例如刚收到 M3/M4、还在切模式。 */
    return spl_context.command_state != spl_context.spl_state;
}

/* 停止完成判定：控制权已交接 + 逻辑停止 + 规划器零目标 + 实际零速。
 * 定向释放交接期间（spl_state 还是 STOP 但已有排队启动）由上面的屏蔽条件挡住，
 * 不会把“还没启动”误报成“已停稳”。 */
static bool spindle_stop_is_complete(void)
{
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (spl_context.spl_state != SPINDLE_STOP)
    {
        return false;
    }
    if (fabs(spl_context.vel_planner.current_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    if (fabs(spl_context.vel_planner.target_vel) > SPINDLE_SPEED_EPSILON)
    {
        return false;
    }
    return fabs((double)spl_drv_actual_vel()) <= SPINDLE_ACTUAL_ZERO_EPSILON;
}

static void spindle_finish_stop_if_ready(void)
{
    /* 只有规划器速度和驱动器实际速度都为零，才允许完成停止握手。 */
    /* 停止完成要求软件规划速度与驱动器实际速度都为零（仅清 60FF 不代表已停，尤其在下使能
     * 或反馈延迟时）；待启动、定向窗口、模式切换、M29 期间一律不算。 */
    if (!spindle_stop_is_complete())
    {
        return;
    }

    if (spl_drv_mode() >= 0)
    {
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        spindle_disable_driver();
    }

    /* 置 ARRIVED 并锁存完成反馈：F45.3 由 PLC 发布层只读查询，保持到
     * 新指令调用 spindle_clear_velocity_arrival() 清除为止。 */
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spindle_report_mcode_complete();
}

/* 清除上一条指令的到速/完成反馈；新指令、相关 PLC 主轴事件、复位、定向与 M29 入口
 * 都要调用（否则 F45.3 会沿用上一条的到达位）。只清完成反馈，不动规划器目标。 */
void spindle_clear_velocity_arrival(void)
{
    spl_context.mcode_feedback = false;
    spl_context.spl_vel_state = SPINDLE_VEL_RAMP;
    clearRegBit(REG_F, 45, 3);
}

/* F45.3 的唯一查询入口：只读，不反向修改控制状态。内部完成标志只是必要条件——发布前
 * 重新核对当前控制权与实时反馈（防新指令在主状态更新后、F 发布前到达时沿用上一条 ARRIVED）。
 * 复位/急停/Fault、定向窗口、M29 未就绪一律不算到速。 */
bool spindle_velocity_is_ready(void)
{
    /* 控制权还没交接（待启动/定向/模式切换/M29/命令未执行）一律不算到速。 */
    if (spindle_velocity_feedback_blocked())
    {
        return false;
    }
    if (!spl_context.mcode_feedback)
    {
        return false;
    }
    if (spl_context.spl_vel_state != SPINDLE_VEL_ARRIVED)
    {
        return false;
    }
    /* 停止：ARRIVED 由 spindle_finish_stop_if_ready 在确认零速后置位，这里再按停止
     * 完成判据复核一次，不只凭 spl_state == STOP 放行。 */
    if (spl_context.spl_state == SPINDLE_STOP)
    {
        return spindle_stop_is_complete();
    }
    return spindle_is_velocity_arrived();
}

/* 定向释放确认后，在同一周期接管排队的 M03/M04 请求。交接必须自己判优先级与安全：
 *  1) 安全条件（停机要求/复位等待新启动）优先，遗留请求丢弃；
 *  2) M29 正在接管主轴时保留请求，交 M29 自己的交接流程；
 *  3) 后到的启动命令优先于先排队的老请求；
 *  4) 最新命令是停止时绝不启动（兜底，M03/M05 入口已清 deferred）；
 *  5) 决定启动就当拍调 spindle_handle_rotation 接管运行状态，否则本拍 spl_state 仍是 STOP，
 *     会被普通 M5 收尾撤使能并提前报 ARRIVED/FIN。 */
static void spindle_apply_deferred_start(void)
{
    if (spindle_stop_is_required(&spl_context) || spl_context.reset_requires_start)
    {
        spl_context.deferred_start_pending = false;
        spl_context.start_request_pending = false;
        return;
    }
    if (spl_context.m29_active)
    {
        return;
    }

    /* 后到的 CNC/PLC 启动命令优先于先前排队的延迟请求。 */
    if (spl_context.deferred_start_pending && !spl_context.start_request_pending)
    {
        spl_context.command_state = spl_context.deferred_start_state;
        spl_context.command_speed = spl_context.deferred_start_speed;
        spl_context.start_request_pending = true;
    }
    spl_context.deferred_start_pending = false;

    if (!spl_context.start_request_pending)
    {
        return;
    }
    if (spl_context.command_state == SPINDLE_STOP)
    {
        /* 最新命令是停止：不能拿排队的老请求把它顶回去。 */
        spl_context.start_request_pending = false;
        return;
    }
    spindle_handle_rotation(&spl_context, spl_context.command_state);
}

static void spindle_disable_driver_before_operation(void)
{
    /* 驱动器尚未进入 Operation Enabled 时，不能做受控减速，只能清零并下使能。 */
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
    spindle_disable_driver();
}

static void spindle_disable_driver(void)
{
    /* 停止只撤销使能、不清期望模式：HAL 侧期望模式仍是 CSV（6060/6061 = 0x09），
     * 后续 M3/M4 直接重新使能即可。 */
    spl_drv_disable();
}

static bool spindle_is_rotating(void)
{
    /* 逻辑方向用于决定规划器是否继续输出目标速度。 */
    switch (spl_context.spl_state)
    {
    case SPINDLE_CW:
    case SPINDLE_CCW:
        return true;
    case SPINDLE_STOP:
    default:
        return false;
    }
}

static void spindle_publish_status(void)
{
    /* DEBUG 模式显示规划器速度，实时模式显示实际写入驱动器的目标速度。 */
#ifdef DEBUG_MODE_ENABLED
    setVarChanInt(0, VAR_CHAN_S_CODE,
                  (int32_t)spl_context.vel_planner.current_vel, 0);
#else
    setVarChanInt(0, VAR_CHAN_S_CODE, s_spl_target_vel, 0);
#endif
}

static void spindle_report_mcode_complete(void)
{
    /* FIN/完成反馈只允许从未完成变为完成一次，避免重复触发上层握手。 */
    if (!spl_context.mcode_feedback)
    {
        spl_context.mcode_feedback = true;
        // LOG(LOG_LEVEL_DEBUG,
        //     "主轴速度到达 %f",
        //     spl_context.vel_planner.current_vel);
    }
}

static void spindle_log_m29_csp_trace(void)
{
    static uint32_t trace_cycles;
    static int32_t last_target_pos;
    static bool target_initialized;
    bool target_changed;

    trace_cycles++;
    target_changed = !target_initialized ||
                     last_target_pos != s_spl_target_pos;

    /* 目标位置变化立即记录；稳定保持时每 100 ms 记录一次，避免实时
     * 周期日志淹没现场，同时保留判断“目标不变但实际仍在运动”的证据。 */
    // if (target_changed || (trace_cycles % 100U) == 0U)
    if (target_changed)
    {
        // LOG(LOG_LEVEL_INFO,
            // "M29 CSP跟踪：target_pos=%.0f target_vel=%.0f actual_pos=%.0f "
            // "actual_vel=%.0f act_mode=%d req_mode=%d enabled=%d "
            // "status_word=0x%04X",
            // s_spl_target_pos, s_spl_target_vel, spl_drv_actual_pos(),
            // spl_drv_actual_vel(), spl_drv_mode(),
            // s_spl_req_mode, spl_drv_enabled(), spl_drv_status_word());
    }

    last_target_pos = s_spl_target_pos;
    target_initialized = true;
}
