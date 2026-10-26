#include "rt_drive_control.h"
#include "emcmot.h"
#include "alarm.h"
/**
 * @file spindle_control.c
 * @brief 主轴控制周期入口、命令仲裁和驱动控制权切换。
 *
 * 本文件在 1 ms 运动周期中把程序命令、PLC
 * 命令、安全停车、M19 和 M29 按固定优先级串行化。速度规划、定向、
 * M29、HAL 访问和安全停车分别由同目录下的专用实现文件负责。
 */

#include "spindle_control.h"
#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include <hmisysapi.h>
#include <math.h>
#include <string.h>

#define SPINDLE_SPEED_EPSILON          (1e-6)
#define SPINDLE_ACTUAL_ZERO_EPSILON    (1.0)

spindle_context_t g_spindle;

/*
 * 主轴控制的职责边界：
 * 1. 本文件只负责命令仲裁和各子状态机的周期调度；
 * 2. CSV 速度规划与到速判定集中在 spindle_speed.c；
 * 3. 驱动器反馈和模式请求只能通过 spindle_hal.c 访问；
 * 4. M3/M4 完成必须核对驱动器实际速度，不能只依据软件规划器；
 * 5. M19、M29 和复位受控停车分别由独立文件维护自己的阶段状态。
 */

/* 内部函数声明 */

static void spindle_update_command_event(void);
static void spindle_process_event(spindle_context_t *context);
static void spindle_handle_stop(spindle_context_t *context);
static void spindle_update_driver_control(void);
static bool spindle_handle_driver_safety_state(void);
static void spindle_disable_driver_before_operation(void);
static bool spindle_is_in_csp_mode(void);

/* 只取消业务归属。公共复位协调器仍持有自己的 CSV/CSP 停车执行器，
 * 何时可以下使能、何时可以重建，由它决定。 */
static void spindle_cancel_business(void)
{
    g_spindle.reset_requires_start=true;
    if(!reset_stop_pending())g_spindle_hal.velocity_submitted=false;
    g_spindle.run_authorized=g_spindle.program_run_authorized=false;
    g_spindle.start_request_pending=g_spindle.deferred_start_pending=false;
    g_spindle.pending_event=SPINDLE_EVENT_NONE;
    spindle_orientation_abort();
    g_spindle.orient_hold_stop=false;
    g_spindle.m29_active=false;
    g_spindle.m29_phase=SPINDLE_M29_IDLE;
    g_spindle.m29_feedback_stable=false;
    g_spindle.csv_switch_pending=false;
    spindle_hal_mode_switch_reset();
    spindle_internal_clear_velocity_arrival();
}

/* 外部函数定义 */

/*
 * 单槽事件的唯一写入口。槽位仍只有一格，但覆盖顺序必须显式：
 *   ORIENT          —— 定向接管控制权，覆盖排队中的启动/释放/停止；
 *   RELEASE_ORIENT  —— 停止优先；其余允许覆盖。覆盖尚未消费的 ORIENT 表示
 *                      该定向从未下发给硬件，由释放流程本地收口；
 *   CW/CCW          —— 不得覆盖尚未消费的 ORIENT（调用方应转为 deferred_start）；
 *   STOP            —— 安全停止优先，可覆盖任何排队事件；普通 M5 的早退由
 *                      spindle_request_stop() 自己保证。
 */
void spindle_post_event(spindle_event_t event)
{
    const spindle_event_t queued = g_spindle.pending_event;

    switch (event)
    {
    case SPINDLE_EVENT_ORIENT:
    case SPINDLE_EVENT_STOP:
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        if (queued == SPINDLE_EVENT_STOP) return;
        break;
    case SPINDLE_EVENT_CW:
    case SPINDLE_EVENT_CCW:
        if (queued == SPINDLE_EVENT_ORIENT) return;
        break;
    case SPINDLE_EVENT_NONE:
    default:
        return;
    }
    g_spindle.pending_event = event;
}

/**
 * @brief 执行一个主轴控制周期。
 *
 * 控制周期按固定顺序完成安全检查、事件处理、定向更新、驱动器控制
 * 和状态发布。主轴控制相关的驱动器细节由本模块统一管理。
 */
void spindle_control_update(void)
{
    bool release_orient_event;
    bool csp_mode;

    HalAxisSnapshot feedback;
    if (spindle_hal_snapshot(&feedback)!=HAL_OK || !(feedback.feedback_valid&HAL_FEEDBACK_STATUS)) {
        spindle_cancel_business();
        g_spindle.m29_phase=SPINDLE_M29_ERROR;
        return;
    }
    spindle_hal_poll_mode_switch();
    csp_mode = spindle_is_in_csp_mode();

    if (spindle_internal_m29_no_orient_preparing() &&
        !(nc_reset && nc_reset->is_resetting) &&
        !rt_data->sys_state[SYS_ESTOP_INPUT] && !rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        spindle_internal_update_m29_no_orient();
        spindle_internal_publish_status();
        spindle_internal_publish_rigid_state();
        return;
    }

    release_orient_event =
        (g_spindle.pending_event == SPINDLE_EVENT_RELEASE_ORIENT);

    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if ((rt_data->sys_state[SYS_ESTOP_INPUT] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
         (spindle_hal_status_word() & 0x0008U) != 0U))
    {
        spindle_cancel_business();
        release_orient_event=false;
    }

    if (rt_data->nc_event==SYS_EVENT_HMI_RESET ||
        (nc_reset && nc_reset->is_resetting))
    {
        spindle_cancel_business();
        release_orient_event=false;
        if(!reset_stop_pending() &&
           (cycle_tp==NULL || cycle_tp->exec_state==EXEC_IDLE) &&
           fabs(spindle_hal_actual_velocity())<=SPINDLE_ACTUAL_ZERO_EPSILON)
            (void)alarm_spindle_request_torque(false);
    }

    if (!csp_mode && spindle_internal_stop_is_required(&g_spindle) &&
        !release_orient_event)
    {
        /* 安全停止优先于排队中的定向启动，可以覆盖 ORIENT。 */
        spindle_post_event(SPINDLE_EVENT_STOP);
    }
    else if (!release_orient_event)
    {
        /* 复位后的旧 CW/CCW 事件由下面的专用过滤逻辑处理。 */
        spindle_update_command_event();
    }

    if (g_spindle.reset_requires_start &&
        !g_spindle.start_request_pending &&
        (g_spindle.pending_event == SPINDLE_EVENT_CW ||
         g_spindle.pending_event == SPINDLE_EVENT_CCW))
    {
        /* 丢弃复位前遗留的 PLC/方向边沿，直到新的 M3/M4 到来。 */
        g_spindle.pending_event = SPINDLE_EVENT_NONE;
    }

    /* 事件处理只负责改变逻辑/驱动器目标；定向完成和速度规划在后续阶段更新。 */
    spindle_process_event(&g_spindle);
    /* HAL 负责驱动器完成判据；这里只推进 M19 业务阶段。 */
    spindle_internal_update_orientation();
    /* 同一周期消费定向 DONE；M29 先保持稳定，再请求切 CSP。 */
    spindle_internal_update_m29();
    /* 根据安全状态、定向状态和速度状态决定使能、模式及 60FF 目标值。 */
    spindle_update_driver_control();
    /* 将主轴实际反馈速度发布到 HMI/PLC 变量，供显示和 SPGET 使用。 */
    spindle_internal_publish_status();
    /* 刚性攻丝方式的状态（F76.3 的来源）每拍跟着 M29 阶段走。 */
    spindle_internal_publish_rigid_state();
}

/**
 * @brief 初始化主轴控制器的全部状态。
 *
 * 除了清空命令、定向和 M29 闩锁，还显式恢复停止态、CNC 速度源和
 * HAL 模式切换事务，确保重复初始化不会继承上一次运行的异步请求。
 */
void spindle_init(void)
{
    memset(&g_spindle, 0, sizeof(g_spindle));
    g_spindle.run_state = SPINDLE_STOP;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.velocity_state = SPINDLE_VELOCITY_ZERO;
    g_spindle.speed_source = SPINDLE_SPEED_SOURCE_CNC;
    g_spindle.remembered_program_speed =
        rt_data->axis_data[AXIS_SP].spdl_default_speed;
    spindle_hal_init();
}

bool spindle_request_program_run(spindle_state_t direction, double speed)
{
    if (direction != SPINDLE_CW && direction != SPINDLE_CCW)
    {
        return false;
    }

    /* 一条新的 M3/M4 必须先清除上一条指令锁存的完成状态。后续只有
     * 规划速度和驱动器实际速度重新满足到速条件时，F45.3 才能再次置位。 */
    g_spindle.command_speed = speed;
    g_spindle.remembered_program_speed = speed;
    g_spindle.m19_active = false;
    g_spindle.orient_hold_stop = false;
    g_spindle.command_state = direction;
    g_spindle.start_request_pending = true;
    g_spindle.program_run_authorized = true;
    g_spindle.deferred_start_pending = false;
    return true;
}

bool spindle_request_manual_run(spindle_state_t direction)
{
    if (direction != SPINDLE_CW && direction != SPINDLE_CCW)
    {
        return false;
    }

    /* 程序 M3/M4 已授权时，PLC 方向边沿不能覆盖程序方向或速度源目标。 */
    if (g_spindle.program_run_authorized)
    {
        return true;
    }

    /* M29 准备过程中主轴控制权属于刚性攻丝状态机，普通方向信号不能
     * 抢占。READY 状态例外：新的 M3/M4 会触发 CSP 到 CSV 的安全交接。 */
    if (g_spindle.m29_active ||
        (g_spindle.m29_phase != SPINDLE_M29_IDLE &&
         g_spindle.m29_phase != SPINDLE_M29_READY &&
         g_spindle.m29_phase != SPINDLE_M29_COMPLETE))
    {
        return false;
    }

    if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
        g_spindle.orient_request_pending)
    {
        /* 定向期间不能立即执行速度命令。槽里尚未消费的 ORIENT 同样算定向进行中，
         * 否则这里写入 CW 会把定向请求直接覆盖掉。只保留最后一条手动方向请求，
         * 待 HAL 确认定向释放后，由主轴模块在同一周期完成控制权交接。 */
        g_spindle.deferred_start_pending = true;
        g_spindle.deferred_start_state = direction;
        g_spindle.deferred_start_speed = g_spindle.remembered_program_speed;
        return true;
    }

    spindle_post_event(direction == SPINDLE_CCW ?
        SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW);
    g_spindle.orient_hold_stop = false;
    g_spindle.command_speed = g_spindle.remembered_program_speed;
    g_spindle.command_state = direction;
    g_spindle.start_request_pending = true;
    g_spindle.program_run_authorized = false;
    return true;
}

void spindle_request_stop(void)
{

    /* 停止命令总是先取消排队启动。即使当前 M29 或 M19 拥有控制权而不能
     * 立即执行普通 M5，也不能在稍后释放时重新播放已经作废的启动请求。 */
    g_spindle.command_speed = 0.0;
    g_spindle.command_state = SPINDLE_STOP;
    g_spindle.run_authorized = false;
    g_spindle.program_run_authorized = false;
    g_spindle.start_request_pending = false;
    g_spindle.deferred_start_pending = false;

    if (g_spindle.m29_active ||
        (g_spindle.m29_phase != SPINDLE_M29_IDLE &&
         g_spindle.m29_phase != SPINDLE_M29_READY &&
         g_spindle.m29_phase != SPINDLE_M29_COMPLETE))
    {
        return;
    }
    if (g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
        g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_request_pending)
    {
        return;
    }

    spindle_post_event(SPINDLE_EVENT_STOP);
}

void spindle_release_torque_request(void)
{
    /* 上层复位流程直接接管下使能时，清除使能仲裁器使用的主轴意愿。 */
    spindle_hal_release_torque();
}

/**
 * @brief 执行已经仲裁完成的 M3/M4 启动请求。
 *
 * 该函数供定向释放模块回调，因此属于主轴模块内部接口，但具有跨文件
 * 链接属性。函数先校验速度，再处理 CSP 到 CSV 的异步交接和复位启动锁，
 * 最后才允许速度规划器和主轴力矩意愿生效。
 */
void spindle_internal_handle_rotation(spindle_context_t *context,
                                      spindle_state_t state)
{
    spindle_internal_clear_velocity_arrival();   /* 新指令（M3/M4）：清旧到速完成 */
    g_spindle_hal.velocity_submitted=false;
    /* 零速是有效运行授权；只拒绝缺省或非有限的速度输入。 */
    if (!spindle_internal_validate_start_request(context))
    {
        context->start_request_pending=false;
        context->program_run_authorized=false;
        if(!motion_spindle_position_active())
            spindle_internal_fail_velocity(HAL_ERROR_ARGUMENT);
        return;
    }

    if (spindle_hal_mode() == HAL_AXIS_MODE_CSP && !spindle_hal_mode_switch_is_pending())
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->m29_feedback_stable = false;
        context->command_state = state;
        /* command_speed 始终保存 NC 最近一次 S 指令；PLC 速度只通过
         * spindle_internal_get_selected_speed() 参与运行目标计算，不能
         * 覆盖 F22~F25 要输出的 NC 指令速度。 */
        /* 新的 M3/M4 明确取得控制权，解除 M29 完成后的停止锁。 */
        context->orient_hold_stop = false;
        context->reset_requires_start = false;
        /* 新指令取消排队的老请求。 */
        context->start_request_pending = false;
        context->deferred_start_pending = false;
        /* 本条业务等待准备；被拒绝后结束等待，不自动重新提交。 */
        spindle_hal_mode_switch_reset();
        context->csv_switch_pending = true;
        return;
    }

    if (context->m29_phase == SPINDLE_M29_READY)
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->m29_feedback_stable = false;
    }
    else if (context->m29_phase == SPINDLE_M29_COMPLETE)
    {
        context->m29_phase = SPINDLE_M29_IDLE;
        context->m29_feedback_stable = false;
    }


    /*
     * 新 M3/M4 是急停/复位后的明确启动授权。清除 reset_requires_start
     * 后才允许使能主轴，防止旧的 PLC 方向信号自动恢复。
     */
    context->run_state = state;
    context->run_authorized = true;
    if (context->reset_requires_start && !context->start_request_pending)
    {
        return;
    }
    context->reset_requires_start = false;
    /* 新指令取消排队的老请求。 */
    context->start_request_pending = false;
    context->deferred_start_pending = false;
    spindle_internal_clear_velocity_arrival();   /* 清旧到速反馈 + F45.3 */
    context->velocity_state = SPINDLE_VELOCITY_RAMPING;
    /* 显式启动必须先完成模式准备，并保持力矩需求。 */
    if (spindle_hal_request_mode(HAL_AXIS_MODE_CSV)!=HAL_OK ||
        !alarm_spindle_request_torque(true)) {
        spindle_internal_fail_velocity(spindle_hal_mode_switch_result()!=HAL_OK ?
            spindle_hal_mode_switch_result():HAL_ERROR_STATE);
    }

}

/* 内部函数定义 */
/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    /*
     * 此函数不直接启动驱动器，只比较“期望目标”和规划器目标，生成
     * CW/CCW/STOP 事件。定向期间禁止执行，防止旧的 S 值抢回控制权。
     */
    /* 定向流程拥有主轴控制权，不能被旧的 M3/M4 目标重新启动。
     * 槽里已排队的 ORIENT 也算"定向进行中"：本函数在事件处理之前运行，
     * 若此时写入 CW 会把尚未消费的定向请求直接吞掉。 */
    if (g_spindle.orient_request_pending ||
        g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
        g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_hold_stop)
    {
        return;
    }

    if (g_spindle.reset_requires_start &&
        !g_spindle.start_request_pending)
    {
        return;
    }

    /* 显式启动请求必须被消费，即使目标速度没变（重复同转速 M3/M4、M3/M4 S0）：
     * 命令入口已清完成反馈，这里若直接返回，请求会一直挂着，而 start_request_pending
     * 会一直挡住 F45.3 —— 主轴已在正常转速，PLC 却拿不到到速信号。
     * 只清请求、不恢复旧完成位；之后由 spindle_update_planner_state() 按实际反馈重新确认。 */
    if (g_spindle.start_request_pending)
    {
        spindle_post_event(g_spindle.command_state == SPINDLE_CCW ?
            SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW);
        return;
    }
    /* 已经授权后，速度每周期由规划器直接跟随所选速度源，不重复生成启动事件。 */
}

static void spindle_process_event(spindle_context_t *context)
{
    static bool stop_log_reported = false;

    /* pending_event 是单槽事件；本函数每周期消费一次并在末尾清空。 */
    switch (context->pending_event)
    {
    case SPINDLE_EVENT_STOP:
        /* G29.6 的停止边沿可能紧跟 G70.6 定向上升沿：定向已接管主轴后普通 STOP
         * 不能撤销使能。正常退出走 RELEASE_ORIENT；复位由公共停机取消业务。 */
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            spindle_hal_orientation_state() != RT_HAL_ORIENT_NONE)
        {
            break;
        }
        if (context->m29_phase == SPINDLE_M29_READY)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
            context->m29_feedback_stable = false;
        }
        else if (context->m29_phase == SPINDLE_M29_COMPLETE)
        {
            context->m29_phase = SPINDLE_M29_IDLE;
            context->m29_feedback_stable = false;
        }
        spindle_handle_stop(context);
        if (!stop_log_reported)
        {
            LOG(LOG_LEVEL_INFO,
                "主轴收到停止指令，当前速度=%f",
                context->velocity_planner.current_vel);
            stop_log_reported = true;
        }
        break;
    case SPINDLE_EVENT_CW:
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            (context->orient_hold_stop &&
             context->m29_phase != SPINDLE_M29_READY))
            break;
        spindle_internal_handle_rotation(context, SPINDLE_CW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_CCW:
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            (context->orient_hold_stop &&
             context->m29_phase != SPINDLE_M29_READY))
            break;
        spindle_internal_handle_rotation(context, SPINDLE_CCW);
        stop_log_reported = false;
        break;
    case SPINDLE_EVENT_ORIENT:
        if(!reset_spindle_business_allowed())break;
        if(spindle_hal_mode()==HAL_AXIS_MODE_CSP && motion_spindle_prepare_velocity()!=1) {
            context->reset_requires_start=true;
            spindle_report_m29_prepare_rejected();
            break; /* explicit refusal, no mode write and no deferred takeover */
        }
        /* 不走普通 M5 归零握手；先准备 CSV 并使能，再由定向模块提交厂商动作。 */
        /* 定向接管主轴：清遗留启动/排队请求，命令字段归位到 STOP；"等待新启动"锁由
         * reset_requires_start=false 显式解除，不能靠 start_request_pending 兜底
         * （会挡住定向自己的完成判定）。 */
        spindle_internal_clear_velocity_arrival();   /* 新指令：清旧到速完成 */
        context->start_request_pending = false;
        context->deferred_start_pending = false;
        context->command_state = SPINDLE_STOP;
        context->command_speed = 0.0;
        context->run_authorized = false;
        context->program_run_authorized = false;
        /* 显式 M19/M29 定向请求可以解除复位后的“等待新启动”锁。 */
        context->reset_requires_start = false;
        context->orient_request_pending = false;
        context->orient_submitted = false;
        context->orient_exit_submitted = false;
        if (spindle_hal_request_mode(HAL_AXIS_MODE_CSV) != HAL_OK) {
            spindle_orientation_failed(false,"定向前请求CSV模式被拒绝");
            break;
        }
        /* 停止与定向是不同业务，不能在此调用普通停止收尾撤销使能需求。 */
        context->run_state = SPINDLE_STOP;
        context->velocity_planner.enable = false;
        context->velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)alarm_spindle_request_torque(true);
        if (spindle_orientation_hal_is_done())
        {
            /* 先退出上一条定向保持，确认后再发新动作，不复用旧完成信号。 */
            const spindle_orient_release_t released = spindle_orientation_release();
            if(released==SPINDLE_ORIENT_RELEASE_FAILED) {
                spindle_orientation_failed(true,"重复定向前的退出被拒绝");
                break;
            }
            if(released==SPINDLE_ORIENT_RELEASE_SUBMITTED ||
               released==SPINDLE_ORIENT_RELEASE_RETRY) {
                /* RETRY：旧保持还在定向中/未停稳，由控制周期继续尝试退出。 */
                context->orient_exit_requested = true;
                context->orient_phase = SPINDLE_ORIENT_RELEASING;
                context->orient_request_pending = true;
                context->orient_elapsed_cycles = 0U;
                break;
            }
            /* IDLE：保持动作在两次查询之间已消失，直接按新定向继续，不再等待退出。 */
        }
        /* 等模式准备与明确使能完成后只提交一次定向，再观察统一状态。 */
        context->orient_request_pending = true;
        context->orient_phase = SPINDLE_ORIENT_WAIT_ZERO;
        context->orient_elapsed_cycles = 0U;
        context->orient_stall_cycles = 0U;
        break;
    case SPINDLE_EVENT_RELEASE_ORIENT:
        /*
         * 请求正常定向交接；只有 HAL 确认退出后才能恢复普通速度业务。
         * 明确拒绝保留定向保持，不能当成公共停机取消。
         * 若定向请求尚未提交给硬件（例如复位后使能还没到位，下一条 M3 的 SF
         * 已经让梯子撤销 G70.6），没有在途动作可退，必须本地收口并执行排队启动，
         * 否则会停在无人推进的 RELEASING。
         */
        context->orient_request_pending = false;
        /* 正常退出中止当前 M19 握手，公共复位另行取消业务。 */
        context->m19_active = false;
        context->run_state = SPINDLE_STOP;
        context->velocity_planner.enable = false;
        context->velocity_planner.target_vel = 0.0;
        {
            const spindle_orient_release_t released = spindle_orientation_release();
            if (released == SPINDLE_ORIENT_RELEASE_SUBMITTED ||
                released == SPINDLE_ORIENT_RELEASE_RETRY)
            {
                /* RETRY：硬件还在定向/未停稳，控制周期按预算继续重试退出。 */
                context->orient_exit_requested = true;
                context->orient_phase = SPINDLE_ORIENT_RELEASING;
                context->orient_elapsed_cycles = 0U;
            }
            else if (released == SPINDLE_ORIENT_RELEASE_IDLE)
            {
                /* 没有在途定向动作：不下发 HAL 退出、不报警，直接交接给速度业务。 */
                spindle_orientation_complete_release();
            }
            else
            {
                spindle_orientation_failed(true,"释放定向请求被拒绝");
            }
        }
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }

    context->pending_event = SPINDLE_EVENT_NONE;
}

static void spindle_update_driver_control(void)
{
    bool velocity_control_ready = false;

    if (spindle_handle_driver_safety_state())
    {
        return;
    }

    if (g_spindle.csv_stop_pending)
    {
        spindle_internal_update_csv_stop();
        return;
    }

    /* 准备阶段仍然需要显式使能需求。交接之后的常规 CSP 保持归运动模块，
     * 与本阶段无关。 */
    if (g_spindle.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)alarm_spindle_request_torque(true);
        return;
    }

    if (spindle_hal_mode_switch_is_pending())
    {
        g_spindle.velocity_planner.enable=false;
        g_spindle.velocity_planner.target_vel=0;
        g_spindle_hal.target_velocity=0;
        return; /* 模式准备占用设备，普通速度目标不能抢先下发。 */
    }

    if (g_spindle.m29_phase == SPINDLE_M29_ERROR)
    {
        g_spindle.run_state = SPINDLE_STOP;
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        if (spindle_hal_mode() == HAL_AXIS_MODE_CSP)
            spindle_hal_mode_switch_reset();
        g_spindle.csv_switch_pending = false;
        (void)alarm_spindle_request_torque(false);
        return;
    }

    /* 定向期间 0x040F 由 HAL 持续下发，这里不能写普通速度，否则两边打架。 */
    if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        if (g_spindle.orient_phase == SPINDLE_ORIENT_RELEASING)
        {
            /* 在 HAL 确认交接之前，保持选定的保持目标不变。 */
        }

        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        return;
    }
    /* 定向已正常退出但尚无新指令：保留零速使能，不套用 M5 的下使能收尾。 */
    if(g_spindle.orient_hold_stop && !g_spindle.m29_active &&
       !g_spindle.orient_request_pending)return;

    if (spindle_hal_mode_switch_result()!=HAL_OK) {
        if(g_spindle.run_authorized || g_spindle.csv_switch_pending)
            spindle_internal_fail_velocity(spindle_hal_mode_switch_result());
        return;
    }

    if (g_spindle.csv_switch_pending)
    {
        if (spindle_hal_mode() == HAL_AXIS_MODE_CSP && !spindle_hal_mode_switch_is_pending())
        {
            const int released=motion_spindle_prepare_velocity();
            if(released==0)return;
            if(released<0 || spindle_hal_request_mode(HAL_AXIS_MODE_CSV)!=HAL_OK) {
                spindle_internal_fail_velocity(released<0?HAL_ERROR_STATE:
                    spindle_hal_mode_switch_result());
                return;
            }
            (void)alarm_spindle_request_torque(true);
            return;
        }

        if (spindle_hal_mode_switch_is_done() && spindle_hal_mode() == HAL_AXIS_MODE_CSV &&
            (g_spindle.command_state==SPINDLE_STOP || spindle_hal_is_enabled()))
        {
            spindle_internal_sync_planner_to_actual();
            g_spindle.csv_switch_pending = false;
            if (g_spindle.command_state == SPINDLE_STOP)
            {
                spindle_post_event(SPINDLE_EVENT_STOP);
            }
            else
            {
                spindle_post_event(
                    (g_spindle.command_state == SPINDLE_CCW) ?
                    SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW);
            }
        }
        return;
    }

    /*
     * CSP 正常运行时，位置目标由插补线程独占发布。M29/定向阶段和
     * CSV-CSP 模式交接已在前面处理；除此之外不发速度目标、不做普通
     * 停止收尾，也不改写插补下发的位置目标。
     */
    if (spindle_is_in_csp_mode() && !g_spindle.csv_switch_pending)
    {
        return;
    }

    if (spindle_hal_mode() == HAL_AXIS_MODE_CSV && spindle_hal_is_enabled())
    {
        velocity_control_ready = true;
    }

    if (g_spindle.run_state == SPINDLE_STOP)
    {
        if (velocity_control_ready)
        {
            spindle_internal_update_velocity();
        }
        else if (spindle_hal_mode() < 0)    /* 实际模式反馈无效，不能据此确认设备已停止。 */
        {
            g_spindle_hal.target_velocity = 0;
            spindle_internal_finish_stop_if_ready();
        }
        else
        {
            /*
             * 已经不在 Operation Enabled 时，不能再做受控减速；只确保
             * 目标速度为零并等待实际速度归零，然后完成停止握手。
             */
            spindle_disable_driver_before_operation();
            /* 即使驱动器已经处于下使能状态，也要完成停止到速握手。 */
            spindle_internal_finish_stop_if_ready();
        }
        return;
    }

    if (velocity_control_ready)
    {
        spindle_internal_update_velocity();
        return;
    }
}

static bool spindle_handle_driver_safety_state(void)
{
    bool reset_or_estop_active;
    bool should_force_disable;

    /* CSP 运行安全由插补/驱动器上层负责，主轴模块不在这里改写目标或使能。 */
    if (spindle_is_in_csp_mode())
    {
        return false;
    }

    reset_or_estop_active = spindle_internal_reset_or_estop_active();
    should_force_disable =
        ((reset_or_estop_active && !reset_stop_pending() &&
          !(g_spindle.orient_phase==SPINDLE_ORIENT_DONE && g_spindle.orient_hold_stop &&
            !spindle_internal_stop_is_required(&g_spindle))) ||
         rt_data->sys_state[SYS_ESTOP_INPUT] ||
         (spindle_hal_status_word() & 0x0008U) != 0U);
    if (!should_force_disable)
    {
        return false;
    }

    /* 安全停车优先于普通控制；不得保留规划速度或旧的到速完成状态。 */
    g_spindle.run_state = SPINDLE_STOP;
    g_spindle.velocity_planner.enable = false;
    g_spindle.velocity_planner.target_vel = 0.0;
    g_spindle.velocity_planner.current_vel = 0.0;
    g_spindle.velocity_planner.active = false;
    g_spindle.velocity_state = SPINDLE_VELOCITY_ZERO;
    g_spindle_hal.target_velocity = 0;
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);

    /* 取消业务等待。重新启动时会显式选择模式。 */
    spindle_hal_mode_switch_reset();
    g_spindle.csv_switch_pending = false;
    (void)alarm_spindle_request_torque(false);
    return true;
}

static bool spindle_is_in_csp_mode(void)
{
    /* 仅以 HAL 实际反馈判定当前模式；请求中的目标模式不代表已进入 CSP。 */
    return spindle_hal_mode() == HAL_AXIS_MODE_CSP;
}

static void spindle_handle_stop(spindle_context_t *context)
{
    context->orient_hold_stop=false;
    spindle_internal_clear_velocity_arrival();   /* 新指令：清旧到速完成 */
    /* M05 入口必须清掉遗留启动请求，否则定向释放交接时会把停止前排队的旧 M3/M4
     * 重新启动；命令字段也要归位到 STOP（停止完成判定依赖 command_state == run_state）。 */
    context->start_request_pending = false;
    context->deferred_start_pending = false;
    context->command_state = SPINDLE_STOP;
    context->command_speed = 0.0;
    context->run_authorized = false;
    context->program_run_authorized = false;
    /*
     * M5 的 CSV 减速阶段保持使能，停稳后撤销需求；HMI 复位由公共流程收口。
     * 急停/Fault 走安全裁决；CSP 攻丝复位的同步位置仍由运动模块执行。
     */
    context->run_state = SPINDLE_STOP;
    context->velocity_planner.enable = false;
    context->velocity_planner.target_vel = 0.0;

    if (spindle_hal_mode() == HAL_AXIS_MODE_CSP)
    {
        if (spindle_hal_mode_switch_is_pending())
        {
            return;
        }
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->m29_feedback_stable = false;
        context->command_state = SPINDLE_STOP;
        spindle_hal_mode_switch_reset();
        context->csv_switch_pending=true;
        return;
    }

    /* 普通 M5 和 HMI 复位受控停车保持 CSV；急停/驱动器 Fault 去使能。
     * 停止握手已完成的拍不能再重新使能（否则会把 spindle_internal_finish_stop_if_ready() 刚下的
     * 去使能每拍盖回，主轴带力矩停着）；定向窗口 M19 与攻丝窗口 M29 READY 要带力矩。 */
    bool stop_finished;

    stop_finished = (g_spindle.run_state == SPINDLE_STOP &&
                     g_spindle.orient_phase == SPINDLE_ORIENT_IDLE &&
                     fabs(g_spindle.velocity_planner.current_vel) <=
                         SPINDLE_SPEED_EPSILON &&
                     fabs(spindle_hal_actual_velocity()) <=
                         SPINDLE_ACTUAL_ZERO_EPSILON);
    if (!stop_finished &&
        (context->csv_stop_pending || !spindle_internal_reset_or_estop_active()) &&
        (spindle_hal_status_word() & 0x0008U) == 0U)
    {
        (void)alarm_spindle_request_torque(true);
    }
    else
    {
        (void)alarm_spindle_request_torque(false);
    }
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
}



static void spindle_disable_driver_before_operation(void)
{
    /* 驱动器尚未进入 Operation Enabled 时，不能做受控减速，只能清零并下使能。 */
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
    (void)alarm_spindle_request_torque(false);
}

