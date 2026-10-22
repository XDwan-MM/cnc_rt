/**
 * @file spindle_control.c
 * @brief 主轴控制周期入口、命令仲裁和驱动控制权切换。
 *
 * 外部线程只提交高层请求，本文件在 1 ms 运动周期中把程序命令、PLC
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
static void spindle_disable_driver(void);
static void spindle_disable_driver_before_operation(void);

/* 外部函数定义 */

/**
 * @brief 执行一个主轴控制周期。
 *
 * 控制周期按固定顺序完成安全检查、事件处理、定向更新、驱动器控制
 * 和状态发布。主轴控制相关的驱动器细节由本模块统一管理。
 */
void spindle_control_update(void)
{
    bool release_orient_event;

    if (spindle_internal_m29_no_orient_preparing())
    {
        spindle_internal_update_m29_no_orient();
        spindle_internal_publish_status();
        spindle_internal_publish_rigid_state();
        return;
    }

    release_orient_event =
        (g_spindle.pending_event == SPINDLE_EVENT_RELEASE_ORIENT);

    if (g_spindle.reset_stop_pending &&
        spindle_internal_reset_stop_abort_required())
    {
        g_spindle.reset_stop_pending = false;
    }
    if (rt_data->nc_event == SYS_EVENT_HMI_RESET &&
        !g_spindle.reset_stop_pending && !g_spindle.reset_requires_start &&
        !spindle_internal_reset_stop_abort_required() &&
        spindle_hal_is_enabled())
    {
        const int mode = spindle_hal_mode();
        bool tapping;
        int feed_axis = -1;

        tapping = (cycle_tp != NULL && cycle_tp->plan_block != NULL &&
                   cycle_tp->exec_cmd == DO_TAP_CMD &&
                   (cycle_tp->plan_block->type == RIGID_TAP ||
                    cycle_tp->plan_block->type == TAP_RETURN));
        if (tapping)
        {
            switch (cycle_tp->plan_block->plane)
            {
                case PLANE_XY:
                    feed_axis = AXIS_Z;
                    break;
                case PLANE_XZ:
                    feed_axis = AXIS_Y;
                    break;
                case PLANE_YZ:
                    feed_axis = AXIS_X;
                    break;
                default:
                    break;
            }
        }
        if ((mode == HAL_SPINDLE_CSV &&
             (fabs(spindle_hal_actual_velocity()) > SPINDLE_ACTUAL_ZERO_EPSILON ||
              fabs(g_spindle.velocity_planner.current_vel) > SPINDLE_SPEED_EPSILON)) ||
            (mode == HAL_SPINDLE_CSP && tapping && feed_axis >= 0))
        {
            g_spindle.reset_stop_pending = true;
            g_spindle.reset_stop_csp = mode == HAL_SPINDLE_CSP;
            g_spindle.reset_stop_feed_axis = feed_axis;
            g_spindle.reset_stop_cycles = 0;
            g_spindle.reset_stop_idle_cycles = 0;
            g_spindle.reset_stop_stable_cycles = 0;
            LOG(LOG_LEVEL_INFO, "HMI复位：主轴保持%s受控停车",
                g_spindle.reset_stop_csp ? "CSP攻丝同步" : "CSV速度斜坡");
        }
    }

    /*
     * 这是 1 ms 周期入口。处理顺序很重要：先锁定安全状态，再处理
     * 新事件，随后更新定向、驱动器控制和对外状态，避免本周期刚检测
     * 到急停又被后续的普通启动逻辑覆盖。
     */
    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        (spindle_hal_status_word() & 0x0008U) != 0U)
    {
        /* 急停解除后必须等待新的 M3/M4，不能恢复急停前的旧速度。 */
        g_spindle.reset_requires_start = true;
        g_spindle.run_authorized = false;
        g_spindle.program_run_authorized = false;
        spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        g_spindle.start_request_pending = false;
        g_spindle.orient_request_pending = false;
        g_spindle.orient_release_pending = false;
        g_spindle.orient_phase = SPINDLE_ORIENT_IDLE;
        g_spindle.orient_elapsed_cycles = 0U;
        g_spindle.orient_hold_stop = false;
        g_spindle.m19_active = false;
        g_spindle.m29_active = false;
        g_spindle.m29_phase = SPINDLE_M29_IDLE;
        g_spindle.m29_feedback_stable = false;
        g_spindle.csv_switch_pending = false;
        g_spindle.deferred_start_pending = false;
        /* 安全事件取消遗留模式切换，免得闸门一直卡在"正在切换"。 */
        spindle_hal_mode_switch_reset();
        spindle_orientation_abort();
    }

    if ((rt_data->nc_event == SYS_EVENT_HMI_RESET) ||
        (nc_reset != NULL && nc_reset->is_resetting))
    {
        g_spindle.reset_requires_start = true;
        g_spindle.run_authorized = false;
        g_spindle.program_run_authorized = false;
        spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
        g_spindle.start_request_pending = false;
        g_spindle.deferred_start_pending = false;

        /*
         * 复位必须能够中止正在进行或已经完成的定向。PLC 的 G70.6
         * 下降沿通常会同时送入 RELEASE_ORIENT，但复位事件优先级更高
         * 时不能让下面的 STOP 覆盖该释放请求；没有下降沿时也要有
         * 兜底释放，避免状态永久停在 SPINDLE_ORIENT_DONE。
         */
        if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
            g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
            g_spindle.orient_request_pending)
        {
            g_spindle.pending_event = SPINDLE_EVENT_RELEASE_ORIENT;
            spindle_internal_clear_velocity_arrival();   /* 新指令/安全事件：清旧到速完成 */
            release_orient_event = true;
        }
        /* 复位结束当前 M19 握手，新的 M3/M4 可以重新取得主轴控制权。 */
        g_spindle.orient_hold_stop = false;
        g_spindle.m19_active = false;
        g_spindle.m29_active = false;
        g_spindle.m29_phase = SPINDLE_M29_IDLE;
        g_spindle.m29_feedback_stable = false;
        g_spindle.csv_switch_pending = false;
        /* HMI RESET 同样取消未完成的模式切换。 */
        spindle_hal_mode_switch_reset();
    }

    if (spindle_internal_stop_is_required(&g_spindle) && !release_orient_event)
    {
        g_spindle.pending_event = SPINDLE_EVENT_STOP;
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
    /* 直接读取 driver.c 的定向状态，并映射到 CNC/PLC 的 orient_phase。 */
    spindle_internal_update_orientation();
    /* M29 consumes the DONE phase on the following cycle, allowing the
     * NC/PLC handshake to observe orient_phase == DONE first. */
    spindle_internal_update_m29();
    spindle_internal_update_m29_feedback();
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
        rt_data->axis_data[AXIS_SP].spl_default_speed;
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
         g_spindle.m29_phase != SPINDLE_M29_READY))
    {
        return false;
    }

    if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_request_pending ||
        g_spindle.orient_release_pending)
    {
        /* 定向期间不能立即执行速度命令。只保留最后一条手动方向请求，
         * 待 HAL 确认定向释放后，由主轴模块在同一周期完成控制权交接。 */
        g_spindle.deferred_start_pending = true;
        g_spindle.deferred_start_state = direction;
        g_spindle.deferred_start_speed = g_spindle.remembered_program_speed;
        return true;
    }

    g_spindle.pending_event = direction == SPINDLE_CCW ?
        SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW;
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
         g_spindle.m29_phase != SPINDLE_M29_READY))
    {
        return;
    }
    if (g_spindle.pending_event == SPINDLE_EVENT_ORIENT ||
        g_spindle.orient_phase != SPINDLE_ORIENT_IDLE ||
        g_spindle.orient_request_pending)
    {
        return;
    }

    g_spindle.pending_event = SPINDLE_EVENT_STOP;
}

/**
 * @brief 执行已经仲裁完成的 M3/M4 启动请求。
 *
 * 该函数供定向释放模块回调，因此属于主轴模块内部接口，但具有跨文件
 * 链接属性。函数先处理 CSP 到 CSV 的异步交接，再校验速度和复位启动锁，
 * 最后才允许速度规划器和主轴力矩意愿生效。
 */
void spindle_internal_handle_rotation(spindle_context_t *context,
                                      spindle_state_t state)
{
    spindle_internal_clear_velocity_arrival();   /* 新指令（M3/M4）：清旧到速完成 */
    if (spindle_hal_mode() == HAL_SPINDLE_CSP && !spindle_hal_mode_switch_is_pending())
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
        /* 请求被拒（台账未清）也不能丢失启动请求，后续周期继续重试。 */
        (void)spindle_hal_request_mode(HAL_SPINDLE_CSV);
        context->csv_switch_pending = true;
        return;
    }

    if (context->m29_phase == SPINDLE_M29_READY)
    {
        context->m29_active = false;
        context->m29_phase = SPINDLE_M29_IDLE;
        context->m29_feedback_stable = false;
    }

    /* 零速是有效运行授权；只拒绝缺省或非有限的速度输入。 */
    if (!spindle_internal_validate_start_request(context))
    {
        return;
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
    /* HAL 只认实际模式（0x6061），故这里真的请求一次 CSV（不能只改本地镜像）；
     * 请求被拒 = 台账未清，返回值有意忽略。 */
    (void)spindle_hal_request_mode(HAL_SPINDLE_CSV);
    spindle_hal_request_torque();

}

/* 不能用读取失败时回零的便捷访问器判断停稳。 */
/* M29 卡住诊断用的只读快照：只查询，不改变任何状态（供 auto_mst 打印一行）。 */
/* 内部函数定义 */
/* CNC M3/M4/M5 只更新目标数据；目标发生变化时在控制层生成事件。 */
static void spindle_update_command_event(void)
{
    /*
     * 此函数不直接启动驱动器，只比较“期望目标”和规划器目标，生成
     * CW/CCW/STOP 事件。定向期间禁止执行，防止旧的 S 值抢回控制权。
     */
    /* 定向流程拥有主轴控制权，不能被旧的 M3/M4 目标重新启动。 */
    if (g_spindle.orient_request_pending ||
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
        g_spindle.pending_event = g_spindle.command_state == SPINDLE_CCW ?
            SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW;
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
         * 不能撤销使能，否则 HAL 等不到完成位（表现为定向超时）；释放/复位走
         * 专用 RELEASE_ORIENT 路径。 */
        if (context->orient_phase != SPINDLE_ORIENT_IDLE ||
            context->orient_request_pending ||
            spindle_hal_orientation_state() != HAL_ORIENT_NONE ||
            context->pending_event == SPINDLE_EVENT_ORIENT)
        {
            break;
        }
        if (context->m29_phase == SPINDLE_M29_READY)
        {
            context->m29_active = false;
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
        /*
         * 定向不是普通的 M5 停止：驱动器收到 0x040F 后会自行减速并
         * 完成定位。因此这里立即提交定向请求，不等待软件速度规划器
         * 归零；否则高速主轴可能永远等不到本地“零速”条件。
         */
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
        context->orient_release_pending = false;
        /* M19 在复位之后必须先重新进入“Operation Enabled”状态。
        -如果在这里调用 spindle_handle_stop()，就可能请求从 CSP 切换到 CSV，
        -并把定向请求覆盖成 target_state=READY_TO_SWITCH_ON。 */
        context->run_state = SPINDLE_STOP;
        context->velocity_planner.enable = false;
        context->velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        spindle_hal_request_torque();
        if (spindle_orientation_hal_is_done())
        {
            /* 在开始一个新的请求之前，先等待 HAL 确认已经释放。
            -在定向模式之外，bit10 可能代表 CSV 速度已到达。 */
            const bool released = spindle_orientation_release();
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
        context->run_state = SPINDLE_STOP;
        context->velocity_planner.enable = false;
        context->velocity_planner.target_vel = 0.0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        if (spindle_orientation_release())
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

    context->pending_event = SPINDLE_EVENT_NONE;
}

static void spindle_update_driver_control(void)
{
    bool velocity_control_ready = false;

    /* 急停、驱动器 Fault 和未进入受控停车的复位立即下使能。
     * 0x0008 是状态字的 Fault 位。 */
    if ((spindle_internal_reset_or_estop_active() && !g_spindle.reset_stop_pending) ||
        rt_data->sys_state[SYS_ESTOP_INPUT] ||
        (spindle_hal_status_word() & 0x0008U) != 0U)
    {
        g_spindle.run_state = SPINDLE_STOP;
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle.velocity_planner.current_vel = 0.0;
        g_spindle.velocity_planner.active = false;
        /* 安全停机不是“到速”：禁止沿用急停前的 ARRIVED/FIN 状态。 */
        g_spindle.velocity_state = SPINDLE_VELOCITY_ZERO;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        /* 急停/Fault 不等待 CSP->CSV，直接清闩锁并请求下使能。 */
        spindle_hal_mode_switch_reset();
        g_spindle.csv_switch_pending = false;
        /* CSP 下没有速度目标可用，复位/急停先把期望模式扳回 CSV，
         * 免得下次 M3 还要再切一次。HAL 只记期望模式，急停中的轴不会因此被使能。 */
        if (spindle_internal_reset_or_estop_active() &&
            (spindle_hal_mode() == HAL_SPINDLE_CSP ||
             g_spindle_hal.requested_mode == HAL_SPINDLE_CSP) &&
            !(g_spindle_hal.requested_mode == HAL_SPINDLE_CSV && spindle_hal_mode_switch_is_done()))
        {
            if (spindle_hal_request_mode(HAL_SPINDLE_CSV) != HAL_OK)
            {
                LOG(LOG_LEVEL_ERR,
                    "复位/急停后切回CSV失败：status_word=0x%04X",
                    spindle_hal_status_word());
            }
        }
        spindle_hal_release_torque();
        return;
    }

    if (g_spindle.reset_stop_pending)
    {
        spindle_internal_update_reset_stop();
        return;
    }

    /* 定向已经由驱动器减速，释放等待不能再次从旧规划速度执行 M5 斜坡。 */
    if (g_spindle.m29_phase == SPINDLE_M29_RELEASING)
    {
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
        spindle_hal_request_torque();
        return;
    }

    /* M29 已在定向释放阶段确认零速。切 CSP 及等待重新使能期间，
     * 不能调用普通 M5 收尾，否则零速会撤销 HAL 的使能请求。 */
    if (g_spindle.m29_phase == SPINDLE_M29_SWITCHING_CSP)
    {
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);
        spindle_hal_request_torque();
        return;
    }

    /* 模式切换期间不允许速度目标继续加速。 */
    if (spindle_hal_mode_switch_is_pending())
    {
        /*
         * 模式切换期间禁止继续加速。若当前源模式是 CSV，仍让规划器
         * 输出受控减速速度；切换到其他模式时则直接保持速度目标为零。
         */
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        if (spindle_hal_mode() == HAL_SPINDLE_CSV)
            spindle_internal_update_velocity();
        else
            g_spindle_hal.target_velocity = 0;
        return;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_ERROR)
    {
        g_spindle.run_state = SPINDLE_STOP;
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        if (spindle_hal_mode() == HAL_SPINDLE_CSP)
            spindle_hal_mode_switch_reset();
        g_spindle.csv_switch_pending = false;
        spindle_hal_release_torque();
        return;
    }

    /* 定向期间 0x040F 由 HAL 持续下发，这里不能写普通速度，否则两边打架。 */
    if (g_spindle.orient_phase != SPINDLE_ORIENT_IDLE)
    {
        if (g_spindle.orient_phase == SPINDLE_ORIENT_RELEASING)
        {
            /* 释放后 HAL 会把控制字交回状态机并**重新使能**，所以这里不能再去
             * 使能，否则下次 M19 会因为轴未使能而收不到定向请求。只清速度目标。 */
            (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
        }

        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
        g_spindle_hal.target_velocity = 0;
        return;
    }

    if (g_spindle.m29_phase == SPINDLE_M29_READY)
    {
        g_spindle.run_state = SPINDLE_STOP;
        g_spindle.velocity_planner.enable = false;
        g_spindle.velocity_planner.target_vel = 0.0;
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
        g_spindle_hal.target_position = rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur;
                spindle_hal_request_torque();
        return;
    }

    if (g_spindle.csv_switch_pending)
    {
        if (spindle_hal_mode() == HAL_SPINDLE_CSP && !spindle_hal_mode_switch_is_pending())
        {
            if (spindle_hal_request_mode(HAL_SPINDLE_CSV) != HAL_OK)
            {
                LOG(LOG_LEVEL_ERR,
                    "主轴CSP切换CSV请求失败，继续保持停止："
                    "status_word=0x%04X enabled=%d",
                    spindle_hal_status_word(), spindle_hal_is_enabled());
            }
            return;
        }

        if (spindle_hal_mode_switch_is_done() && spindle_hal_mode() == HAL_SPINDLE_CSV)
        {
            spindle_internal_sync_planner_to_actual();
            g_spindle.csv_switch_pending = false;
            if (g_spindle.command_state == SPINDLE_STOP)
            {
                g_spindle.pending_event = SPINDLE_EVENT_STOP;
            }
            else
            {
                g_spindle.pending_event =
                    (g_spindle.command_state == SPINDLE_CCW) ?
                    SPINDLE_EVENT_CCW : SPINDLE_EVENT_CW;
            }
        }
        else if (spindle_hal_mode_switch_result() != HAL_OK)
        {
            LOG(LOG_LEVEL_ERR,
                "主轴CSP切换CSV失败，保留重试：status_word=0x%04X enabled=%d",
                spindle_hal_status_word(), spindle_hal_is_enabled());
            if (spindle_hal_request_mode(HAL_SPINDLE_CSV) != HAL_OK)
                spindle_hal_release_torque();
        }
        return;
    }

    if (spindle_hal_mode() == HAL_SPINDLE_CSV && spindle_hal_is_enabled())
    {
        velocity_control_ready = true;
    }

    if (g_spindle.run_state == SPINDLE_STOP)
    {
        if (velocity_control_ready)
        {
            spindle_internal_update_velocity();
        }
        else if (spindle_hal_mode() < 0)    /* < 0 = 驱动器没报出工作模式。HAL 里 CSV == 0！ */
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

static void spindle_handle_stop(spindle_context_t *context)
{
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
     * 停止事件只撤销速度目标。普通 M5 和 HMI 复位的 CSV 停车保持驱动器使能，
     * 急停/Fault 则直接下使能；CSP 攻丝复位由插补继续下发同步位置。
     */
    context->run_state = SPINDLE_STOP;
    context->velocity_planner.enable = false;
    context->velocity_planner.target_vel = 0.0;

    if (context->reset_stop_pending && context->reset_stop_csp)
        return;

    if (spindle_hal_mode() == HAL_SPINDLE_CSP)
    {
        bool preserve_m29_request;

        /*
         * 复位/急停不能等待 CSP->CSV 的正常减速握手。CSP 下没有速度
         * 目标可供本模块执行受控减速，继续发起模式切换会把 target_pos
         * 锁在当前反馈位置并可能长期卡在切换状态，复位后主轴仍保持
         * 使能。此时立即清零目标并下使能，由驱动器安全状态机接管停车。
         */
        if (spindle_internal_reset_or_estop_active())
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
            context->m29_feedback_stable = false;
            context->csv_switch_pending = false;
            context->command_state = SPINDLE_STOP;
            /* 复位必须取消尚未完成的 CSP/CSV 模式切换。 */
            spindle_hal_mode_switch_reset();
            g_spindle_hal.target_position = spindle_hal_actual_position();
            (void)rt_hal_request_axis_pos(RT_HAL_AXIS_SP, spindle_hal_actual_position());
            spindle_hal_release_torque();
            (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
            return;
        }

        if (spindle_hal_mode_switch_is_pending())
        {
            return;
        }
        preserve_m29_request = context->m29_active &&
                               context->pending_event == SPINDLE_EVENT_ORIENT;
        if (!preserve_m29_request)
        {
            context->m29_active = false;
            context->m29_phase = SPINDLE_M29_IDLE;
            context->m29_feedback_stable = false;
        }
        context->command_state = SPINDLE_STOP;
        if (spindle_hal_request_mode(HAL_SPINDLE_CSV) == HAL_OK)
        {
            context->csv_switch_pending = true;
        }
        else
        spindle_hal_release_torque();
        return;
    }

    /* 普通 M5 和 HMI 复位受控停车保持 CSV；急停/驱动器 Fault 去使能。
     * 停止握手已完成的拍不能再重新使能（否则会把 spindle_internal_finish_stop_if_ready() 刚下的
     * 去使能每拍盖回，主轴带力矩停着）；定向窗口 M19 与攻丝窗口 M29 READY 要带力矩。 */
    bool stop_finished;

    stop_finished = (g_spindle.run_state == SPINDLE_STOP &&
                     g_spindle.orient_phase == SPINDLE_ORIENT_IDLE &&
                     g_spindle.m29_phase != SPINDLE_M29_READY &&
                     fabs(g_spindle.velocity_planner.current_vel) <=
                         SPINDLE_SPEED_EPSILON &&
                     fabs(spindle_hal_actual_velocity()) <=
                         SPINDLE_ACTUAL_ZERO_EPSILON);
    if (!stop_finished &&
        (context->reset_stop_pending || !spindle_internal_reset_or_estop_active()) &&
        (spindle_hal_status_word() & 0x0008U) == 0U)
    {
        spindle_hal_request_torque();
    }
    else
    {
        spindle_hal_release_torque();
    }
    (void)rt_hal_request_spindle_speed(RT_HAL_AXIS_SP, 0.0, 0);   /* 停转 */
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
    spindle_hal_release_torque();
}

