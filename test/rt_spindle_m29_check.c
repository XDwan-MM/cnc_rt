#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

/* Exercise the production state machine without shared memory or a bus. */
#undef LOG
#define LOG(...) ((void)0)
#include "../src/motion/spindle/spl_control.c"

static RT_DATA data;
RT_DATA *rt_data = &data;
RESET_DATA *nc_reset;
VarUnion (*var_chan)[VAR_CHAN_NUM];
/* 速度规划会读主轴修调，必须给实体存储，否则空指针。 */
static VarUnion var_store[1][VAR_CHAN_NUM];
static HalCAxisStatus axis;
static HalCSpindleStatus spindle;
static unsigned alarms;
static bool orient_done;
static double actual_rpm;      /* 桩：602C 实际转速 */
static double last_req_rpm;
static int last_req_dir;

int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus *out)
{ assert(slot == RT_HAL_AXIS_SP); *out = axis; return HAL_OK; }
int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus *out)
{ assert(slot == RT_HAL_AXIS_SP); *out = spindle; return HAL_OK; }
int rt_hal_spindle_speed(RtHalAxisSlot slot, double *out)
{ assert(slot == RT_HAL_AXIS_SP); *out = actual_rpm; return HAL_OK; }
int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir)
{ assert(slot == RT_HAL_AXIS_SP); last_req_rpm = rpm; last_req_dir = dir; return HAL_OK; }
int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalSpindleMode mode)
{ assert(slot == RT_HAL_AXIS_SP); (void)mode; return HAL_OK; }
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t *out)
{ assert(slot == RT_HAL_AXIS_SP); *out = HAL_ORIENT_NONE; return HAL_OK; }
int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos)
{ assert(slot == RT_HAL_AXIS_SP); (void)pos; return HAL_OK; }
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos)
{ assert(slot == RT_HAL_AXIS_SP); (void)pos; return HAL_OK; }
bool spl_orient_request(void) { return true; }
bool spl_orient_is_done(void) { return orient_done; }
bool spl_orient_release(void) { return true; }
void spl_orient_abort(void) { }
int setAlarm(int type, int level, int id)
{ (void)type; (void)level; assert(id == SPINDLE_ALARM_ORIENT_TIMEOUT); ++alarms; return 0; }

static void prepare(int mode, int enabled)
{
    memset(&data, 0, sizeof(data));
    memset(&spl_context, 0, sizeof(spl_context));
    memset(&axis, 0, sizeof(axis));
    memset(&spindle, 0, sizeof(spindle));
    memset(var_store, 0, sizeof(var_store));
    var_chan = var_store;
    var_store[0][VAR_CHAN_SPDL_OVERRIDE].value.f = 100.0;
    data.axis_data[AXIS_SP].spdl_max_speed = 8000.0;
    data.axis_data[AXIS_SP].max_feed_acc = 8000.0;
    axis.enabled = enabled;
    axis.raw_status = enabled ? 0x27 : 0x23;
    spindle.mode = mode;
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_SWITCHING_CSP;
    data.axis_data[AXIS_SP].status.spindle_wants_torque = true;
    s_spl_req_mode = HAL_SPINDLE_CSP;
    s_spl_ms = SPL_MS_SWITCHING;
    s_spl_ms_rc = HAL_OK;
    alarms = 0;
    orient_done = false;
    actual_rpm = 0.0;
    last_req_rpm = 0.0;
    last_req_dir = 0;
}

/* ================= 交接回归：F45.3 完成判定 ================= */

static void prepare_handoff(void);

/* 还没接管的新启动请求不能算“到速”：即使内部残留 ARRIVED + 完成反馈，
 * spl_state 还是 STOP，也必须挡住（旧代码在这里直接 return true）。 */
static void test_velocity_ready_blocks_unhandled_start(void)
{
    /* (a) 排队中的延迟启动（定向释放交接那一拍） */
    prepare_handoff();
    spl_context.spl_state = SPINDLE_STOP;
    spl_context.mcode_feedback = true;
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spl_context.deferred_start_pending = true;
    assert(!spindle_velocity_is_ready());

    /* (b) 已排队的启动请求还没执行 */
    spl_context.deferred_start_pending = false;
    spl_context.start_request_pending = true;
    assert(!spindle_velocity_is_ready());

    /* (c) 命令字段与运行状态不一致（刚收到 M3/M4） */
    spl_context.start_request_pending = false;
    spl_context.command_state = SPINDLE_CW;
    assert(!spindle_velocity_is_ready());

    /* (d) 命令一致但实际速度还没归零/到位 */
    spl_context.command_state = SPINDLE_STOP;
    spl_context.spl_state = SPINDLE_CW;
    assert(!spindle_velocity_is_ready());
    puts("PASS: 未接管的启动请求不发布 F45.3");
}

/* F45.3 查询入口必须只读：发布层不再靠它反向改控制状态。 */
static void test_velocity_ready_is_read_only(void)
{
    spindle_context_t before;

    prepare_handoff();
    spl_context.spl_state = SPINDLE_STOP;
    spl_context.mcode_feedback = true;
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    before = spl_context;
    (void)spindle_velocity_is_ready();
    assert(memcmp(&before, &spl_context, sizeof(spl_context)) == 0);

    /* 停止完成路径也一样：查询不得改动任何控制状态。
     * 释放尾（orient_phase != IDLE）本来就该挡住，交接完成后才允许发布。 */
    spl_context.deferred_start_pending = false;
    spl_context.start_request_pending = false;
    spl_context.orient_phase = SPINDLE_ORIENT_IDLE;
    before = spl_context;
    assert(spindle_velocity_is_ready());
    assert(memcmp(&before, &spl_context, sizeof(spl_context)) == 0);
    puts("PASS: F45.3 查询入口只读");
}

/* 交接公共前置：CSV 已使能、无 M29、正在收释放尾、已排一个延迟启动。 */
static void prepare_handoff(void)
{
    prepare(HAL_SPINDLE_CSV, 1);
    spl_ms_reset();
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_IDLE;
    spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
    spl_context.spl_state = SPINDLE_STOP;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.command_speed = 0.0;
    spl_context.mcp_target_speed = 1000.0;
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CCW;
    spl_context.deferred_start_speed = 500.0;
}

/* 旧请求 CCW500 排队期间来了新命令 CW1000：交接必须听新命令，
 * 不能把命令字段回退成排队的旧请求。 */
static void test_handoff_newer_start_wins(void)
{
    prepare_handoff();
    spl_context.command_state = SPINDLE_CW;
    spl_context.command_speed = 1000.0;
    spl_context.start_request_pending = true;

    spindle_update_orient_state();

    assert(spl_context.orient_phase == SPINDLE_ORIENT_IDLE);
    assert(spl_context.command_state == SPINDLE_CW);
    assert(spl_context.command_speed == 1000.0);
    assert(spl_context.spl_state == SPINDLE_CW);
    assert(!spl_context.deferred_start_pending);
    assert(!spl_context.start_request_pending);
    puts("PASS: 交接优先新命令（旧 CCW500 不覆盖新 CW1000）");
}

/* 停下来的命令不能被排队的旧启动顶回去：M05 入口清队列，交接也要兜底。 */
static void test_handoff_stop_is_not_rolled_back(void)
{
    prepare_handoff();
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 500.0;

    /* M05 入口 */
    spindle_handle_stop(&spl_context);
    assert(!spl_context.deferred_start_pending);
    assert(!spl_context.start_request_pending);
    assert(spl_context.command_state == SPINDLE_STOP);

    spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
    spindle_update_orient_state();

    assert(spl_context.spl_state == SPINDLE_STOP);
    assert(spl_context.command_state == SPINDLE_STOP);
    assert(!spl_context.start_request_pending);
    assert(!spl_context.deferred_start_pending);
    puts("PASS: 停止后交接不恢复排队启动");
}

/* P1 复现：释放那一拍 spl_state 还是 STOP，延迟启动必须当拍接管运行状态，
 * 否则会掉进普通 M5 收尾（撤使能 + 提前报 ARRIVED/FIN）。 */
static void test_handoff_takes_over_same_cycle(void)
{
    prepare_handoff();
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 1000.0;
    spl_context.command_speed = 0.0;

    /* 交接前：残留“到速”状态也不能发布 */
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    spl_context.mcode_feedback = true;
    assert(!spindle_velocity_is_ready());

    spindle_update_orient_state();

    assert(spl_context.orient_phase == SPINDLE_ORIENT_IDLE);
    assert(spl_context.spl_state == SPINDLE_CW);
    assert(spl_context.command_state == SPINDLE_CW);
    assert(!spl_context.deferred_start_pending);
    assert(!spl_context.mcode_feedback);

    /* 同一拍继续走驱动器控制：不能走普通停止收尾 */
    spindle_update_driver_control();
    assert(data.axis_data[AXIS_SP].status.spindle_wants_torque);
    assert(!spl_context.mcode_feedback);
    assert(!spindle_velocity_is_ready());
    puts("PASS: 释放当拍接管延迟启动，未进入普通 M5 收尾");
}

/* M29 正在接管主轴：延迟启动既不能丢，也不能抢跑。 */
static void test_handoff_m29_keeps_deferred(void)
{
    prepare_handoff();
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_ORIENTING;

    spindle_apply_deferred_start();

    assert(spl_context.deferred_start_pending);
    assert(!spl_context.start_request_pending);
    assert(spl_context.spl_state == SPINDLE_STOP);
    puts("PASS: M29 进行中保留延迟启动请求");
}

/* 安全条件优先：复位等待新启动时必须丢弃排队的启动请求。 */
static void test_handoff_safety_cancels_deferred(void)
{
    prepare_handoff();
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 1000.0;
    spl_context.reset_requires_start = true;

    spindle_apply_deferred_start();

    assert(!spl_context.deferred_start_pending);
    assert(!spl_context.start_request_pending);
    assert(spl_context.spl_state == SPINDLE_STOP);
    puts("PASS: 安全条件取消延迟启动");
}

/* 驱动器还在 CSP：交接只请求切回 CSV，不能撤使能、也不能报完成。 */
static void test_handoff_csp_mode_keeps_torque(void)
{
    prepare_handoff();
    spindle.mode = HAL_SPINDLE_CSP;
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 1000.0;

    spindle_update_orient_state();
    assert(spl_context.command_state == SPINDLE_CW);
    assert(spl_context.csv_switch_pending);

    spindle_update_driver_control();
    assert(data.axis_data[AXIS_SP].status.spindle_wants_torque);
    assert(!spl_context.mcode_feedback);
    assert(!spindle_velocity_is_ready());
    puts("PASS: CSP 下交接保留使能等待模式切换");
}

/* 模拟 auto_mst 的 M03 入口（含新指令该做的清理）。 */
static void simulate_m03_entry(double speed)
{
    spindle_clear_velocity_arrival();
    spl_context.command_speed = speed;
    spl_context.mcp_target_speed = speed;
    spl_context.mcode_feedback = false;
    spl_context.m19_active = false;
    spl_context.orient_hold_stop = false;
    spl_context.command_state = SPINDLE_CW;
    spl_context.start_request_pending = true;
    spl_context.deferred_start_pending = false;
}

static void run_control_cycles(int n)
{
    for (int i = 0; i < n; ++i)
    {
        spindle_update_command_event();
        spindle_process_event(&spl_context);
        spindle_update_driver_control();
    }
}

/* 已在稳定转速上跑，重复同转速 M3：命令速度与规划器目标相同，
 * 新指令仍必须被消费，否则反馈屏蔽条件会把 F45.3 永久按住。 */
static void test_repeated_same_speed_start(void)
{
    prepare(HAL_SPINDLE_CSV, 1);
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_IDLE;
    spl_ms_reset();
    /* 第一条 M03 S1000 已经稳定运行 */
    spl_context.spl_state = SPINDLE_CW;
    spl_context.command_state = SPINDLE_CW;
    spl_context.command_speed = 1000.0;
    spl_context.mcp_target_speed = 1000.0;
    spl_context.vel_planner.target_vel = 1000.0;
    spl_context.vel_planner.current_vel = 1000.0;
    spl_context.vel_planner.active = true;
    spl_context.vel_planner.enable = true;
    spl_context.mcode_feedback = true;
    spl_context.spl_vel_state = SPINDLE_VEL_ARRIVED;
    actual_rpm = 1000.0;

    simulate_m03_entry(1000.0);          /* 第二条 M03 S1000：目标速度没变 */
    assert(!spindle_velocity_is_ready()); /* 新指令还没被确认 */

    run_control_cycles(100);

    assert(!spl_context.start_request_pending);   /* 请求被消费 */
    assert(spl_context.spl_state == SPINDLE_CW);
    assert(spl_context.mcode_feedback);
    assert(spindle_velocity_is_ready());          /* 按实际反馈重新确认到速 */
    puts("PASS: 重复同转速 M3 仍能重新确认到速（不再永久屏蔽）");
}

/* M3/M4 S0 等价停止：已停稳时也必须给出停止完成反馈。 */
static void test_m3_zero_speed_reports_stop(void)
{
    prepare(HAL_SPINDLE_CSV, 1);
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_IDLE;
    spl_ms_reset();
    spl_context.spl_state = SPINDLE_STOP;
    spl_context.command_state = SPINDLE_STOP;
    spl_context.mcp_target_speed = 1000.0;   /* 面板启动速度有值，避开缺省 S 报警 */
    actual_rpm = 0.0;

    simulate_m03_entry(0.0);

    run_control_cycles(100);

    assert(!spl_context.start_request_pending);
    assert(spl_context.command_state == SPINDLE_STOP);
    assert(spl_context.spl_state == SPINDLE_STOP);
    assert(spl_context.mcode_feedback);
    assert(spindle_velocity_is_ready());
    puts("PASS: M3 S0（等价停止）在已停稳时报停止完成");
}

/* F76.3 的状态来源：M29 期间置起，M29 结束/出错、M5、复位/急停落下。
 * 位本身由 plc_f_signal_handler 发布（SYS_IS_RIGID），这里只验状态。 */
static void test_rigid_state_follows_m29(void)
{
    prepare(HAL_SPINDLE_CSV, 1);
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_IDLE;
    rt_data->sys_state[SYS_IS_RIGID] = true;     /* 先污染，验证会被清掉 */
    spindle_publish_rigid_state();
    assert(!rt_data->sys_state[SYS_IS_RIGID]);

    /* 非 M29（M19 定向窗口）不能置起来 */
    spl_context.m19_active = true;
    spl_context.orient_phase = SPINDLE_ORIENT_RUNNING;
    spindle_publish_rigid_state();
    assert(!rt_data->sys_state[SYS_IS_RIGID]);
    spl_context.m19_active = false;
    spl_context.orient_phase = SPINDLE_ORIENT_IDLE;

    /* 真实入口：M29 请求 -> 置起 */
    spindle_request_m29();
    assert(spl_context.m29_phase == SPINDLE_M29_ORIENTING);
    spindle_publish_rigid_state();
    assert(rt_data->sys_state[SYS_IS_RIGID]);

    /* 握手完成后的 CSP 保持/攻丝期间仍然是"刚性攻丝方式中" */
    spl_context.m29_phase = SPINDLE_M29_READY;
    spindle_complete_m29();
    assert(!spl_context.m29_active);
    spindle_publish_rigid_state();
    assert(rt_data->sys_state[SYS_IS_RIGID]);

    /* M29 出错 -> 落下 */
    spl_context.m29_phase = SPINDLE_M29_ERROR;
    spindle_publish_rigid_state();
    assert(!rt_data->sys_state[SYS_IS_RIGID]);

    /* 走真实 M5 事件入口：阶段清回 IDLE，方式中跟着落下 */
    spl_context.m29_phase = SPINDLE_M29_READY;
    spl_context.m29_active = false;
    spindle_publish_rigid_state();
    assert(rt_data->sys_state[SYS_IS_RIGID]);

    spl_context.spl_event = SPINDLE_EVENT_STOP;
    spindle_process_event(&spl_context);
    assert(spl_context.m29_phase == SPINDLE_M29_IDLE);
    spindle_publish_rigid_state();
    assert(!rt_data->sys_state[SYS_IS_RIGID]);

    /* 接线检查：走真实周期入口，状态同样跟着 M29 阶段走 */
    rt_data->sys_state[SYS_IS_RIGID] = true;
    spindle_control_update();
    assert(!rt_data->sys_state[SYS_IS_RIGID]);

    spl_context.m29_phase = SPINDLE_M29_READY;
    spl_context.m29_active = false;
    spindle_control_update();
    assert(rt_data->sys_state[SYS_IS_RIGID]);
    puts("PASS: F76.3 状态来源（M29 期间置起，结束/出错落下）");
}

int main(void)
{
    prepare(HAL_SPINDLE_CSV, 1);
    spindle_update_driver_control();
    assert(data.axis_data[AXIS_SP].status.spindle_wants_torque);
    assert(!spl_context.mcode_feedback);

    /* Mode feedback may precede Operation Enabled by several cycles. */
    spindle.mode = HAL_SPINDLE_CSP;
    axis.enabled = 0;
    axis.raw_status = 0x23;
    for (int i = 0; i < 20; ++i) {
        spindle_update_m29();
        spindle_update_driver_control();
        assert(spl_context.m29_phase == SPINDLE_M29_SWITCHING_CSP);
        assert(data.axis_data[AXIS_SP].status.spindle_wants_torque);
        assert(!spl_context.mcode_feedback);
    }
    axis.enabled = 1;
    axis.raw_status = 0x27;
    spindle_update_m29();
    spindle_update_driver_control();
    assert(spl_context.m29_phase == SPINDLE_M29_READY);
    assert(data.axis_data[AXIS_SP].status.spindle_wants_torque);

    prepare(HAL_SPINDLE_CSV, 0);
    axis.raw_status = 0;
    for (unsigned i = 0; i < SPINDLE_M29_MODE_TIMEOUT_CYCLES; ++i) {
        spindle_update_m29();
        spindle_update_driver_control();
    }
    assert(alarms == 1);
    assert(spl_context.m29_phase == SPINDLE_M29_ERROR);
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);

    prepare(HAL_SPINDLE_CSV, 1);
    data.sys_state[SYS_IS_ESTOP_ACTIVE] = true;
    spindle_update_driver_control();
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);
    prepare(HAL_SPINDLE_CSV, 1);
    axis.raw_status = 0x08;
    spindle_update_driver_control();
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);
    prepare(HAL_SPINDLE_CSV, 1);
    spl_context.reset_requires_start = true;
    spindle_update_driver_control();
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);

    /* Ordinary M5 must still release torque when stopped. */
    prepare(HAL_SPINDLE_CSV, 1);
    spl_context.m29_active = false;
    spl_context.m29_phase = SPINDLE_M29_IDLE;
    spl_ms_reset();
    spindle_update_driver_control();
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);
    assert(spl_context.mcode_feedback);
    puts("PASS: M29 mode/enable wait, timeout, safety stops and ordinary M5");

    test_velocity_ready_blocks_unhandled_start();
    test_velocity_ready_is_read_only();
    test_handoff_newer_start_wins();
    test_handoff_stop_is_not_rolled_back();
    test_handoff_takes_over_same_cycle();
    test_handoff_m29_keeps_deferred();
    test_handoff_safety_cancels_deferred();
    test_handoff_csp_mode_keeps_torque();
    puts("PASS: 定向释放交接（延迟启动优先级/安全条件/当拍接管）");

    test_repeated_same_speed_start();
    test_m3_zero_speed_reports_stop();
    puts("PASS: 显式启动请求消费（重复同转速 M3 / M3 S0）");

    test_rigid_state_follows_m29();
    return 0;
}

/* 走真实周期入口 spindle_control_update() 会用到 HMI 变量发布，给个空实现。 */
int setVarChanInt(int no, int index, int value, int clientNo)
{ (void)no; (void)index; (void)value; (void)clientNo; return 0; }

/* 自查程序不接真实寄存器：REG_F/REG_G 的读写给空实现。
 * 签名必须与 regapi.h 一致（int f(RegType, int, int)），否则声明冲突。 */
int setRegBit(RegType type, int idx, int bit)   { (void)type; (void)idx; (void)bit; return 0; }
int clearRegBit(RegType type, int idx, int bit) { (void)type; (void)idx; (void)bit; return 0; }
