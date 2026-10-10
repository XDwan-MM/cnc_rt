#include "alarm.h"
#include "rt_data.h"
#include "rt_hal_adapter.h"   /* HAL 错误码与复位接口 */
#include "reset.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include "handwheel.h"

/* HAL 错误 → RT 报警。HAL 给正错误码 + hal_error_text() 的描述，RT 翻译成报警号。 */

/* HAL 错误码 → 报警号。
 * **这些号需要在 HMI 的报警文本表里有对应条目**，否则界面显示未知报警。
 * 旧 master.c 用 100~111 与 999，这里另起 120~ 一段避免含义冲突。 */
#define HAL_ALARM_BUS            120  /* 总线错误（含闭锁） */
#define HAL_ALARM_TIMEOUT        121  /* 启动预算耗尽或周期超时 */
#define HAL_ALARM_STOPPED        122  /* 已请求停止 */
#define HAL_ALARM_NOT_RUNNING    123  /* 尚未就绪 */
#define HAL_ALARM_STATE          124  /* 调用顺序错误 */
#define HAL_ALARM_CONFIG         125  /* 配置无效 */
#define HAL_ALARM_ABI            126  /* ABI 不匹配 */
#define HAL_ALARM_ARGUMENT       127  /* 参数无效 */
#define HAL_ALARM_MEMORY         128  /* 内存分配失败 */
#define HAL_ALARM_UNSUPPORTED    129  /* 设备不支持该功能 */
#define HAL_ALARM_BUSY           130  /* 主站已被占用 */
#define HAL_ALARM_RESET_TIMEOUT  131  /* 故障复位超时 */
#define HAL_ALARM_UNKNOWN        139  /* 未归类的 HAL 错误 */

static int hal_code_to_alarm(int32_t rc) {
    switch (rc) {
    case HAL_ERROR_BUS:         return HAL_ALARM_BUS;
    case HAL_ERROR_TIMEOUT:     return HAL_ALARM_TIMEOUT;
    case HAL_ERROR_STOPPED:     return HAL_ALARM_STOPPED;
    case HAL_ERROR_NOT_RUNNING: return HAL_ALARM_NOT_RUNNING;
    case HAL_ERROR_STATE:       return HAL_ALARM_STATE;
    case HAL_ERROR_CONFIG:      return HAL_ALARM_CONFIG;
    case HAL_ERROR_ABI:         return HAL_ALARM_ABI;
    case HAL_ERROR_ARGUMENT:    return HAL_ALARM_ARGUMENT;
    case HAL_ERROR_MEMORY:      return HAL_ALARM_MEMORY;
    case HAL_ERROR_UNSUPPORTED: return HAL_ALARM_UNSUPPORTED;
    case HAL_ERROR_BUSY:        return HAL_ALARM_BUSY;
    default:                    return HAL_ALARM_UNKNOWN;
    }
}

/* 把一个 HAL 错误码报成报警。HAL 的码是正数，不能沿用旧 RT 的 `rc < 0` 判断。
 *
 * 做成公开函数是因为 **PDO 线程必须能直接调它**：周期循环一因总线错误退出，
 * PDO 就不再 post motion_sem，运动线程卡在 sem_wait 上、alarm_poll_hal 不再跑，
 * 报警恰好会在最该报的时候丢掉。发现错误的人要能自己报出去。 */
void alarm_report_hal_error(int32_t rc) {
    if (rc == HAL_OK) return;
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    const int id = hal_code_to_alarm(rc);
    LOG(LOG_LEVEL_ERR, "HAL 错误 rc=%d（%s）-> 报警 %d", rc, text, id);
    setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, id);
}

/* 是否有一次故障复位在等收口。由 handle_alarm_cmd 的复位分支置起。 */
static int s_reset_waiting;

/* 把"每根轴该不该使能"表达给 HAL。
 *
 * 基线（4222562）里这件事在 master.c 的 Master_Run() 里做：没被屏蔽的轴推 DS402
 * 使能梯子（0x06 → 0x07 → 0x0f），被屏蔽的写 0x00。换成 HAL 之后梯子归 HAL 自己，
 * 而"要不要使能"必须由 RT **显式**说出来——缺了这一句，轴永远不会使能。
 *
 * is_disable 由 G124（各轴屏蔽，plc_g_signal_scanner.c:503/508）与参数域
 * PAR_AX_DISABLE 综合而来（stored_stroke_check.c:234-240），业务层多处都在读它。
 *
 * 每拍都发：它是状态不是事件；适配层只在值变化时真正下发，所以不产生额外总线动作。 */
void hal_axis_intent_poll() {
    /* 急停输入或报警中：**不发使能**，改走安全停止。
     *
     * 用原始急停输入 SYS_ESTOP_INPUT 而不是 SYS_IS_ESTOP_ACTIVE：后者由
     * handle_estop_cmd() 置起，中间隔着几拍；安全方向上"早一拍停"才对。
     *
     * 基线里急停是"经梯形图抬 G124 → is_disable → 写 0x00"实现的。实测本台架
     * 的梯形图**并不抬 G124**（fault 场景里急停按下时 is_disable 仍为 0），
     * 所以急停这一步必须在 RT 侧显式做——否则轴会在急停期间被使能。 */
    /* 故障复位在飞时**什么都不发**。
     *
     * HAL 里急停与去使能都会取消 PENDING 的复位（hal_context.c:757 与 :740），
     * 而且都会设 stop_override，commit 时它每拍覆盖控制字（:691）——每拍重发就等于
     * 把复位永远挡在门外：清不掉故障，也就清不掉报警，轴再也动不了。
     * 复位终态一律未使能，收口之后本函数会按当时的意图重新表达。 */
    const int estop_held = rt_data->sys_state[SYS_ESTOP_INPUT];

    /* **急停优先于复位**：物理急停一按下就放弃复位等待，改走急停。
     *
     * HAL 的契约本来就是"急停会取消待复位"（hal_rt_api.h 的 hal_rt_axis_fault_reset
     * 注释），所以让它去取消；反过来——继续等复位、把急停挡在门外——最长挡 5 秒，
     * 而复位本身要是收不了尾就是无限期。安全上不能接受这个方向。
     * 复位进度不用在这里清：HAL 收到急停后自己把 PENDING 归 NONE；适配层那道
     * "复位在飞时不下发使能"的闸门看的就是 HAL 的状态，会自己放开。 */
    if (estop_held && s_reset_waiting) {
        LOG(LOG_LEVEL_INFO, "急停按下：放弃故障复位等待，改走急停");
        s_reset_waiting = 0;
    }
    if (s_reset_waiting) return;

    /* 只在意图变化时打一行。这是"这台机床什么时候决定上力矩"的唯一记录，
     * 排查时最想知道的就是它；每拍打会刷屏。 */
    static int s_last[RT_HAL_AXIS_COUNT];
    static int s_last_valid;
    if (!s_last_valid) { memset(s_last, 0xFF, sizeof(s_last)); s_last_valid = 1; }

    /* 报警**不拦使能**（口径已定，2026-10-04）：基线（master.c 的 Master_Run()）也只认
     * 急停与 is_disable。加上报警抑制会过度锁死——实测离线场景里一条"行程超限"报警
     * （stored_stroke_check 的 Outside_Alarm，它并不设 is_disable）就能让整台机床再也
     * 使不上力矩。所以这里**有意不看 SYS_IS_ALARMING**。 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const RtHalAxisSlot slot = (RtHalAxisSlot)i;
        const int disabled = rt_data->axis_data[i].status.is_disable;
        /* 主轴还能自己要求"这拍不带力矩"（M5 停稳、CSP/CSV 切换失败、驱动器 Fault），
         * 见 spl_control.c 的 spl_drv_enable/disable。它只能**减**力矩、不能加：
         * 急停与轴屏蔽永远赢。使能字段的写者从两个收敛成这一个。 */
        const int spindle_off = (slot == RT_HAL_AXIS_SP) &&
                                !rt_data->axis_data[i].status.spindle_wants_torque;
        const int intent = estop_held ? 2 : ((disabled || spindle_off) ? 0 : 1);
        if (intent != s_last[i]) {
            s_last[i] = intent;
            LOG(LOG_LEVEL_INFO, "轴槽 %d 使能意图 -> %s", i,
                intent == 1 ? "使能" : (intent == 2 ? "急停停止" : "去使能"));
        }
        if (estop_held) {
            /* 物理急停走 HAL 的安全路径：总线故障闭锁时它仍受理（hal_context.c:748），
             * 控制字随 profile 的 estop_action。每拍重申——急停是事件，适配层不替我们
             * 记状态。 */
            (void)rt_hal_request_estop(slot);
        } else if (disabled || spindle_off) {
            /* 轴屏蔽（G124 / PAR_AX_DISABLE）是**去使能**，不是急停（口径已定，2026-10-04）。
             * 走 enable(0) 而不走 estop，是为了跟 estop_action 解耦：否则哪天把急停改成
             * "撤销电压"，例行的轴屏蔽会跟着一起断电——两件事被一个字段绑死。
             * 控制字仍是 0x07（SwitchedOn，带电不带载），并且 HAL-10 保证它不会把已有的
             * 撤销电压降级。 */
            (void)rt_hal_request_axis_enable(slot, 0);
        } else {
            (void)rt_hal_request_axis_enable(slot, 1);
        }
    }
}

/* 驱动器故障码 0x603F → rt_data[].status.is_error。
 *
 * 基线在 master 周期里做这件事（cnc_rt_last/src/master/master.c:277-320），顺带报
 * 报警 30~33；移植时那段随 master 一起删了，于是 is_error 全树**没人写** ——
 * F0.6（轴无报警）恒 1、F45.0（主轴报警）恒 0、jog 的伺服错误判断永不触发，
 * 驱动器真坏了屏幕也不说。这里把它接回来。
 *
 * **先只置标志 + 打日志，不报报警**：有些驱动器故障清除后 0x603F 还留着码，
 * 直接开 setAlarm 可能现场消不掉。等上机确认口径后再补；报警号就用基线那四个
 * 30/31/32/33（HMI 文本表里正是"X/Y/Z/主轴驱动器报错"）。
 *
 * 读不到快照（未 start 等）当"没有报警"：报不出来总比误报强。
 * 被屏蔽的轴沿用基线口径：不报。 */
static void alarm_poll_drive_errors(void) {
    static const char* const kAxisName[RT_HAL_AXIS_COUNT] = {"X", "Y", "Z", "SP"};
    static int32_t s_last_code[RT_HAL_AXIS_COUNT] = {0, 0, 0, 0};

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        HalCAxisStatus st;
        const int readable = (rt_hal_axis_status((RtHalAxisSlot)i, &st) == HAL_OK);
        const int masked = rt_data->axis_data[i].status.is_disable ? 1 : 0;
        const int has_err = readable && !masked && st.error_code_valid && st.error_code != 0;
        const int32_t code = has_err ? (int32_t)st.error_code : 0;

        rt_data->axis_data[i].status.is_error = has_err ? true : false;

        if (code != s_last_code[i]) {
            s_last_code[i] = code;
            if (code != 0)
                LOG(LOG_LEVEL_ERR, "[HAL] %s 轴驱动器报错：0x603F=0x%04X",
                    kAxisName[i], (unsigned)code);
            else
                LOG(LOG_LEVEL_INFO, "[HAL] %s 轴驱动器报警已清除", kAxisName[i]);
        }
    }
}

/* 每拍检查 HAL 侧的状态。由运动线程调用（它每拍都跑，且只读快照、不碰 context）。 */
void alarm_poll_hal() {
    /* 1) 周期循环异常退出。这是最严重的一类——主站已经跑不动了。 */
    const int32_t ce = rt_hal_cycle_error();
    if (ce) alarm_report_hal_error(ce);

    /* 2) 驱动器自己的故障码（0x603F）→ is_error（→ F45.0 / F0.6 / jog）。 */
    alarm_poll_drive_errors();

    /* 3) 故障复位的收口。等四轴都到终态（DONE 或 TIMEOUT）才结算。 */
    if (!s_reset_waiting) return;
    int timed_out = 0;
    if (!rt_hal_fault_reset_settled(&timed_out)) return;
    s_reset_waiting = 0;
    if (timed_out) {
        /* 边沿发了、驱动器没在配置时间内离开 Fault。这跟"没复位"是两回事，
         * 单独报一个号，免得现场把它当成"复位按钮没生效"。 */
        LOG(LOG_LEVEL_ERR, "故障复位超时：驱动器未在配置时间内离开 Fault 态");
        setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, HAL_ALARM_RESET_TIMEOUT);
    } else {
        LOG(LOG_LEVEL_INFO, "故障复位完成，轴保持未使能（要继续运动须显式重新使能）");
    }
}

// 急停处理逻辑
void handle_estop_cmd()
{
    // 按下急停时执行一次
    if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = true;
        // 系统事件置为NONE
        rt_data->nc_event = SYS_EVENT_NONE;
        // 人机交互状态栏显示急停报警
        setVarChanInt(0, VAR_CHAN_IS_ESTOP, 1, 0);
        // 1.急停信号ESP
        nc_set_resetting(ESTOP_RESET);
        // 清除回参考点信号
        reset_ref_state();
        // 重置各个模式
        auto_handle_estop();
        ref_handle_estop();
        jog_handle_estop();
        handwheel_handle_estop();
        LOG(LOG_LEVEL_INFO, "处理按下急停");
    }
    // 急停下不生效的事件
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_RERUN:
        rt_data->nc_event = SYS_EVENT_NONE;
        LOG(LOG_LEVEL_ERR, "急停下操作 %d 按键无效", rt_data->nc_event);
        break;
    default:
        break;
    }
}

// 报警状态逻辑
void handle_alarm_cmd()
{
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_NONE:
        break;
    case SYS_EVENT_HMI_RESET:
        // 系统急停输入信号为真时，不能响应复位事件
        if (rt_data->sys_state[SYS_ESTOP_INPUT])
            break;
        // 清除报警，不清除PLC和用户PLC报警
        cleanAllAlarm(ALARM_SY, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_CH, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_AX, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_SV, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_DV, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_PS, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_HM, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_ETHCAT_SV, ALARM_LEVEL_ALL);
        rt_data->sys_state[SYS_IS_ALARMING] = false;
        // 清除内部急停标志和人机急停显示标志
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
        setVarChanInt(0, VAR_CHAN_IS_ESTOP, 0, 0);
        // 状态复位、数据重置
        tp_reset();
        rt_data_reset();
        // 重置事件
        event_queue_reset(event_queue);
        // 更新cmd_pos位置
        update_pos_flag = 0;
        // 四轴一起发非阻塞复位请求，由 PDO 线程下发直至离开 Fault 或超时。
        // 移植时旧的 Master_Reset() 随 master_wcf.c 一起删掉了，这里必须补上 HAL 侧的
        // 替代，否则故障消不掉、轴再也上不了力矩（链上再没有第二个触发点）。
        (void)rt_hal_request_fault_reset(RT_HAL_AXIS_X);
        (void)rt_hal_request_fault_reset(RT_HAL_AXIS_Y);
        (void)rt_hal_request_fault_reset(RT_HAL_AXIS_Z);
        (void)rt_hal_request_fault_reset(RT_HAL_AXIS_SP);
        s_reset_waiting = 1;
        // 复位生效参数
        Axis_Limit_Manager_Init();
        // 复位清除跟随误差报警标志
        reset_ferror_control();
        // 清除译码器状态
        rt_set_parser_state(-1, 0);
        LOG(LOG_LEVEL_INFO, "报警已清除");
        break;
    case SYS_EVENT_RERUN:
    case SYS_EVENT_PAUSE:
    case SYS_EVENT_X_POS_PRESS:
    case SYS_EVENT_X_POS_RELEASE:
    case SYS_EVENT_X_NEG_PRESS:
    case SYS_EVENT_X_NEG_RELEASE:
    case SYS_EVENT_Y_POS_PRESS:
    case SYS_EVENT_Y_POS_RELEASE:
    case SYS_EVENT_Y_NEG_PRESS:
    case SYS_EVENT_Y_NEG_RELEASE:
    case SYS_EVENT_Z_POS_PRESS:
    case SYS_EVENT_Z_POS_RELEASE:
    case SYS_EVENT_Z_NEG_PRESS:
    case SYS_EVENT_Z_NEG_RELEASE:
    default:
        // 报警时由pmc控制面板按键是否可执行，nc侧不约束
        LOG(LOG_LEVEL_ERR, "报警下操作 %d 按键", rt_data->nc_event);
        break;
    }
}
