#include "alarm.h"
#include "rt_data.h"
#include "rt_hal_adapter.h"   /* HAL 错误码与复位接口 */
#include "reset.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include "handwheel.h"
#include <time.h>

/* HAL 错误码 → 报警号。RT 直接用 HAL 的返回值报号。
 * **这些号需要在 HMI 的报警文本表里有对应条目**，否则界面显示未知报警。 */
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

/* 把一个 HAL 错误码（正数，不能沿用旧 RT 的 `rc < 0` 判断）报成报警。
 * 公开给 **PDO 线程**直接调用：周期循环一退出，运动线程就卡在 sem_wait 上、
 * alarm_poll_hal 不再跑，报警必须在发现错误的地方报出去。 */
void alarm_report_hal_error(int32_t rc) {
    if (rc == HAL_OK) return;
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    const int id = hal_code_to_alarm(rc);
    LOG(LOG_LEVEL_ERR, "HAL 错误 rc=%d（%s）-> 报警 %d", rc, text, id);
    setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, id);
}

/* 是否有一次故障复位在等收口，由 handle_alarm_cmd 的复位分支置起。 */
static int s_reset_waiting;
/* 复位未确认成功时禁止重新上力矩；下一次显式复位才解除。 */
static int s_reset_failed;
static struct timespec s_reset_started;

bool alarm_fault_reset_pending(void) {
    return s_reset_waiting || s_reset_failed;
}

static int drive_feedback_clear(void) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        HalCAxisStatus st;
        if (rt_hal_axis_status((RtHalAxisSlot)i, &st) != HAL_OK ||
            (st.raw_status & 0x0008) ||
            (st.error_code_valid && st.error_code != 0)) return 0;
    }
    return 1;
}

/* 把"每根轴该不该使能"表达给 HAL：DS402 使能梯子由 HAL 内部推进，
 * 但"要不要使能"必须由 RT 每拍显式说出，缺了它轴永远不会使能。
 * disabled 的输入是 is_disable（G124 与 PAR_AX_DISABLE 综合）与驱动状态 is_error。
 * 它是状态不是事件：适配层只在值变化时真正下发。 */
void hal_axis_intent_poll() {
    /* 急停用原始输入 SYS_ESTOP_INPUT，不用 handle_estop_cmd() 置起的
     * SYS_IS_ESTOP_ACTIVE：后者隔着几拍，安全方向上要早一拍停。 */
    /* 故障复位在飞时**什么都不发**：急停与去使能都会取消 HAL 的待复位，
     * 每拍重发就等于把复位永远挡在门外。复位终态一律未使能，收口后按当时的意图重新表达。 */
    const int estop_held = rt_data->sys_state[SYS_ESTOP_INPUT];

    /* **急停优先于复位**：物理急停一按下就放弃复位等待，改走急停——
     * 反方向最长能把急停挡 5 秒以上，安全上不能接受。HAL 收到急停后自己取消待复位。 */
    if (estop_held && s_reset_waiting) {
        LOG(LOG_LEVEL_INFO, "急停按下：放弃故障复位等待，改走急停");
        s_reset_waiting = 0;
    }
    if (s_reset_waiting) return;

    /* 只在意图变化时打一行；每拍打会刷屏。 */
    static int s_last[RT_HAL_AXIS_COUNT];
    static int s_last_valid;
    if (!s_last_valid) { memset(s_last, 0xFF, sizeof(s_last)); s_last_valid = 1; }

    /* 报警**不拦使能**：这里**有意不看 SYS_IS_ALARMING**，只认急停与轴屏蔽；
     * 驱动器自身故障和复位失败在下面单独拦截。 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const RtHalAxisSlot slot = (RtHalAxisSlot)i;
        const int disabled = rt_data->axis_data[i].status.is_disable ||
                             rt_data->axis_data[i].status.is_error || s_reset_failed;
        /* 主轴可自己要求这拍不带力矩（spl_control.c 的 spl_drv_enable/disable）；
         * 它只能**减**力矩不能加：急停与轴屏蔽永远赢。 */
        const int spindle_off = (slot == RT_HAL_AXIS_SP) &&
                                !rt_data->axis_data[i].status.spindle_wants_torque;
        const int intent = estop_held ? 2 : ((disabled || spindle_off) ? 0 : 1);
        if (intent != s_last[i]) {
            s_last[i] = intent;
            LOG(LOG_LEVEL_INFO, "轴槽 %d 使能意图 -> %s", i,
                intent == 1 ? "使能" : (intent == 2 ? "急停停止" : "去使能"));
        }
        if (estop_held) {
            /* 物理急停走 HAL 的安全路径：总线故障闭锁时仍受理，控制字随 profile 的
             * estop_action。急停是事件，必须每拍重申（适配层不替我们记状态）。 */
            (void)rt_hal_request_estop(slot);
        } else if (disabled || spindle_off) {
            /* 轴屏蔽是**去使能**，不是急停：走 enable(0) 与 estop_action 解耦，
             * 否则改急停动作会连带切断例行轴屏蔽。控制字仍是 0x07（SwitchedOn，带电不带载）。 */
            (void)rt_hal_request_axis_enable(slot, 0);
        } else {
            (void)rt_hal_request_axis_enable(slot, 1);
        }
    }
}

/* 恢复原版轴报警 30..33；清库后仍存在的故障必须重新发布，不能只报边沿。
 * 屏蔽轴仅抑制报警，不伪造健康反馈；读失败时保留上次故障。 */
static void alarm_poll_drive_errors(void) {
    static const char* const kAxisName[RT_HAL_AXIS_COUNT] = {"X", "Y", "Z", "SP"};
    static int32_t s_last_code[RT_HAL_AXIS_COUNT] = {0, 0, 0, 0};
    static int s_last_fault[RT_HAL_AXIS_COUNT];

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        HalCAxisStatus st;
        const int readable = (rt_hal_axis_status((RtHalAxisSlot)i, &st) == HAL_OK);
        const int masked = rt_data->axis_data[i].status.is_disable ? 1 : 0;
        if (!readable) {
            if (!masked && rt_data->axis_data[i].status.is_error)
                setAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
            continue;
        }
        const int has_err = (st.raw_status & 0x0008) ||
                            (st.error_code_valid && st.error_code != 0);
        const int32_t code = st.error_code_valid ? (int32_t)st.error_code : 0;

        rt_data->axis_data[i].status.is_error = has_err ? true : false;
        if (has_err && !masked)
            setAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
        else
            delAlarm(ALARM_AX, ALARM_ERROR, 30 + i);

        if (code != s_last_code[i] || has_err != s_last_fault[i]) {
            s_last_code[i] = code;
            s_last_fault[i] = has_err;
            if (has_err)
                LOG(LOG_LEVEL_ERR, "[HAL] %s 轴驱动器报错：status_word=0x%04X 0x603F=0x%04X valid=%d",
                    kAxisName[i], (unsigned)st.raw_status, (unsigned)code, (int)st.error_code_valid);
            else
                LOG(LOG_LEVEL_INFO, "[HAL] %s 轴驱动器报警已清除", kAxisName[i]);
        }
    }
}

/* 每拍检查 HAL 侧状态，由运动线程调用（只读适配层快照，不碰 HAL context）。 */
void alarm_poll_hal() {
    /* 1) 周期循环异常退出，主站已经跑不动了。 */
    const int32_t ce = rt_hal_cycle_error();
    if (ce) alarm_report_hal_error(ce);

    /* 2) 驱动器故障码（0x603F）→ is_error。 */
    alarm_poll_drive_errors();

    /* 3) 等四轴复位终态与故障反馈清除，外层截止时间防止无限等待。 */
    if (!s_reset_waiting) return;
    int timed_out = 0;
    const int settled = rt_hal_fault_reset_settled(&timed_out);
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    /* 外层六秒：覆盖 HAL 默认五秒之外有请求未受理、快照缺失和残留 603F 的情况。 */
    const double elapsed = (double)(now.tv_sec - s_reset_started.tv_sec) +
                           (double)(now.tv_nsec - s_reset_started.tv_nsec) / 1e9;
    if (!(settled && timed_out) && !(settled && drive_feedback_clear())) {
        if (elapsed < 6.0) return;
        timed_out = 1;
    }
    s_reset_waiting = 0;
    if (timed_out) {
        s_reset_failed = 1;
        /* 边沿发了、驱动器没在配置时间内离开 Fault：与"没复位"是两回事，单独报号。 */
        LOG(LOG_LEVEL_ERR, "故障复位超时：复位未结束、反馈不可读或 Fault/0x603F 未清除");
        setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, HAL_ALARM_RESET_TIMEOUT);
    } else {
        LOG(LOG_LEVEL_INFO, "故障复位完成：Fault 与有效故障码均已清除，恢复轴使能意图管理");
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
        // 四轴一起发非阻塞复位请求，由 PDO 线程下发直至离开 Fault 或超时；
        // 这是链上唯一的复位触发点，缺了它故障消不掉。
        clock_gettime(CLOCK_MONOTONIC, &s_reset_started);
        s_reset_failed = 0;
        for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i)
            alarm_report_hal_error(rt_hal_request_fault_reset((RtHalAxisSlot)i));
        s_reset_waiting = 1;
        alarm_poll_drive_errors();
        // 复位生效参数
        Axis_Limit_Manager_Init();
        // 复位清除跟随误差报警标志
        reset_ferror_control();
        // 清除译码器状态
        rt_set_parser_state(-1, 0);
        LOG(LOG_LEVEL_INFO, "已请求故障复位，驱动器报警以反馈为准");
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
