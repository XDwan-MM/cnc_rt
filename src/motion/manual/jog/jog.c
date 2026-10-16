#include "jog.h"
#include "global.h"
#include "jog_planner.h"
#include "rt_hal_adapter.h"
#include <math.h>
#include <string.h>

/*
 * JOG 的公共入口只有周期处理、急停和速度查询；外部减速状态保存在 AxisStatus。
 * 本文件把输入锁存、保护判断、状态迁移、速度计算、规划器和输出
 * 按阶段分开；新增停止条件时优先扩展 stop_reason 或保护判断，
 * 不要再为每一种原因增加一个运动状态。
 * 
 * 机械锁住：事件只改“请求” jog_update_mac_lock()
 * 状态不一致期间由 jog_transition_axis() 的切换分支让点动先停稳，
 * 全轴空闲时在 jog_update_outputs() 里落“生效”（不做周期性的速度判断）
 * 它只决定机床指令位置是否累加，不改变点动本身的规划与按键锁存。
 */

typedef enum
{
    /* 状态只描述运动阶段；停止的具体原因由 stop_reason 单独记录。 */
    JOG_AXIS_STATE_IDLE = 0,
    JOG_AXIS_STATE_RUNNING,
    JOG_AXIS_STATE_STOPPING,
} jog_axis_state_t;

typedef enum
{
    JOG_DIRECTION_NONE = JOG_DIR_NONE,
    JOG_DIRECTION_POSITIVE = JOG_DIR_POSITIVE,
    JOG_DIRECTION_NEGATIVE = JOG_DIR_NEGATIVE,
} jog_direction_t;

typedef enum
{
    /* 状态机只需要记住一个最近停止原因，报警原因还承担“复位解锁”职责。 */
    JOG_STOP_REASON_NONE = 0,
    JOG_STOP_REASON_RELEASE,
    JOG_STOP_REASON_LIMIT,
    JOG_STOP_REASON_MODE,
    JOG_STOP_REASON_ALARM,
    JOG_STOP_REASON_RESET,
    JOG_STOP_REASON_LOCK, // 机械锁住状态切换中，先停止手动运动
} jog_stop_reason_t;

typedef struct
{
    bool positive;
    bool negative;
} jog_key_latch_t;

typedef struct
{
    SYS_EVENT positive_press;
    SYS_EVENT positive_release;
    SYS_EVENT negative_press;
    SYS_EVENT negative_release;
} jog_key_event_map_t;

typedef struct
{
    jog_axis_state_t state;        /* 当前运动阶段 */
    jog_direction_t direction;     /* 当前/目标方向 */
    jog_key_latch_t key;           /* 两个方向键的保持状态 */
    jog_stop_reason_t stop_reason; /* 最近一次停止原因；报警会锁定重启 */
    jog_planner_t planner;         /* 该轴唯一的速度/位置规划器 */
} jog_axis_t;

typedef struct
{
    jog_axis_t axis[JOG_AXIS_COUNT];
    NC_MODE last_sys_mode; /* 用于检测重新进入 JOG 的首个周期 */
} jog_context_t;

static jog_context_t jog_context = {
    .last_sys_mode = MODE_IDLE,
};

static const jog_key_event_map_t jog_key_event_map[JOG_AXIS_COUNT] = {
    {SYS_EVENT_X_POS_PRESS, SYS_EVENT_X_POS_RELEASE, SYS_EVENT_X_NEG_PRESS, SYS_EVENT_X_NEG_RELEASE},
    {SYS_EVENT_Y_POS_PRESS, SYS_EVENT_Y_POS_RELEASE, SYS_EVENT_Y_NEG_PRESS, SYS_EVENT_Y_NEG_RELEASE},
    {SYS_EVENT_Z_POS_PRESS, SYS_EVENT_Z_POS_RELEASE, SYS_EVENT_Z_NEG_PRESS, SYS_EVENT_Z_NEG_RELEASE},
};

/* 内部函数声明 */
static void jog_reset_axis(jog_axis_t *axis, double position, bool sync_position);
static void jog_sync_on_mode_entry(void);
static void jog_update_key_latches(SYS_EVENT event);
static jog_direction_t jog_get_requested_direction(const jog_axis_t *axis);
static bool jog_is_global_fatal_alarm(void);
static bool jog_is_mac_lock_switching(void);
static void jog_apply_mac_lock(void);
static void jog_update_mac_lock(void);
static double jog_get_axis_acceleration(int axis, bool is_rapid);
static bool jog_is_hard_limit_active(int axis, jog_direction_t direction);
static bool jog_is_direction_blocked(int axis, jog_direction_t direction, double max_acc);
static int jog_get_direction_index(jog_direction_t direction);
/*
 * 状态迁移规则：
 *   IDLE -> RUNNING    有效按键且目标方向未被阻挡
 *   RUNNING -> STOPPING 松键、限位、切模或报警
 *   STOPPING -> IDLE   规划器已经停稳
 *   STOPPING -> RUNNING 停止过程中重新按下可行方向
 */
static void jog_transition_axis(jog_axis_t *axis, int axis_index, jog_direction_t desired_direction, bool global_fatal, bool reset_requested, bool switching_mode);
static double jog_get_target_velocity(int axis, const jog_axis_t *jog_axis, bool is_rapid);
static double jog_get_external_decel_velocity(int axis, const jog_axis_t *jog_axis, jog_direction_t direction);
static void jog_execute_axis(int axis, jog_axis_t *jog_axis, bool is_rapid);
static void jog_process_axis(int axis, bool is_rapid, bool global_fatal, bool reset_requested, bool switching_mode);
static void jog_update_outputs(void);

/* 基线里这个信号是 master.c 的 reset_sem：驱动器报警复位已发出、还没收口。
 * HAL 把它拆成了每轴的复位进展，这里逐轴查一遍。 */
static bool jog_fault_reset_in_progress(void)
{
    static const RtHalAxisSlot slots[] = {
        RT_HAL_AXIS_X, RT_HAL_AXIS_Y, RT_HAL_AXIS_Z, RT_HAL_AXIS_SP
    };
    unsigned i;

    for (i = 0; i < sizeof(slots) / sizeof(slots[0]); ++i)
    {
        int32_t state = HAL_RESET_NONE;
        if (rt_hal_fault_reset_state(slots[i], &state) == HAL_OK &&
            state == HAL_RESET_PENDING)
        {
            return true;
        }
    }

    return false;
}

/* 外部函数定义 */

/**
 * @brief JOG 任务主循环 (由伺服周期调用)
 */
void handle_jog_cmd(void)
{
    bool global_fatal;
    bool reset_requested;
    bool switching_mode;
    bool is_rapid;
    SYS_EVENT event;

    if (rt_data == NULL)
        return;

    if (rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        jog_handle_estop();
        return;
    }

    /* 1. 读取本周期输入，并处理模式进入。 */
    event = rt_data->nc_event;
    switching_mode = rt_data->sys_state[SYS_IS_SWITCHING_MODE];
    is_rapid = rt_data->sys_state[SYS_IS_RAPID];
    reset_requested = (event == SYS_EVENT_HMI_RESET || jog_fault_reset_in_progress());

    /* 机械锁住：点动模式下由本模块维护请求与生效状态，停稳后生效状态才跟随请求。 */
    jog_update_mac_lock();

    /* 切换模式期间不接受新的 JOG 运动请求，等速度降为零后再离开。 */
    if (switching_mode)
        jog_context.last_sys_mode = MODE_IDLE;

    if (jog_context.last_sys_mode != MODE_JOG && !switching_mode)
        jog_sync_on_mode_entry();

    /* 2. 将事件转换为按键保持状态，报警只在本周期计算一次。 */
    jog_update_key_latches(event);
    global_fatal = jog_is_global_fatal_alarm();

    /* 3. 每轴独立完成状态迁移和规划器更新。 */
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
        jog_process_axis(i, is_rapid, global_fatal, reset_requested, switching_mode);

    /* 4. 汇总轴输出，并在停止完成后消费本周期事件。 */
    jog_update_outputs();
}

/**
 * @brief 处理急停异常情况
 */
void jog_handle_estop(void)
{
    /* 急停直接清空所有运动状态；PLC 外部减速电平保存在 AXIS_STATUS 中。 */
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
        jog_reset_axis(&jog_context.axis[i], 0.0, false);

    if (rt_data != NULL)
    {
        for (int i = 0; i < JOG_AXIS_COUNT; i++)
        {
            /* handle_jog_cmd 在急停分支会立即返回，因此这里同步清零对外速度。 */
            rt_data->axis_data[i].cmd_vel = 0.0;
        }
        rt_data->chan_data[0].cmd_vel = 0.0;
    }
    jog_context.last_sys_mode = MODE_IDLE;
}

double jog_get_axis_velocity(int axis)
{
    if (axis < 0 || axis >= JOG_AXIS_COUNT)
        return 0.0;
    return jog_context.axis[axis].planner.curr_vel;
}

/* 内部函数定义 */

/* 清除一轴运动状态；sync_position 只在模式进入时用于对齐当前位置。 */
static void jog_reset_axis(jog_axis_t *axis, double position, bool sync_position)
{
    if (axis == NULL)
        return;

    axis->state = JOG_AXIS_STATE_IDLE;
    axis->direction = JOG_DIRECTION_NONE;
    axis->key.positive = false;
    axis->key.negative = false;
    axis->stop_reason = JOG_STOP_REASON_NONE;

    memset(&axis->planner, 0, sizeof(axis->planner));
    if (sync_position)
        axis->planner.curr_pos = position;
}

static void jog_sync_on_mode_entry(void)
{
    /* 进入 JOG 时从当前指令位置重新开始，避免继承其他模式的规划器状态。 */
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
    {
        bool alarm_latched;

        /* 模式切换只重置规划器，不能绕过非限位报警的 RESET 解锁要求。 */
        alarm_latched = (jog_context.axis[i].stop_reason == JOG_STOP_REASON_ALARM);
        jog_reset_axis(&jog_context.axis[i], rt_data->axis_data[i].wcs_cmd_pos_cur, true);
        if (alarm_latched)
            jog_context.axis[i].stop_reason = JOG_STOP_REASON_ALARM;
    }
    jog_context.last_sys_mode = MODE_JOG;
}

/*
 * 机械锁住请求与实际生效状态不一致：正在锁住或正在解除。
 * 此期间由 jog_transition_axis() 的切换分支让点动先按斜坡停稳；
 * 全轴空闲后由 jog_update_outputs() 落 READY。
 * 注意：不清理按键锁存，所以切换完成后若键仍按着，点动会继续。
 */
static bool jog_is_mac_lock_switching(void)
{
    return rt_data->sys_state[SYS_IS_MAC_LOCK_ON] != rt_data->sys_state[SYS_IS_MAC_LOCK_READY];
}

/*
 * 机械锁住：请求(ON) 与 生效(READY) 分开，写法与自动模式保持一致。
 *
 *   ON=0,READY=0  未锁住
 *   ON=1,READY=0  已请求锁住：点动先按斜坡停稳（jog_transition_axis() 的切换分支
 *                 让本轴不再接受新运动 → STOPPING），等全轴空闲才生效
 *   ON=1,READY=1  锁住生效：calcul_mac_cmd_pos() 不再累加机床指令位置 → 机床不动
 *   ON=0,READY=1  请求解除：先把（虚拟）运动按同样的斜坡停稳
 *   ON=0,READY=0  解除完成：机床链从冻结值继续累加，账面与机床的差值保留
 *
 * 按键锁存全程保留，所以切换完成后若键还按着，点动会继续。
 */
static void jog_apply_mac_lock(void)
{
    bool lock_request;

    lock_request = rt_data->sys_state[SYS_IS_MAC_LOCK_ON];
    /* 只有真的发生变化才打日志，避免全轴空闲时每周期刷屏；赋值本身与 auto 一样是无条件的 */
    if (rt_data->sys_state[SYS_IS_MAC_LOCK_READY] != lock_request)
    {
        LOG(LOG_LEVEL_INFO, "手动模式机械锁住%s", lock_request ? "生效" : "解除");
    }
    rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = lock_request;
}

/* 机械锁住开关事件：只改“请求”，生效状态交给“全轴空闲”的分支去落 */
static void jog_update_mac_lock(void)
{
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_MAC_LOCK_ON:
        LOG(LOG_LEVEL_INFO, "手动模式处理开启机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = true;
        break;
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_INFO, "手动模式处理关闭机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = false;
        break;
    default:
        break;
    }
}

static void jog_update_key_latches(SYS_EVENT event)
{
    /* 一个周期只处理一个 nc_event；非 JOG 事件直接忽略，保持上一周期电平。 */
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
    {
        if (event == jog_key_event_map[i].positive_press)
            jog_context.axis[i].key.positive = true;
        else if (event == jog_key_event_map[i].positive_release)
            jog_context.axis[i].key.positive = false;
        else if (event == jog_key_event_map[i].negative_press)
            jog_context.axis[i].key.negative = true;
        else if (event == jog_key_event_map[i].negative_release)
            jog_context.axis[i].key.negative = false;
    }
}

/* 将两个按键保持状态转换为单一方向；双按和全松开均视为无方向。 */
static jog_direction_t jog_get_requested_direction(const jog_axis_t *axis)
{
    if (axis == NULL || axis->key.positive == axis->key.negative)
        return JOG_DIRECTION_NONE;
    return axis->key.positive ? JOG_DIRECTION_POSITIVE : JOG_DIRECTION_NEGATIVE;
}

/* 判断是否为需要所有 JOG 轴停止的报警。纯限位报警留给对应轴处理。 */
static bool jog_is_global_fatal_alarm(void)
{
    bool has_limit_alarm;

    if (!rt_data->sys_state[SYS_IS_ALARMING])
        return false;

    has_limit_alarm = false;
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
    {
        AXIS_STATUS *status;

        status = &rt_data->axis_data[i].status;
        /* 伺服错误和跟随误差需要所有 JOG 轴停止，并由 RESET 解锁。 */
        if (status->is_error || status->is_lag_error)
            return true;

        if (status->is_fwd_limit || status->is_rev_limit)
            has_limit_alarm = true;
    }

    /* 只有明确的纯限位报警允许各轴独立处理；其他报警默认按全局报警处理。 */
    if (has_limit_alarm)
        return false;
    return true;
}

/* 读取当前 JOG 模式的加速度，并将无效参数钳制为 0。 */
static double jog_get_axis_acceleration(int axis, bool is_rapid)
{
    double acceleration;

    if (axis < 0 || axis >= JOG_AXIS_COUNT)
        return 0.0;
    acceleration = is_rapid ? rt_data->axis_data[axis].max_rapid_acc : rt_data->axis_data[axis].max_feed_acc;
    if (!isfinite(acceleration) || acceleration < 0.0)
        return 0.0;
    return acceleration;
}

/* 查询指定方向的硬限位；反方向硬限位不会阻挡当前运动。 */
static bool jog_is_hard_limit_active(int axis, jog_direction_t direction)
{
    if (axis < 0 || axis >= JOG_AXIS_COUNT)
        return false;
    if (direction == JOG_DIRECTION_POSITIVE)
        return rt_data->axis_data[axis].status.is_fwd_limit;
    if (direction == JOG_DIRECTION_NEGATIVE)
        return rt_data->axis_data[axis].status.is_rev_limit;
    return false;
}

/* 将方向映射到外部减速矩阵的第二维。 */
static int jog_get_direction_index(jog_direction_t direction)
{
    if (direction == JOG_DIRECTION_POSITIVE)
        return 0;
    if (direction == JOG_DIRECTION_NEGATIVE)
        return 1;
    return -1;
}

/* 判断请求方向是否受硬限位、软限位前瞻或无效加速度约束。 */
static bool jog_is_direction_blocked(int axis, jog_direction_t direction, double max_acc)
{
    double current_velocity;
    double stop_distance;
    double positive_limit;
    double negative_limit;
    bool hard_limit;

    if (axis < 0 || axis >= JOG_AXIS_COUNT || direction == JOG_DIRECTION_NONE)
        return false;

    hard_limit = jog_is_hard_limit_active(axis, direction);
    if (hard_limit)
        return true;

    positive_limit = rt_data->axis_data[axis].plmt;
    negative_limit = rt_data->axis_data[axis].nlmt;
    /* 没有有效加速度时不能建立速度斜坡，保守地禁止新的运动请求。 */
    if (!isfinite(max_acc) || max_acc <= 0.0)
        return true;
    if (positive_limit <= negative_limit)
        return false;

    current_velocity = jog_context.axis[axis].planner.curr_vel;
    /* 预估按当前速度刹停所需距离，提前禁止会来不及刹停的方向。 */
    stop_distance = (current_velocity * current_velocity) / (2.0 * max_acc) * JOG_BRAKE_FACTOR;
    if (direction == JOG_DIRECTION_POSITIVE)
        return (jog_context.axis[axis].planner.curr_pos + stop_distance) >= positive_limit;
    return (jog_context.axis[axis].planner.curr_pos - stop_distance) <= negative_limit;
}

static void jog_transition_axis(jog_axis_t *axis, int axis_index, jog_direction_t desired_direction, bool global_fatal, bool reset_requested, bool switching_mode)
{
    bool direction_blocked;
    bool current_direction_hard_blocked;
    bool same_direction_limit_stop;

    if (axis == NULL)
        return;

    direction_blocked = jog_is_direction_blocked(axis_index, desired_direction, axis->planner.max_acc);
    same_direction_limit_stop = (axis->stop_reason == JOG_STOP_REASON_LIMIT && desired_direction == axis->direction);

    /* 优先级：复位、全局报警、报警复位锁存，然后才处理模式/机械锁住/按键/限位。 */
    if (reset_requested)
    {
        axis->key.positive = false;
        axis->key.negative = false;
        axis->stop_reason = JOG_STOP_REASON_RESET;
        axis->state = JOG_AXIS_STATE_STOPPING;
        return;
    }

    if (global_fatal)
    {
        /* 即使轴已经停稳，也要保留报警原因，防止报警消失后直接重启。 */
        axis->stop_reason = JOG_STOP_REASON_ALARM;
        if (axis->state != JOG_AXIS_STATE_IDLE)
            axis->state = JOG_AXIS_STATE_STOPPING;
        return;
    }

    if (axis->stop_reason == JOG_STOP_REASON_ALARM)
    {
        /* 非限位报警解除后仍保持锁定，必须收到 HMI_RESET 才能再次 JOG。 */
        return;
    }

    if (switching_mode)
    {
        axis->stop_reason = JOG_STOP_REASON_MODE;
        axis->state = JOG_AXIS_STATE_STOPPING;
        return;
    }

    /*
     * 机械锁住状态切换（ON != READY）：先让本轴按斜坡停稳，停稳后
     * 全轴空闲后由 jog_update_outputs() 落生效状态。按键锁存保留 —— 切换完成后
     * 若键还按着，下一周期就会恢复 RUNNING（锁住期间只走账面）。
     */
    if (jog_is_mac_lock_switching())
    {
        if (axis->state != JOG_AXIS_STATE_IDLE)
        {
            axis->stop_reason = JOG_STOP_REASON_LOCK;
            axis->state = JOG_AXIS_STATE_STOPPING;
        }
        return;
    }

    /* 当前方向只检查硬限位；软限位通过 desired_direction 的阻挡处理，允许换向退出。 */
    current_direction_hard_blocked = jog_is_hard_limit_active(axis_index, axis->direction);
    switch (axis->state)
    {
    case JOG_AXIS_STATE_IDLE:
        if (desired_direction != JOG_DIRECTION_NONE && !direction_blocked)
        {
            axis->state = JOG_AXIS_STATE_RUNNING;
            axis->direction = desired_direction;
            axis->stop_reason = JOG_STOP_REASON_NONE;
        }
        break;

    case JOG_AXIS_STATE_RUNNING:
        if (current_direction_hard_blocked)
        {
            axis->stop_reason = JOG_STOP_REASON_LIMIT;
            axis->state = JOG_AXIS_STATE_STOPPING;
        }
        else if (desired_direction != JOG_DIRECTION_NONE && !direction_blocked)
        {
            /* 运行中允许直接换向，规划器会先按当前加速度减速再反向。 */
            axis->direction = desired_direction;
            axis->stop_reason = JOG_STOP_REASON_NONE;
        }
        else
        {
            axis->stop_reason = (desired_direction == JOG_DIRECTION_NONE) ? JOG_STOP_REASON_RELEASE : JOG_STOP_REASON_LIMIT;
            axis->state = JOG_AXIS_STATE_STOPPING;
        }
        break;

    case JOG_AXIS_STATE_STOPPING:
        /* 限位停止后必须先换向；松键后重新按同方向仍可再次点动。 */
        if (desired_direction != JOG_DIRECTION_NONE && !same_direction_limit_stop && !direction_blocked)
        {
            /* 停止尚未完成时重新按键，恢复运行而不丢失规划器当前速度。 */
            axis->state = JOG_AXIS_STATE_RUNNING;
            axis->direction = desired_direction;
            axis->stop_reason = JOG_STOP_REASON_NONE;
        }
        break;

    default:
        axis->state = JOG_AXIS_STATE_IDLE;
        axis->direction = JOG_DIRECTION_NONE;
        axis->stop_reason = JOG_STOP_REASON_NONE;
        break;
    }
}

/* 计算最终目标速度：基础 JOG 速度乘倍率，再叠加同轴同向外部减速上限。 */
static double jog_get_target_velocity(int axis, const jog_axis_t *jog_axis, bool is_rapid)
{
    double base_velocity;
    double override;
    double target_velocity;
    double target_speed;
    double external_limit;

    if (jog_axis == NULL)
        return 0.0;
    if (is_rapid)
    {
        base_velocity = rt_data->axis_data[axis].jog_fastspd;
        override = var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / 100.0;
    }
    else
    {
        base_velocity = rt_data->axis_data[axis].jog_lowspd;
        override = var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / 100.0;
    }
    /* SYS_IS_RAPID 只决定基础 JOG 速度/倍率；外部减速统一使用 rpd_ex_decN_vel。 */
    target_velocity = base_velocity * override * (double)jog_axis->direction;
    target_speed = fabs(target_velocity);
    external_limit = jog_get_external_decel_velocity(axis, jog_axis, jog_axis->direction);
    if (external_limit < target_speed)
        target_speed = external_limit;
    return copysign(target_speed, target_velocity);
}

/* 返回有效外部减速中的最小速度；没有信号时返回无穷大表示“不限速”。 */
static double jog_get_external_decel_velocity(int axis, const jog_axis_t *jog_axis, jog_direction_t direction)
{
    bool function_enabled[SYS_EXTERNAL_DECEL_FUNCTION_COUNT];
    double configured_velocity[SYS_EXTERNAL_DECEL_FUNCTION_COUNT];
    double limit;
    int direction_index;

    if (jog_axis == NULL || !rt_data->chan_data[0].ex_dec_enable)
        return HUGE_VAL;

    direction_index = jog_get_direction_index(direction);
    if (direction_index < 0)
        return HUGE_VAL;

    /* 下标 0~4 分别对应外部减速功能 1~5。 */
    function_enabled[0] = true; /* 功能 1 没有独立开关。 */
    function_enabled[1] = rt_data->chan_data[0].ex_dec2_enable;
    function_enabled[2] = rt_data->chan_data[0].ex_dec3_enable;
    function_enabled[3] = rt_data->chan_data[0].ex_dec4_enable;
    function_enabled[4] = rt_data->chan_data[0].ex_dec5_enable;
    configured_velocity[0] = rt_data->axis_data[axis].rpd_ex_dec1_vel;
    configured_velocity[1] = rt_data->axis_data[axis].rpd_ex_dec2_vel;
    configured_velocity[2] = rt_data->axis_data[axis].rpd_ex_dec3_vel;
    configured_velocity[3] = rt_data->axis_data[axis].rpd_ex_dec4_vel;
    configured_velocity[4] = rt_data->axis_data[axis].rpd_ex_dec5_vel;

    limit = HUGE_VAL;
    for (int function = 0; function < SYS_EXTERNAL_DECEL_FUNCTION_COUNT; function++)
    {
        if (!rt_data->axis_data[axis].status.external_decel_active[function][direction_index] || !function_enabled[function])
            continue;

        if (!isfinite(configured_velocity[function]) || configured_velocity[function] < 0.0)
            configured_velocity[function] = 0.0;
        if (configured_velocity[function] < limit)
            limit = configured_velocity[function];
    }
    return limit;
}

/* 根据状态设置规划器目标，并推进一个伺服周期。 */
static void jog_execute_axis(int axis, jog_axis_t *jog_axis, bool is_rapid)
{
    if (!isfinite(jog_axis->planner.max_acc) || jog_axis->planner.max_acc <= 0.0)
    {
        /* 规划器无法在无效加速度下减速；清零输出，避免轴保持在不可停止状态。 */
        jog_axis->planner.target_vel = 0.0;
        jog_axis->planner.enable = 0;
        jog_axis->planner.curr_vel = 0.0;
        jog_axis->planner.active = 0;
        jog_axis->state = JOG_AXIS_STATE_IDLE;
        jog_axis->direction = JOG_DIRECTION_NONE;
        if (jog_axis->stop_reason != JOG_STOP_REASON_ALARM)
            jog_axis->stop_reason = JOG_STOP_REASON_NONE;
        return;
    }

    switch (jog_axis->state)
    {
    case JOG_AXIS_STATE_RUNNING:
        jog_axis->planner.enable = 1;
        jog_axis->planner.target_vel = jog_get_target_velocity(axis, jog_axis, is_rapid);
        break;

    case JOG_AXIS_STATE_STOPPING:
        /* 保持 enable，让规划器按正常加速度斜坡减到 0。 */
        jog_axis->planner.target_vel = 0.0;
        jog_axis->planner.enable = 1;
        if (!jog_axis->planner.active)
        {
            jog_axis->state = JOG_AXIS_STATE_IDLE;
            jog_axis->direction = JOG_DIRECTION_NONE;
            /* 报警原因必须保留到 RESET；其他停止原因在停稳后即可清除。 */
            if (jog_axis->stop_reason != JOG_STOP_REASON_ALARM)
                jog_axis->stop_reason = JOG_STOP_REASON_NONE;
            jog_axis->planner.enable = 0;
        }
        break;

    case JOG_AXIS_STATE_IDLE:
        jog_axis->planner.enable = 0;
        jog_axis->planner.target_vel = 0.0;
        break;

    default:
        jog_axis->state = JOG_AXIS_STATE_IDLE;
        jog_axis->direction = JOG_DIRECTION_NONE;
        jog_axis->planner.enable = 0;
        jog_axis->planner.target_vel = 0.0;
        break;
    }
    jog_planner_update(&jog_axis->planner, rt_data->ncu_data.cycle);
}

/* 执行单轴的完整周期：同步、解析请求、迁移状态、更新规划器和输出。 */
static void jog_process_axis(int axis, bool is_rapid, bool global_fatal, bool reset_requested, bool switching_mode)
{
    jog_axis_t *jog_axis;
    jog_direction_t desired_direction;

    jog_axis = &jog_context.axis[axis];

    /* 空闲轴持续同步当前位置，避免从其他模式切回 JOG 时产生位置跳变。 */
    if (jog_axis->state == JOG_AXIS_STATE_IDLE)
    {
        jog_axis->planner.curr_pos = rt_data->axis_data[axis].wcs_cmd_pos_cur;
        jog_axis->planner.curr_vel = 0.0;
        jog_axis->planner.enable = 0;
    }

    desired_direction = jog_get_requested_direction(jog_axis);

    /* 软限位预估和速度规划必须使用同一个加速度。 */
    jog_axis->planner.max_acc = jog_get_axis_acceleration(axis, is_rapid);
    jog_transition_axis(jog_axis, axis, desired_direction, global_fatal, reset_requested, switching_mode);
    jog_execute_axis(axis, jog_axis, is_rapid);

    /* 规划器是 JOG 的唯一运动输出源。 */
    rt_data->axis_data[axis].wcs_cmd_pos_cur = jog_axis->planner.curr_pos;
    rt_data->axis_data[axis].cmd_vel = jog_axis->planner.curr_vel;
}

/* 汇总通道速度、限位减速完成标志；全轴空闲时落机械锁住状态并消费事件。 */
static void jog_update_outputs(void)
{
    double feed_velocity_squared;
    bool all_axes_idle;

    feed_velocity_squared = 0.0;
    all_axes_idle = true;
    for (int i = 0; i < JOG_AXIS_COUNT; i++)
    {
        double axis_velocity;

        axis_velocity = jog_context.axis[i].planner.curr_vel;
        feed_velocity_squared += axis_velocity * axis_velocity;

        /* 只有限位触发导致的停止才置减速完成标志。 */
        if (fabs(axis_velocity) < JOG_SPEED_THRESHOLD && (rt_data->axis_data[i].status.is_fwd_limit || rt_data->axis_data[i].status.is_rev_limit))
        {
            rt_data->axis_data[i].status.is_decel_done = 1;
        }

        if (jog_context.axis[i].state != JOG_AXIS_STATE_IDLE)
            all_axes_idle = false;
    }

    /* 通道速度是各轴速度的矢量合成，不是各轴速度的代数和。 */
    rt_data->chan_data[0].cmd_vel = sqrt(feed_velocity_squared);
    if (all_axes_idle)
    {
        /* 全轴空闲：与 auto_update_stage() 的“没有运动指令”分支对应 ——
           在这里让机械锁住的生效状态跟随请求（赋值本身无条件，不在这里做条件组合判断） */
        jog_apply_mac_lock();
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}
