#include "jog.h"
#include "global.h"
#include "ferror_control.h" // 必须引入以获取具体的跟随误差状态
#include <math.h>
#include <string.h>

/* JOG规划器实例 */
jog_planner_t jog_tp[SYS_AXES_NUM_JOG];

/* 控制状态初始化 */
static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}};

/* 在 jog.c 的文件头部定义 */
static NC_MODE last_sys_mode = MODE_IDLE; // 记录进入当前模式前，上一次的操作模式

/* 事件-轴号映射查找表：用轴号 i 直接索引对应的 SYS_EVENT */
static const SYS_EVENT pos_events[SYS_AXES_NUM_JOG] = {SYS_EVENT_X_POS_PRESS, SYS_EVENT_Y_POS_PRESS, SYS_EVENT_Z_POS_PRESS};
static const SYS_EVENT neg_events[SYS_AXES_NUM_JOG] = {SYS_EVENT_X_NEG_PRESS, SYS_EVENT_Y_NEG_PRESS, SYS_EVENT_Z_NEG_PRESS};
static const SYS_EVENT rel_events[SYS_AXES_NUM_JOG] = {SYS_EVENT_X_RELEASE, SYS_EVENT_Y_RELEASE, SYS_EVENT_Z_RELEASE};

/* 内部私有函数声明 */
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl, bool global_fatal);
static void execute_axis_motion(int i, JOG_AXIS_CTRL *jog_axis_ctrl, jog_planner_t *axis_jog_tp, const bool is_rapid);
static void sync_jog_on_mode_entry();
static bool check_all_axes_idle_jog();

/**
 * @brief JOG 任务主循环 (由 1ms 伺服中断调用)
 */
void handle_jog_cmd()
{
    // 内部急停标志触发，立即停止规划，复位规划器
    if (rt_data->sys_state[SYS_IS_ESTOP_ON])
    {
        jog_handle_estop();
        return;
    }

    // 模式切换中：重置进入标志，确保下次切回 JOG 时重新同步规划器位置
    if (rt_data->sys_state[SYS_IS_SWITCHING_MODE])
        last_sys_mode = MODE_IDLE;
    // 模式切换完成后首次进入 JOG：同步规划器坐标到当前物理位置
    if (last_sys_mode != MODE_JOG && !rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        sync_jog_on_mode_entry();
        last_sys_mode = MODE_JOG;
    }

    /* 报警分类预计算：每周期一次，判定是否为全局致命报警。
       - 仅当报警纯粹由限位开关引起时，不致命，放行各轴自行处理（X轴限位不影响Y/Z轴）。
       - 其余一切情况（非限位报警、限位+非限位混合、系统级报警）均为致命，三轴全停。

       SYS_IS_ALARMING 为真时：
         non_limit   pure_limit   →  global_fatal
         ✓           任意          →  true   非限位报警
         ✗           ✓            →  false  纯限位报警（放行）
         ✗           ✗            →  true   系统级报警（无限位标志） */
    bool global_fatal = false;
    if (rt_data->sys_state[SYS_IS_ALARMING])
    {
        bool any_non_limit = false;
        bool any_pure_limit = false;
        for (int j = 0; j < SYS_AXES_NUM_JOG; j++)
        {
            if (rt_data->axis_data[j].axis_status.is_error || ferror_control[j].error_flag)
                any_non_limit = true;
            else if (rt_data->axis_data[j].axis_status.is_fwd_limit ||
                     rt_data->axis_data[j].axis_status.is_rev_limit)
                any_pure_limit = true;
        }
        global_fatal = (any_non_limit || !any_pure_limit);
    }

    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        /* 1. 静态同步：IDLE状态下强制同步位置，防止从JOG切入其他模式时位置突跳 */
        if (jog_ctrls[i].state == JOG_STATE_IDLE)
        {
            jog_tp[i].curr_pos = rt_data->axis_data[i].wcs_cmd_pos_cur;
            jog_tp[i].curr_vel = 0.0;
            jog_tp[i].enable = 0;
        }

        /* 2. 处理按键信号与状态机切换 */
        update_axis_state(i, &jog_ctrls[i], global_fatal);

        /* 3. 执行JOG运动计算逻辑 */
        execute_axis_motion(i, &jog_ctrls[i], &jog_tp[i], rt_data->sys_state[SYS_IS_RAPID]);

        /* 4. 指令输出给外部系统变量 */
        rt_data->axis_data[i].wcs_cmd_pos_cur = jog_tp[i].curr_pos;
        var_axis[i][VAR_AXIS_CMD_VEL].value.f = jog_tp[i].curr_vel;
    }

    /* 计算合成进给速度并更新给人机交互显示 */
    double f_sq = 0;
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        f_sq += (jog_tp[i].curr_vel * jog_tp[i].curr_vel);
    }
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, sqrt(f_sq) * 60.0, 0);

    // 减速完成信号：仅在限位触发且速度归零时置位，否则清零
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (fabs(var_axis[i][VAR_AXIS_CMD_VEL].value.f) < SPEED_THRESHOLD &&
            (rt_data->axis_data[i].axis_status.is_fwd_limit || rt_data->axis_data[i].axis_status.is_rev_limit))
        {
            rt_data->axis_data[i].axis_status.is_decel_done = 1;
        }
        // else
        // {
        //     rt_data->axis_data[i].axis_status.is_decel_done = 0;
        // }
    }

    // 无动作时将通道事件全局变量置为NONE事件
    if (check_all_axes_idle_jog())
    {
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}

/**
 * @brief 处理急停异常情况
 */
void jog_handle_estop()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        // JOG急停复位
        jog_ctrls[i].state = JOG_STATE_IDLE;
        jog_ctrls[i].direction = JOG_DIR_NONE;
        memset(&jog_tp[i], 0, sizeof(jog_planner_t));
    }
}

/**
 * @brief 根据按键事件更新状态机
 */
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl, bool global_fatal)
{
    SYS_EVENT event = rt_data->nc_event;

    /* 获取当前轴的限位状态 */
    bool is_fwd_limit = rt_data->axis_data[i].axis_status.is_fwd_limit;
    bool is_rev_limit = rt_data->axis_data[i].axis_status.is_rev_limit;
    bool is_jog_pos_req = rt_data->axis_data[i].axis_status.is_jog_pos_req;
    bool is_jog_neg_req = rt_data->axis_data[i].axis_status.is_jog_neg_req;

    /* 查表解析当前轴的按键意图方向 */
    bool press_pos = (event == pos_events[i]);
    bool press_neg = (event == neg_events[i]);
    bool is_release = (event == rel_events[i]);

    /* 1. 处理复位,切模式：强制停止所有运动 */
    if (event == SYS_EVENT_RESET || rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 2. 方向互锁：同一轴正负两个方向同时有效 -> 禁止运动 (工业安全标准) */
    if (is_jog_pos_req && is_jog_neg_req)
    {
        if (jog_axis_ctrl->state != JOG_STATE_IDLE)
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 3. 全局致命报警拦截（由调用方每周期预计算一次） */
    if (global_fatal)
    {
        if (jog_axis_ctrl->state != JOG_STATE_IDLE)
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 4. 状态机切换逻辑 */
    switch (jog_axis_ctrl->state)
    {
    case JOG_STATE_IDLE:
    {
        // 捕捉【正向按键按下】
        if (press_pos && !is_fwd_limit)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = JOG_DIR_POSITIVE;
        }
        // 捕捉【负向按键按下】
        else if (press_neg && !is_rev_limit)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = JOG_DIR_NEGATIVE;
        }
        break;
    }
    case JOG_STATE_RUNNING:
    {
        /* 运行中如果后台限位状态突然变为触发，立即刹车 */
        /* 情况 A: 撞到了运动方向的限位 -> 停止
         * 情况 B: 运动过程中发生了严重报警 (由步骤 4 拦截，此处作为冗余保护)*/
        if ((jog_axis_ctrl->direction == JOG_DIR_POSITIVE && is_fwd_limit) ||
            (jog_axis_ctrl->direction == JOG_DIR_NEGATIVE && is_rev_limit))
        {
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
            break;
        }

        /* 捕捉运行过程中的反向按键（或重复按下），直接切换目标方向 */
        /* 规划器会自动处理减速过零并反向加速的平滑过渡 */
        if (press_pos && !is_fwd_limit)
        {
            jog_axis_ctrl->direction = JOG_DIR_POSITIVE;
        }
        else if (press_neg && !is_rev_limit)
        {
            jog_axis_ctrl->direction = JOG_DIR_NEGATIVE;
        }

        /* 2. 捕捉按键释放（核心修复：只响应与当前运行方向匹配的释放事件） */
        if (is_release)
        {
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
        }
        break;
    }

    case JOG_STATE_STOPPING:
    {
        /* 核心逻辑：在减速过程中，如果按键再次按下，立即恢复 RUNNING */
        // 捕捉【正向按键再次按下】，且未触发正向限位才允许运行
        if (press_pos && !is_fwd_limit)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = JOG_DIR_POSITIVE;
        }
        // 捕捉【负向按键再次按下】，且未触发负向限位才允许运行
        else if (press_neg && !is_rev_limit)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = JOG_DIR_NEGATIVE;
        }
        break;
    }
    }
}

/**
 * @brief 轴运动执行逻辑 (带前瞻软限位拦截)
 * 根据快速标志位选择 运动类型：快速JOG/普通JOG。
 */
static void execute_axis_motion(int i, JOG_AXIS_CTRL *jog_axis_ctrl, jog_planner_t *axis_jog_tp, bool is_rapid)
{
    switch (jog_axis_ctrl->state)
    {
    case JOG_STATE_RUNNING:
    {
        /* 使能规划器 */
        axis_jog_tp->enable = 1; // 保持使能开启

        /* 1. 更新运动参数 */
        // 根据模式选择速度与加速度参数  普通jog：1）慢速点动速度 2）加工加速度；  快速：1）快速点动速度 2）快速加速度
        double base_vel = is_rapid ? par_axis[i][PAR_AX_JOG_FASTSPD].value.val_real : par_axis[i][PAR_AX_JOG_LOWSPD].value.val_real;
        double base_acc = is_rapid ? par_axis[i][PAR_AX_RAPID_ACC].value.val_real : par_axis[i][PAR_AX_FEED_ACC].value.val_real;
        // 设置目标速度：实时获取倍率，设定带方向的目标速度
        double current_override = is_rapid ? (var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / 100.0) : (var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / 100.0);
        axis_jog_tp->target_vel = base_vel * current_override * jog_axis_ctrl->direction;
        axis_jog_tp->max_acc = base_acc;

        /* 2. 软限位前瞻拦截预测 */
        double p_limit = par_axis[i][PAR_AX_PLMT].value.val_real;
        double n_limit = par_axis[i][PAR_AX_NLMT].value.val_real;

        // 兼容工业规范：如果正限位严格大于负限位，视为有限位直线轴。
        // 如果正负限位设置相等(如全是0)，则视为无限旋转轴，跳过限位检查。
        if (p_limit > n_limit)
        {
            double curr_v = axis_jog_tp->curr_vel;
            double decel = axis_jog_tp->max_acc;

            // 计算理论刹车距离，乘以 1.05 作为安全余量，防止浮点累加误差导致轻微越界
            // 减小系数，就可以超过限位
            double stop_dist = (curr_v * curr_v) / (2.0 * decel) * 0.9999;

            bool is_limit = false;
            if (jog_axis_ctrl->direction == JOG_DIR_POSITIVE)
            {
                is_limit = ((axis_jog_tp->curr_pos + stop_dist) >= p_limit);
            }
            else if (jog_axis_ctrl->direction == JOG_DIR_NEGATIVE)
            {
                is_limit = ((axis_jog_tp->curr_pos - stop_dist) <= n_limit);
            }
            // 超出限位，则降速
            if (is_limit)
            {
                jog_axis_ctrl->state = JOG_STATE_STOPPING;
            }
        }
        break;
    }

    case JOG_STATE_STOPPING:
    {
        // 注意：这里不要设 enable = 0，而是让 target_vel = 0
        axis_jog_tp->target_vel = 0.0;
        axis_jog_tp->enable = 1;

        /* 检查是否完全停稳 */
        if (!axis_jog_tp->active)
        {
            jog_axis_ctrl->state = JOG_STATE_IDLE;
            jog_axis_ctrl->direction = JOG_DIR_NONE;
            axis_jog_tp->enable = 0;
        }
        break;
    }
    case JOG_STATE_IDLE:
    {
        axis_jog_tp->enable = 0;
        axis_jog_tp->target_vel = 0.0;
        break;
    }
    default:
        break;
    }
    // 统一调用更新
    jog_planner_update(axis_jog_tp, JOG_CYCLE_TIME);
}

/**
 * @brief 内部同步函数：当检测到模式切换进入点动时，强制对齐所有状态
 */
static void sync_jog_on_mode_entry()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        double actual_pos = rt_data->axis_data[i].wcs_cmd_pos_cur;
        jog_ctrls[i].state = JOG_STATE_IDLE;
        jog_ctrls[i].direction = JOG_DIR_NONE;
        jog_tp[i].curr_pos = actual_pos;
        jog_tp[i].curr_vel = 0.0;
        jog_tp[i].enable = 0;
    }
}

/**
 * @brief jog模式下 检查所有轴是否都处于静止空闲状态
 */
static bool check_all_axes_idle_jog()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (jog_ctrls[i].state != JOG_STATE_IDLE)
        {
            return false;
        }
    }
    return true;
}
