#include "jog.h"
#include "ferror_control.h" // 必须引入以获取具体的跟随误差状态
#include "global.h"
#include <math.h>
#include <string.h>

/* JOG规划器实例 */
jog_planner_t jog_tp[SYS_AXES_NUM_JOG];

/* 控制状态初始化 */
static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}
};

/* 在 jog.c 的文件头部定义 */
static NC_MODE last_sys_mode = MODE_IDLE; // 记录进入当前模式前，上一次的操作模式

/* 本地按键保持状态 -- 由 nc_event 边沿更新，替代原先 rt_data 中的 is_jog_pos_req / is_jog_neg_req
 */
static bool jog_pos_held[SYS_AXES_NUM_JOG];
static bool jog_neg_held[SYS_AXES_NUM_JOG];

/* 内部私有函数声明 */
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl, bool global_fatal);
static void execute_axis_motion(
    int i,
    JOG_AXIS_CTRL *jog_axis_ctrl,
    jog_planner_t *axis_jog_tp,
    const bool is_rapid
);
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
            else if (
                rt_data->axis_data[j].axis_status.is_fwd_limit ||
                rt_data->axis_data[j].axis_status.is_rev_limit
            )
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
            (rt_data->axis_data[i].axis_status.is_fwd_limit ||
             rt_data->axis_data[i].axis_status.is_rev_limit))
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
        /* 清除本地保持状态 */
        jog_pos_held[i] = false;
        jog_neg_held[i] = false;
    }
}

/**
 * @brief 根据 nc_event 边沿更新本地保持状态，并驱动 JOG 状态机
 *
 * 状态机完全由 nc_event 边沿事件驱动，不再依赖 rt_data 中的电平标志。
 * 本函数将 nc_event 边沿转换为本地保持状态（jog_pos_held / jog_neg_held），
 * 供状态机作为电平信号消费。
 *
 * 行为规则：
 * - 仅正向按键按住 → 正向运动
 * - 仅负向按键按住 → 负向运动
 * - 正负同时按住  → 停止（工业安全互锁）
 * - 正向按住时松开负向 → 继续正向运动（不受负向释放影响）
 * - 全松开 → 停止
 */
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl, bool global_fatal)
{
    SYS_EVENT event = rt_data->nc_event;

    /* 0. nc_event 边沿 → 本地保持状态 */
    switch (i)
    {
    case 0:
        if (event == SYS_EVENT_X_POS_PRESS)
            jog_pos_held[0] = true;
        else if (event == SYS_EVENT_X_POS_RELEASE)
            jog_pos_held[0] = false;
        else if (event == SYS_EVENT_X_NEG_PRESS)
            jog_neg_held[0] = true;
        else if (event == SYS_EVENT_X_NEG_RELEASE)
            jog_neg_held[0] = false;
        break;
    case 1:
        if (event == SYS_EVENT_Y_POS_PRESS)
            jog_pos_held[1] = true;
        else if (event == SYS_EVENT_Y_POS_RELEASE)
            jog_pos_held[1] = false;
        else if (event == SYS_EVENT_Y_NEG_PRESS)
            jog_neg_held[1] = true;
        else if (event == SYS_EVENT_Y_NEG_RELEASE)
            jog_neg_held[1] = false;
        break;
    case 2:
        if (event == SYS_EVENT_Z_POS_PRESS)
            jog_pos_held[2] = true;
        else if (event == SYS_EVENT_Z_POS_RELEASE)
            jog_pos_held[2] = false;
        else if (event == SYS_EVENT_Z_NEG_PRESS)
            jog_neg_held[2] = true;
        else if (event == SYS_EVENT_Z_NEG_RELEASE)
            jog_neg_held[2] = false;
        break;
    }

    /* 本地保持状态（替代原先 is_jog_pos_req / is_jog_neg_req） */
    bool pos_held = jog_pos_held[i];
    bool neg_held = jog_neg_held[i];
    bool is_fwd_limit = rt_data->axis_data[i].axis_status.is_fwd_limit;
    bool is_rev_limit = rt_data->axis_data[i].axis_status.is_rev_limit;

    /* 1. 复位事件：清除本地保持状态并强制刹车 */
    if (event == SYS_EVENT_RESET)
    {
        jog_pos_held[i] = false;
        jog_neg_held[i] = false;
        jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 2. 切模式：强制停止 */
    if (rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 3. 全局致命报警 */
    if (global_fatal)
    {
        if (jog_axis_ctrl->state != JOG_STATE_IDLE)
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* 4. 按键保持状态 → 期望方向：
     *    仅当有且仅有一个方向按住时才产生有效方向；
     *    双按（互锁）、全松均映射为 JOG_DIR_NONE（停止）。 */
    int desired_dir = JOG_DIR_NONE;
    if (pos_held && !neg_held)
        desired_dir = JOG_DIR_POSITIVE;
    else if (neg_held && !pos_held)
        desired_dir = JOG_DIR_NEGATIVE;

    /* 软限位前瞻：预测刹车距离，若超出限位则阻止该方向运动 */
    bool soft_limit_blocked = false;
    {
        double p_limit = par_axis[i][PAR_AX_PLMT].value.val_real;
        double n_limit = par_axis[i][PAR_AX_NLMT].value.val_real;
        // p_limit > n_limit 视为有限位直线轴；相等(如全0)视为无限旋转轴，跳过检查
        if (p_limit > n_limit && jog_tp[i].max_acc > 0.0)
        {
            double curr_v = jog_tp[i].curr_vel;
            double stop_dist = (curr_v * curr_v) / (2.0 * jog_tp[i].max_acc) * BRAKE_FACTOR;
            if (desired_dir == JOG_DIR_POSITIVE)
                soft_limit_blocked = ((jog_tp[i].curr_pos + stop_dist) >= p_limit);
            else if (desired_dir == JOG_DIR_NEGATIVE)
                soft_limit_blocked = ((jog_tp[i].curr_pos - stop_dist) <= n_limit);
        }
    }

    /* dir_blocked 是一个方向阻塞标志，用于判断当前期望的运动方向是否被禁止 */
    // 三个条件，任一为true即阻塞：
    // desired_dir == JOG_DIR_POSITIVE && is_fwd_limit	想往正向走，但正向硬件限位开关已触发
    // desired_dir == JOG_DIR_NEGATIVE && is_rev_limit	想往负向走，但负向硬件限位开关已触发
    // soft_limit_blocked	想走的方向，软限位前瞻预测会冲过限位
    bool dir_blocked = (desired_dir == JOG_DIR_POSITIVE && is_fwd_limit) ||
                       (desired_dir == JOG_DIR_NEGATIVE && is_rev_limit) || soft_limit_blocked;

    /* 5. 状态机切换 */
    switch (jog_axis_ctrl->state)
    {
    case JOG_STATE_IDLE:
        if (desired_dir != JOG_DIR_NONE && !dir_blocked)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = desired_dir;
        }
        break;

    case JOG_STATE_RUNNING:
        /* 运行中运动方向限位触发 → 刹车 */
        if ((jog_axis_ctrl->direction == JOG_DIR_POSITIVE && is_fwd_limit) ||
            (jog_axis_ctrl->direction == JOG_DIR_NEGATIVE && is_rev_limit))
        {
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
            break;
        }

        if (desired_dir != JOG_DIR_NONE && !dir_blocked)
        {
            /* 平滑换向/保持：规划器自动处理减速过零 */
            jog_axis_ctrl->direction = desired_dir;
        }
        else
        {
            /* 无有效方向 → 刹车（双按互锁、全松、或被限位阻挡） */
            jog_axis_ctrl->state = JOG_STATE_STOPPING;
        }
        break;

    case JOG_STATE_STOPPING:
        /* 减速过程中恢复：按键电平有效且方向无障碍则切回 RUNNING */
        if (desired_dir != JOG_DIR_NONE && !dir_blocked)
        {
            jog_axis_ctrl->state = JOG_STATE_RUNNING;
            jog_axis_ctrl->direction = desired_dir;
        }
        break;
    }
}

/**
 * @brief 轴运动执行逻辑 (带前瞻软限位拦截)
 * 根据快速标志位选择 运动类型：快速JOG/普通JOG。
 */
static void
execute_axis_motion(int i, JOG_AXIS_CTRL *jog_axis_ctrl, jog_planner_t *axis_jog_tp, bool is_rapid)
{
    switch (jog_axis_ctrl->state)
    {
    case JOG_STATE_RUNNING:
    {
        /* 使能规划器 */
        axis_jog_tp->enable = 1; // 保持使能开启

        /* 1. 更新运动参数 */
        // 根据模式选择速度与加速度参数  普通jog：1）慢速点动速度 2）加工加速度；
        // 快速：1）快速点动速度 2）快速加速度
        double base_vel = is_rapid ? par_axis[i][PAR_AX_JOG_FASTSPD].value.val_real
                                   : par_axis[i][PAR_AX_JOG_LOWSPD].value.val_real;
        double base_acc = is_rapid ? par_axis[i][PAR_AX_RAPID_ACC].value.val_real
                                   : par_axis[i][PAR_AX_FEED_ACC].value.val_real;
        // 设置目标速度：实时获取倍率，设定带方向的目标速度
        double current_override = is_rapid ? (var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f / 100.0)
                                           : (var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / 100.0);
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
            double stop_dist = (curr_v * curr_v) / (2.0 * decel) * BRAKE_FACTOR;

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
        /* 清除本地保持状态，防止切模式时残留的 hold 状态导致误动作 */
        jog_pos_held[i] = false;
        jog_neg_held[i] = false;
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
