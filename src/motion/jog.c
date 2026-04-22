#include "jog.h"
#include "global.h"
#include "ferror_control.h" // 必须引入以获取具体的跟随误差状态
#include <math.h>
#include <string.h>

/* JOG规划器实例 */
jog_planner_t jog_tp[SYS_AXES_NUM_JOG];
jog_planner_t rapid_jog_tp[SYS_AXES_NUM_JOG];

/* 控制状态初始化 */
static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}};
static JOG_AXIS_CTRL rapid_jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}};

/* 在 jog.c 的文件头部定义 */
static NC_MODE last_sys_mode = MODE_EDIT; // 记录进入当前模式前，上一次的操作模式

/* 内部私有函数声明 */
static void prepare_jog_params(int i, int direction, double feed_override, jog_planner_t *axis_jog_tp, bool is_rapid);
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl);
static void execute_axis_logic(int i, JOG_AXIS_CTRL *jog_axis_ctrl, jog_planner_t *axis_jog_tp, const bool is_rapid);
static bool check_all_axes_idle_jog(void);
static bool check_all_axes_idle_rapid_jog(void);
static void sync_jog_on_mode_entry(int mode_type);

/**
 * @brief JOG 任务主循环 (由 1ms 伺服中断调用)
 */
void handle_jog_cmd()
{
    // 【核心逻辑】：
    // 如果 handle_jog_cmd 被调用了，说明系统当前一定是在执行点动逻辑。
    // 如果此时发现 last_sys_mode 还没更新成 MODE_JOG，
    // 那就说明这一帧是“从其他模式切入点动模式”的第一帧！
    if (last_sys_mode != MODE_JOG)
    {
        // 1. 立即执行同步，把规划器坐标对齐到当前物理位置
        sync_jog_on_mode_entry(0);

        // 2. 标记“我已经上班了”，下一毫秒就不会再进这个 if 了
        last_sys_mode = MODE_JOG;
    }

    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        /* 1. 静态同步：IDLE状态下强制同步位置，防止从JOG切入其他模式时位置突跳 */
        if (jog_ctrls[i].state == JOG_STATE_IDLE)
        {
            jog_tp[i].curr_pos = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;
            jog_tp[i].curr_vel = 0.0;
            jog_tp[i].enable = 0;
        }

        /* 2. 处理按键信号与状态机切换 */
        update_axis_state(i, &jog_ctrls[i]);

        /* 3. 执行普通JOG运动计算逻辑 */
        execute_axis_logic(i, &jog_ctrls[i], &jog_tp[i], false);

        /* 4. 指令输出给外部系统变量 */
        var_axis[i][VAR_AXIS_CMD_POS_EX].value.f = jog_tp[i].curr_pos;
        var_axis[i][VAR_AXIS_CMD_VEL].value.f = jog_tp[i].curr_vel;
    }

    /* 计算合成进给速度并更新给人机交互显示 */
    double f_sq = 0;
    for (int i = 0; i < 3; i++)
    {
        f_sq += (jog_tp[i].curr_vel * jog_tp[i].curr_vel);
    }
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, sqrt(f_sq) * 60.0, 0);

    // 减速完成信号赋值
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (fabs(var_axis[i][VAR_AXIS_CMD_POS_EX].value.f) < SPEED_THRESHOLD)
        {
            if (rt_data->axis_data[i].axis_status.is_fwd_limit || rt_data->axis_data[i].axis_status.is_rev_limit)
            {
                rt_data->axis_data[i].axis_status.is_decel_done = 1;
            }
        }
    }

    // 无动作时将通道事件全局变量置为NONE事件
    if (check_all_axes_idle_jog())
    {
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}

/**
 * @brief 快速JOG 任务主循环 (由 1ms 伺服中断调用)
 */
void handle_rapid_jog_cmd()
{
    if (last_sys_mode != MODE_RAPID_JOG)
    {
        sync_jog_on_mode_entry(1);
        last_sys_mode = MODE_RAPID_JOG;
    }

    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        /* 1. 静态同步：IDLE状态下强制同步位置，防止从JOG切入其他模式时位置突跳 */
        if (rapid_jog_ctrls[i].state == JOG_STATE_IDLE)
        {
            rapid_jog_tp[i].curr_pos = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;
            rapid_jog_tp[i].curr_vel = 0.0;
            rapid_jog_tp[i].enable = 0;
        }

        /* 2. 处理按键信号与状态机切换 */
        update_axis_state(i, &rapid_jog_ctrls[i]);

        /* 3. 执行快速JOG运动计算逻辑 */
        execute_axis_logic(i, &rapid_jog_ctrls[i], &rapid_jog_tp[i], true);

        /* 4. 指令输出给外部系统变量 */
        var_axis[i][VAR_AXIS_CMD_POS_EX].value.f = rapid_jog_tp[i].curr_pos;
        var_axis[i][VAR_AXIS_CMD_VEL].value.f = rapid_jog_tp[i].curr_vel;
    }

    /* 计算合成进给速度并更新给人机交互显示 */
    double f_sq = 0;
    for (int i = 0; i < 3; i++)
    {
        f_sq += (rapid_jog_tp[i].curr_vel * rapid_jog_tp[i].curr_vel);
    }
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, sqrt(f_sq) * 60.0, 0);

    // 减速完成信号赋值
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (fabs(var_axis[i][VAR_AXIS_CMD_POS_EX].value.f) < SPEED_THRESHOLD)
        {
            if (rt_data->axis_data[i].axis_status.is_fwd_limit || rt_data->axis_data[i].axis_status.is_rev_limit)
            {
                rt_data->axis_data[i].axis_status.is_decel_done = 1;
            }
        }
    }

    // 无动作时将通道事件全局变量置为NONE事件
    if (check_all_axes_idle_rapid_jog())
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
        // 普通JOG急停复位
        jog_ctrls[i].state = JOG_STATE_IDLE;
        jog_ctrls[i].direction = JOG_DIR_NONE;
        memset(&jog_tp[i], 0, sizeof(jog_planner_t));
        // 快速JOG急停复位
        rapid_jog_ctrls[i].state = JOG_STATE_IDLE;
        rapid_jog_ctrls[i].direction = JOG_DIR_NONE;
        memset(&rapid_jog_tp[i], 0, sizeof(jog_planner_t));
    }
}

/**
 * @brief 根据按键事件更新状态机
 */
static void update_axis_state(int i, JOG_AXIS_CTRL *jog_axis_ctrl)
{
    // 获取当前通道的系统事件（从全局变量获取）
    SYS_EVENT event = rt_data->nc_event;

    /* 获取当前轴的限位状态 （从全局变量获取）*/
    bool is_fwd_limit = rt_data->axis_data[i].axis_status.is_fwd_limit;
    bool is_rev_limit = rt_data->axis_data[i].axis_status.is_rev_limit;

    /* 解析当前轴的按键意图方向 */
    bool press_pos = false;
    bool press_neg = false;
    bool is_release = false;

    if (i == AXIS_X)
    {
        press_pos = (event == SYS_EVENT_X_POS_PRESS);
        press_neg = (event == SYS_EVENT_X_NEG_PRESS);
        is_release = (event == SYS_EVENT_X_POS_RELEASE || event == SYS_EVENT_X_NEG_RELEASE);
    }
    else if (i == AXIS_Y)
    {
        press_pos = (event == SYS_EVENT_Y_POS_PRESS);
        press_neg = (event == SYS_EVENT_Y_NEG_PRESS);
        is_release = (event == SYS_EVENT_Y_POS_RELEASE || event == SYS_EVENT_Y_NEG_RELEASE);
    }
    else if (i == AXIS_Z)
    {
        press_pos = (event == SYS_EVENT_Z_POS_PRESS);
        press_neg = (event == SYS_EVENT_Z_NEG_PRESS);
        is_release = (event == SYS_EVENT_Z_POS_RELEASE || event == SYS_EVENT_Z_NEG_RELEASE);
    }

    /* 1. 处理复位,切模式：强制停止所有运动 */
    if (event == SYS_EVENT_RESET || rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        // 切换至减速停止状态
        jog_axis_ctrl->state = JOG_STATE_STOPPING;
        return;
    }

    /* ==================== 核心修改：全局非限位报警拦截逻辑 ==================== */
    if (rt_data->sys_state[SYS_IS_ALARMING])
    {
        bool has_global_fatal_alarm = false; // 是否存在全局级别的致命报警
        bool any_axis_fatal = false;         // 是否有任何轴存在跟随误差/伺服报警
        bool any_axis_limit = false;         // 是否有任何轴触发了限位

        // 遍历所有轴，收集当前报警的具体特征
        for (int j = 0; j < SYS_AXES_NUM_JOG; j++)
        {
            // 只要有任意一轴发生了伺服错误或跟随误差，整个系统判定为致命报警
            if (rt_data->axis_data[j].axis_status.is_error || ferror_control[j].error_flag)
            {
                any_axis_fatal = true;
                // break; // 只要找到一个致命报警，就无需继续判断
            }
            if (rt_data->axis_data[j].axis_status.is_fwd_limit || rt_data->axis_data[j].axis_status.is_rev_limit)
            {
                any_axis_limit = true;
            }
        }

        /* * 逻辑综合：
         * 1. 存在致命报警 (any_axis_fatal) -> 必须停！
         * 2. 没有致命报警，且没有任何轴在限位 (!any_axis_limit) 但 SYS_IS_ALARMING 仍为真
         * -> 说明发生了系统级其他报警（如主轴、IO异常等） -> 必须停！
         */
        if (any_axis_fatal || !any_axis_limit)
        {
            has_global_fatal_alarm = true;
        }

        // 如果存在全局致命报警，拦截当前轴(所有轴遍历到此时都会被拦截)
        if (has_global_fatal_alarm)
        {
            if (jog_axis_ctrl->state != JOG_STATE_IDLE)
                jog_axis_ctrl->state = JOG_STATE_STOPPING;
            return;
        }

        // 运行到这里说明：SYS_IS_ALARMING虽然为真，但仅仅是因为某些轴触发了限位引起的。
        // 那么放行逻辑，继续往下走判断方向是否合法（实现了X轴限位不影响Y轴运动）。
    }
    /* ========================================================================= */

    /* 2. 状态机切换逻辑改写 */
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
static void execute_axis_logic(int i, JOG_AXIS_CTRL *jog_axis_ctrl, jog_planner_t *axis_jog_tp, bool is_rapid)
{
    switch (jog_axis_ctrl->state)
    {
    case JOG_STATE_RUNNING:
    {
        /* 使能规划器 */
        axis_jog_tp->enable = 1; // 保持使能开启
        // 获取倍率
        double current_override = var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / 100.0;

        /* 1. 更新目标速度 */
        prepare_jog_params(i, jog_axis_ctrl->direction, current_override, axis_jog_tp, is_rapid);

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
 * @brief 内部函数：准备并更新规划器参数
 * @param i 轴索引
 * @param direction 运动方向
 * @param feed_override 实时倍率 (0.0 - 1.2)
 * @param is_rapid 是否为快速点动模式
 */
static void prepare_jog_params(int i, int direction, double feed_override, jog_planner_t *axis_jog_tp, bool is_rapid)
{
    /* 1. 模式参数选择 */
    // 普通jog：1）慢速点动速度 2）加工加速度
    // 快速jog：1）快速点动速度 2）快速加速度
    double base_vel = is_rapid ? par_axis[i][PAR_AX_JOG_FASTSPD].value.val_real : par_axis[i][PAR_AX_JOG_LOWSPD].value.val_real;
    double base_acc = is_rapid ? par_axis[i][PAR_AX_RAPID_ACC].value.val_real : par_axis[i][PAR_AX_FEED_ACC].value.val_real;

    /* 2. 直接设定带方向的目标速度 */
    axis_jog_tp->target_vel = base_vel * feed_override * direction;
    axis_jog_tp->max_acc = base_acc;
}

/**
 * @brief jog模式下 检查所有轴是否都处于静止空闲状态
 */
static bool check_all_axes_idle_jog(void)
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

/**
 * @brief 快速jog模式下 检查所有轴是否都处于静止空闲状态
 */
static bool check_all_axes_idle_rapid_jog(void)
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (rapid_jog_ctrls[i].state != JOG_STATE_IDLE)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 内部同步函数：当检测到模式切换进入点动时，强制对齐所有状态
 */
static void sync_jog_on_mode_entry(int mode_type)
{
    // mode_type: 0 - 普通JOG, 1 - 快速JOG
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        double actual_pos = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;

        if (mode_type == 0)
        {
            jog_ctrls[i].state = JOG_STATE_IDLE;
            jog_ctrls[i].direction = JOG_DIR_NONE;
            jog_tp[i].curr_pos = actual_pos;
            jog_tp[i].curr_vel = 0.0;
            jog_tp[i].enable = 0;
        }
        else
        {
            rapid_jog_ctrls[i].state = JOG_STATE_IDLE;
            rapid_jog_ctrls[i].direction = JOG_DIR_NONE;
            rapid_jog_tp[i].curr_pos = actual_pos;
            rapid_jog_tp[i].curr_vel = 0.0;
            rapid_jog_tp[i].enable = 0;
        }
    }
}