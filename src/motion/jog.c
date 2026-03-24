#include "jog.h"
#include "global.h"
#include <math.h>
#include <string.h>

/* JOG规划器及控制状态 */
simple_tp_t jog_tp[SYS_AXES_NUM_JOG];

static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}};

/* 内部私有函数声明 */
static void prepare_jog_params(int i, int direction, double feed_override, bool is_rapid);
static void update_axis_state(int i);
static void execute_axis_logic(int i);
static bool check_all_axes_idle(void);
static void jog_handle_alarm();

/**
 * @brief JOG 任务主循环 (由 1ms 伺服中断调用)
 */
void handle_jog_cmd()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        /* 1. 静态同步：IDLE状态下强制同步位置，防止从JOG切入其他模式时位置突跳 */
        if (jog_ctrls[i].state == JOG_STATE_IDLE)
        {
            jog_tp[i].curr_pos = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;
            jog_tp[i].curr_vel = 0.0;
            jog_tp[i].enable = 0;
            jog_tp[i].active = 0;
        }

        /* 2. 处理按键信号与状态机切换 */
        update_axis_state(i);

        /* 3. 系统级报警拦截 */
        if (rt_data->sys_state[SYS_IS_ALARMING])
        {
            jog_handle_alarm();
        }

        /* 4. 执行运动计算逻辑 */
        execute_axis_logic(i);

        /* 5. 指令输出给系统变量 */
        var_axis[i][VAR_AXIS_CMD_POS_EX].value.f = jog_tp[i].curr_pos;
        var_axis[i][VAR_AXIS_CMD_VEL].value.f = jog_tp[i].curr_vel;
    }

    /* 6. 计算合成进给速度并更新显示变量 */
    double f_sq = 0;
    for (int i = 0; i < 3; i++)
    {
        f_sq += (jog_tp[i].curr_vel * jog_tp[i].curr_vel);
    }
    setVarChanDouble(0, VAR_CHAN_CMD_FEEDRATE, sqrt(f_sq) * 60.0, 0);

    // 
    if (check_all_axes_idle())
    {
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}

void jog_handle_estop()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        jog_ctrls[i].state = JOG_STATE_IDLE;
        memset(&jog_tp[i], 0, sizeof(simple_tp_t));
    }
}

/**
 * @brief 内部函数：准备并更新规划器参数
 * @param i 轴索引
 * @param direction 运动方向
 * @param feed_override 实时倍率 (0.0 - 1.2)
 * @param is_rapid 是否为快速点动模式
 */
static void prepare_jog_params(int i, int direction, double feed_override, bool is_rapid)
{
    /* 1. 模式参数选择：快速(Rapid) 或 普通(Low Speed) */
    double base_vel = is_rapid ? par_axis[i][PAR_AX_JOG_FASTSPD].value.val_real : par_axis[i][PAR_AX_JOG_LOWSPD].value.val_real;

    double base_acc = is_rapid ? par_axis[i][PAR_AX_RAPID_ACC].value.val_real : par_axis[i][PAR_AX_FEED_ACC].value.val_real;

    /* 2. 计算实时限制值 */
    jog_tp[i].max_vel = base_vel * feed_override;
    jog_tp[i].max_acc = base_acc;

    /* 3. 设置目标位置：设定为极远端限位以产生持续运动趋势 */
    jog_tp[i].pos_cmd = (direction == JOG_DIR_POSITIVE) ? (par_axis[i][PAR_AX_PLMT].value.val_real + 0.1) : (par_axis[i][PAR_AX_NLMT].value.val_real - 0.1);

    /* 4. 激活规划器 */
    jog_tp[i].enable = 1;
}

/**
 * @brief 轴运动执行逻辑
 */
static void execute_axis_logic(int i)
{
    switch (jog_ctrls[i].state)
    {
    case JOG_STATE_RUNNING:
    {
        /* 实时获取环境参数 */
        double cur_ovr = var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f / 100.0;
        bool is_rapid = false; // 获取快速键状态

        /* 更新规划器参数：配合线性斜坡算法，每帧更新参数可实现平滑响应 */
        prepare_jog_params(i, jog_ctrls[i].direction, cur_ovr, false);

        /* 软限位实时拦截判断 */
        bool is_limit = (jog_ctrls[i].direction == JOG_DIR_POSITIVE) ? (jog_tp[i].curr_pos >= par_axis[i][PAR_AX_PLMT].value.val_real) : (jog_tp[i].curr_pos <= par_axis[i][PAR_AX_NLMT].value.val_real);

        if (is_limit)
        {
            jog_ctrls[i].state = JOG_STATE_STOPPING;
        }

        /* 运行规划器：每一帧执行速度线性逼近 */
        simple_tp_update(&jog_tp[i], JOG_CYCLE_TIME);
        break;
    }

    case JOG_STATE_STOPPING:
        /* 关闭使能，触发底层线性减速逻辑 */
        jog_tp[i].enable = 0;
        simple_tp_update(&jog_tp[i], JOG_CYCLE_TIME);

        /* 检查是否完全停稳 */
        if (!jog_tp[i].active || fabs(jog_tp[i].curr_vel) < SPEED_THRESHOLD)
        {
            jog_ctrls[i].state = JOG_STATE_IDLE;
            jog_ctrls[i].direction = JOG_DIR_NONE;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 更新按键信号和状态机
 */
static void update_axis_state(int i)
{
    SYS_EVENT event = rt_data->nc_event;

    /* 捕捉启动信号 */
    if (jog_ctrls[i].state == JOG_STATE_IDLE && !rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        if ((i == AXIS_X && event == SYS_EVENT_X_POS_PRESS) ||
            (i == AXIS_Y && event == SYS_EVENT_Y_POS_PRESS) ||
            (i == AXIS_Z && event == SYS_EVENT_Z_POS_PRESS))
        {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_POSITIVE;
        }
        else if ((i == AXIS_X && event == SYS_EVENT_X_NEG_PRESS) ||
                 (i == AXIS_Y && event == SYS_EVENT_Y_NEG_PRESS) ||
                 (i == AXIS_Z && event == SYS_EVENT_Z_NEG_PRESS))
        {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_NEGATIVE;
        }
    }

    // B. 捕捉【释放/重置】事件：切换到 STOPPING 减速
    if ((i == AXIS_X && (event == SYS_EVENT_X_POS_RELEASE || event == SYS_EVENT_X_NEG_RELEASE)) ||
        (i == AXIS_Y && (event == SYS_EVENT_Y_POS_RELEASE || event == SYS_EVENT_Y_NEG_RELEASE)) ||
        (i == AXIS_Z && (event == SYS_EVENT_Z_POS_RELEASE || event == SYS_EVENT_Z_NEG_RELEASE)) ||
        (event == SYS_EVENT_RESET))
    {
        if (jog_ctrls[i].state == JOG_STATE_RUNNING)
        {
            jog_ctrls[i].state = JOG_STATE_STOPPING;
        }
    }
}

/**
 * @brief 检查所有轴是否都处于静止空闲状态
 */
static bool check_all_axes_idle(void)
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

static void jog_handle_alarm()
{
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        if (jog_ctrls[i].state == JOG_STATE_RUNNING)
        {
            jog_ctrls[i].state = JOG_STATE_STOPPING;
        }
    }
}