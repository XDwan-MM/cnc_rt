#include "jog.h"
#include "jog_control.h"
#include "global.h"
#include "log.h"
#include "alarmapi.h"
#include <math.h>

/* 全局变量定义 */
simple_tp_t jog_tp[3]; 

static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}};

/* 内部私有函数 */
static void update_axis_state(int i);
static void execute_axis_motion(int i);
static bool check_all_axes_idle(void);

/* --- 3. 外部接口函数定义 --- */

/**
 * @brief JOG 任务主循环 (1ms周期)
 */
void handle_jog_cmd()
{
    int i;
    for (i = 0; i < SYS_AXES_NUM_JOG; i++)
    {
        /* * 静态跟随：
         * 只有在空闲时，规划器才跟随系统当前位置。
         * 一旦进入 RUNNING，此处不再执行，保证了规划器内部积分的连续性。
         */
        if (jog_ctrls[i].state == JOG_STATE_IDLE)
        {
            jog_tp[i].curr_pos = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;
            jog_tp[i].curr_vel = 0.0;
            jog_tp[i].enable   = 0;
            jog_tp[i].active   = 0;
        }

        // 1. 更新轴状态机 (处理按键信号)
        update_axis_state(i);

        // 2. 执行运动控制 (驱动算法)
        execute_axis_motion(i);

        /* * 写回系统：
         * 无论在哪个状态，都将规划器的最新计算结果同步给轴指令变量。
         */
        var_axis[i][VAR_AXIS_CMD_POS_EX].value.f = jog_tp[i].curr_pos;
        var_axis[i][VAR_AXIS_CMD_VEL].value.f = jog_tp[i].curr_vel;
    }

    if (check_all_axes_idle())
    {
        rt_data->nc_event = SYS_EVENT_NONE;
    }
}
/* --- 4. 内部私有函数定义 --- */

/**
 * @brief 更新单轴状态机
 * 逻辑：只对属于本轴的事件做出响应。若收到其他轴的 Press/Release，本轴状态维持不变。
 */
static void update_axis_state(int i)
{
    SYS_EVENT event = rt_data->nc_event;

    // A. 捕捉【按下】事件：切换到 RUNNING 并锁定方向
    if (!rt_data->sys_state[SYS_IS_SWITCHING_MODE])
    {
        if ((i == AXIS_X && event == SYS_EVENT_X_POS_PRESS) ||
            (i == AXIS_Y && event == SYS_EVENT_Y_POS_PRESS) ||
            (i == AXIS_Z && event == SYS_EVENT_Z_POS_PRESS))
        {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_POSITIVE;
            return;
        }
        if ((i == AXIS_X && event == SYS_EVENT_X_NEG_PRESS) ||
            (i == AXIS_Y && event == SYS_EVENT_Y_NEG_PRESS) ||
            (i == AXIS_Z && event == SYS_EVENT_Z_NEG_PRESS))
        {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_NEGATIVE;
            return;
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
 * @brief 根据本轴独立持有的状态和方向，执行底层驱动
 * 逻辑：增加了限位报警拦截。若报警未复位，禁止向超限方向运动，仅允许反向退出。
 */
static void execute_axis_motion(int i)
{
    switch (jog_ctrls[i].state)
    {
    case JOG_STATE_RUNNING:
    {
        /* 按需配置：
         * 仅当 enable 为 0 时（即进入 RUNNING 的第一帧），配置一次参数。
         */
        if (jog_tp[i].enable == 0)
        {
            prepare_jog_params(&par_axis[i], &jog_tp[i], jog_ctrls[i].direction, 
                             var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f, false);
        }

        // 1. 获取当前是否处于限位位置
        bool is_at_pos_limit = (fabs(jog_tp[i].curr_pos - par_axis[i][PAR_AX_PLMT].value.val_real) < SPEED_THRESHOLD);
        bool is_at_neg_limit = (fabs(jog_tp[i].curr_pos - par_axis[i][PAR_AX_NLMT].value.val_real) < SPEED_THRESHOLD);

        // 2. 限位后与运动拦截逻辑
        // 如果在正限位且按下了正向移动，或者在负限位且按下了负向移动
        if ((is_at_pos_limit && jog_ctrls[i].direction == JOG_DIR_POSITIVE) || 
            (is_at_neg_limit && jog_ctrls[i].direction == JOG_DIR_NEGATIVE))
        {
            // 关键：修改状态为 STOPPING，强制减速，不进入下方的 start_jog_cont
            jog_ctrls[i].state = JOG_STATE_STOPPING;
            return; 
        }
        
        // todo
        // 如果正向软限位报警没清除，且还是按下正向移动按键 或者 负向软限位报警没清除，且还是按下正向移动按键
        // 不执行运动
        
        // 4. 执行运动：只有通过了上面的拦截逻辑，才能真正调用驱动
        /* 周期性执行算法更新 */
        simple_tp_update(&jog_tp[i], JOG_CYCLE_TIME);
        break;
    }

    case JOG_STATE_STOPPING:
        /* 撤销使能，让规划器自动减速 */
        request_jog_stop(&jog_tp[i], false);
        
        /* 继续驱动算法，直到速度降为 0 */
        simple_tp_update(&jog_tp[i], JOG_CYCLE_TIME);

        // 速度降至阈值以下，彻底释放轴控制权
        if (!jog_tp[i].active && fabs(jog_tp[i].curr_vel) < SPEED_THRESHOLD)
        {
            jog_ctrls[i].state = JOG_STATE_IDLE;
            jog_ctrls[i].direction = JOG_DIR_NONE;
        }
        break;

    case JOG_STATE_IDLE:
    default:
        break;
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
