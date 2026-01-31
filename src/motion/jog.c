#include "jog.h"
#include "jog_control.h"
#include <math.h>


// 静态局部变量，保存各轴 JOG 状态
static JOG_CTRL x_jog_ctrl = {JOG_STATE_IDLE, 0.001};
static JOG_CTRL y_jog_ctrl = {JOG_STATE_IDLE, 0.001};
static JOG_CTRL z_jog_ctrl = {JOG_STATE_IDLE, 0.001};

// 内部逻辑函数
static void jog_set_state();
static void jog_execute();
static bool jog_is_all_idle();

/**
 * @brief JOG 任务主入口
 */
void handle_jog_cmd() {
    // 1. 处理急停 (E-Stop 是最高优先级，强制状态重置)
    if (var_chan[0].is_estop) {
        stop_jog_cont(&var_axis[AXIS_X], 0.001, true);
        stop_jog_cont(&var_axis[AXIS_Y], 0.001, true);
        stop_jog_cont(&var_axis[AXIS_Z], 0.001, true);

        x_jog_ctrl.state = JOG_STATE_IDLE;
        y_jog_ctrl.state = JOG_STATE_IDLE;
        z_jog_ctrl.state = JOG_STATE_IDLE;
    }

    // 2. 更新位置信息 (同步当前位置到 TP)
    var_axis[AXIS_X].jog_tp.curr_pos = var_axis[AXIS_X].cmd_pos;
    var_axis[AXIS_Y].jog_tp.curr_pos = var_axis[AXIS_Y].cmd_pos;
    var_axis[AXIS_Z].jog_tp.curr_pos = var_axis[AXIS_Z].cmd_pos;

    // 3. 根据事件设置状态
    jog_set_state();

    // 4. 执行状态对应的动作 (start/stop)
    jog_execute();

    // 5. 将计算后的指令位置写回全局变量
    var_axis[AXIS_X].cmd_pos = var_axis[AXIS_X].jog_tp.curr_pos;
    var_axis[AXIS_Y].cmd_pos = var_axis[AXIS_Y].jog_tp.curr_pos;
    var_axis[AXIS_Z].cmd_pos = var_axis[AXIS_Z].jog_tp.curr_pos;

    // 6. 如果所有轴都静止了，清除事件（参考 homing 完成逻辑）
    if (jog_is_all_idle()) {
        sys->nc_event = SYS_EVENT_NONE;
    }
}

/**
 * @brief 根据系统事件切换各轴状态机
 */
static void jog_set_state() {
    switch (sys->nc_event) {
    case SYS_EVENT_NONE:
        break;

    case SYS_EVENT_RESET:
        x_jog_ctrl.state = JOG_STATE_STOPPING;
        y_jog_ctrl.state = JOG_STATE_STOPPING;
        z_jog_ctrl.state = JOG_STATE_STOPPING;
        break;

        // X 轴事件
    case SYS_EVENT_X_POS_PRESS:
        if (!sys->is_switching_mode) 
        {
            x_jog_ctrl.state = JOG_STATE_RUNNING;
        }
        else
        {
            x_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        // 内部通过方向区分，这里简单标记为运行
        break;
    case SYS_EVENT_X_NEG_PRESS:
        if (!sys->is_switching_mode) 
        {
            x_jog_ctrl.state = JOG_STATE_RUNNING;
        } 
        else
        {
            x_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        break;
    case SYS_EVENT_X_POS_RELEASE:
    case SYS_EVENT_X_NEG_RELEASE:
        x_jog_ctrl.state = JOG_STATE_STOPPING;
        break;

        // Y 轴事件
    case SYS_EVENT_Y_POS_PRESS:
        if (!sys->is_switching_mode) 
        {
            y_jog_ctrl.state = JOG_STATE_RUNNING;
        }
        else
        {
            y_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        break;
    case SYS_EVENT_Y_NEG_PRESS:
        if (!sys->is_switching_mode) 
        {
            y_jog_ctrl.state = JOG_STATE_RUNNING;
        }
        else
        {
            y_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        
        break;
    case SYS_EVENT_Y_POS_RELEASE:
    case SYS_EVENT_Y_NEG_RELEASE:
        y_jog_ctrl.state = JOG_STATE_STOPPING;
        break;

        // Z 轴事件
    case SYS_EVENT_Z_POS_PRESS:
        if (!sys->is_switching_mode) 
        {
            z_jog_ctrl.state = JOG_STATE_RUNNING;
        }
        else
        {
            z_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        break;
    case SYS_EVENT_Z_NEG_PRESS:
        if (!sys->is_switching_mode) 
        {
            z_jog_ctrl.state = JOG_STATE_RUNNING;
        }
        else
        {
            z_jog_ctrl.state = JOG_STATE_STOPPING;
        }
        break;
    case SYS_EVENT_Z_POS_RELEASE:
    case SYS_EVENT_Z_NEG_RELEASE:
        z_jog_ctrl.state = JOG_STATE_STOPPING;
        break;

    default:
        break;
    }
}

/**
 * @brief 执行具体运动指令
 */
static void jog_execute() {
    // 处理 X 轴
    if (x_jog_ctrl.state == JOG_STATE_RUNNING) 
    {
        int dir = 0;
        if (sys->nc_event == SYS_EVENT_X_POS_PRESS)
        {
            dir = JOG_DIR_POSITIVE;
        }
        else if (sys->nc_event == SYS_EVENT_X_NEG_PRESS)
        {
            dir = JOG_DIR_NEGATIVE;
        }
        start_jog_cont(&var_axis[AXIS_X], &par_axis[AXIS_X], dir, 0.001, var_chan[0].feed_override);
    } 
    else if (x_jog_ctrl.state == JOG_STATE_STOPPING) 
    {
        stop_jog_cont(&var_axis[AXIS_X], 0.001, false);
        if (fabs(var_axis[AXIS_X].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD) 
        {
            x_jog_ctrl.state = JOG_STATE_IDLE;
        }
    }

    // 处理 Y 轴
    if (y_jog_ctrl.state == JOG_STATE_RUNNING) 
    {
        int dir = 0;
        if (sys->nc_event == SYS_EVENT_Y_POS_PRESS)
        {
            dir = JOG_DIR_POSITIVE;
        }
        else if (sys->nc_event == SYS_EVENT_Y_NEG_PRESS)
        {
            dir = JOG_DIR_NEGATIVE;
        }
        start_jog_cont(&var_axis[AXIS_Y], &par_axis[AXIS_Y], dir, 0.001, var_chan[0].feed_override);
    } 
    else if (y_jog_ctrl.state == JOG_STATE_STOPPING) 
    {
        stop_jog_cont(&var_axis[AXIS_Y], 0.001, false);
        if (fabs(var_axis[AXIS_Y].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD) 
        {
            y_jog_ctrl.state = JOG_STATE_IDLE;
        }
    }

    // 处理 Z 轴
    if (z_jog_ctrl.state == JOG_STATE_RUNNING) 
    {
        int dir = 0;
        if (sys->nc_event == SYS_EVENT_Z_POS_PRESS)
        {
            dir = JOG_DIR_POSITIVE;
        }
        else if (sys->nc_event == SYS_EVENT_Z_NEG_PRESS)
        {
            dir = JOG_DIR_NEGATIVE;
        }
        start_jog_cont(&var_axis[AXIS_Z], &par_axis[AXIS_Z], dir, 0.001, var_chan[0].feed_override);
    } 
    else if (z_jog_ctrl.state == JOG_STATE_STOPPING)
    {
        stop_jog_cont(&var_axis[AXIS_Z], 0.001, false);
        if (fabs(var_axis[AXIS_Z].jog_tp.curr_vel - 0.0) < SPEED_THRESHOLD) 
        {
            z_jog_ctrl.state = JOG_STATE_IDLE;
        }
    }
}

/**
 * @brief 检查所有轴是否处于空闲状态
 */
static bool jog_is_all_idle() {
    return (x_jog_ctrl.state == JOG_STATE_IDLE &&
            y_jog_ctrl.state == JOG_STATE_IDLE &&
            z_jog_ctrl.state == JOG_STATE_IDLE);
}
