#include "jog.h"
#include "jog_control.h"
#include "global.h"
#include <math.h>

/* --- 1. 静态局部变量 (每个轴私有的状态块) --- */
static JOG_AXIS_CTRL jog_ctrls[SYS_AXES_NUM_JOG] = {
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE},
    {JOG_STATE_IDLE, JOG_DIR_NONE}
};

/* --- 2. 内部私有函数声明 --- */
static void update_axis_state(int i);
static void execute_axis_motion(int i);
static bool check_all_axes_idle(void);

/* --- 3. 外部接口函数定义 --- */

/**
 * @brief JOG 任务主入口
 */
void handle_jog_cmd() {
    int i;

    // 1. 处理系统级急停
    if (var_chan[0].is_estop) {
        for (i = 0; i < SYS_AXES_NUM_JOG; i++) {
            stop_jog_cont(&var_axis[i].jog_tp, JOG_CYCLE_TIME, true);
            jog_ctrls[i].state = JOG_STATE_IDLE;
            jog_ctrls[i].direction = JOG_DIR_NONE;
        }
        sys->nc_event = SYS_EVENT_NONE;
        return;
    }

    // 2. 遍历各轴独立处理
    for (i = 0; i < SYS_AXES_NUM_JOG; i++) {
        // 同步位置
        var_axis[i].jog_tp.curr_pos = var_axis[i].cmd_pos;

        // 更新本轴独立状态机 (事件驱动)
        update_axis_state(i);

        // 执行运动控制 (状态驱动)
        execute_axis_motion(i);

        // 写回位置
        var_axis[i].cmd_pos = var_axis[i].jog_tp.curr_pos;
    }

    // 3. 自动清理事件：若所有轴都回到了 IDLE 状态，重置系统事件
    if (check_all_axes_idle()) {
        sys->nc_event = SYS_EVENT_NONE;
    }
}

/* --- 4. 内部私有函数定义 --- */

/**
 * @brief 更新单轴状态机
 * 逻辑：只对属于本轴的事件做出响应。若收到其他轴的 Press/Release，本轴状态维持不变。
 */
static void update_axis_state(int i) {
    SYS_EVENT event = sys->nc_event;

    // A. 捕捉【按下】事件：切换到 RUNNING 并锁定方向
    if (!sys->is_switching_mode) {
        if ((i == AXIS_X && event == SYS_EVENT_X_POS_PRESS) ||
            (i == AXIS_Y && event == SYS_EVENT_Y_POS_PRESS) ||
            (i == AXIS_Z && event == SYS_EVENT_Z_POS_PRESS)) {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_POSITIVE;
            return;
        }
        if ((i == AXIS_X && event == SYS_EVENT_X_NEG_PRESS) ||
            (i == AXIS_Y && event == SYS_EVENT_Y_NEG_PRESS) ||
            (i == AXIS_Z && event == SYS_EVENT_Z_NEG_PRESS)) {
            jog_ctrls[i].state = JOG_STATE_RUNNING;
            jog_ctrls[i].direction = JOG_DIR_NEGATIVE;
            return;
        }
    }

    // B. 捕捉【释放/重置】事件：切换到 STOPPING 减速
    if ((i == AXIS_X && (event == SYS_EVENT_X_POS_RELEASE || event == SYS_EVENT_X_NEG_RELEASE)) ||
        (i == AXIS_Y && (event == SYS_EVENT_Y_POS_RELEASE || event == SYS_EVENT_Y_NEG_RELEASE)) ||
        (i == AXIS_Z && (event == SYS_EVENT_Z_POS_RELEASE || event == SYS_EVENT_Z_NEG_RELEASE)) ||
        (event == SYS_EVENT_RESET)) {

        if (jog_ctrls[i].state == JOG_STATE_RUNNING) {
            jog_ctrls[i].state = JOG_STATE_STOPPING;
        }
    }
}

/**
 * @brief 根据本轴独立持有的状态和方向，执行底层驱动
 */
static void execute_axis_motion(int i) {
    VAR_AXIS *pAxis = &var_axis[i];

    switch (jog_ctrls[i].state) {
    case JOG_STATE_RUNNING:
        // 使用记忆中的方向锁定运行
        start_jog_cont(&par_axis[i], &var_axis[i].jog_tp, jog_ctrls[i].direction, var_chan[0].feed_override, JOG_CYCLE_TIME);
        break;

    case JOG_STATE_STOPPING:
        stop_jog_cont(&var_axis[i].jog_tp, JOG_CYCLE_TIME, false);
        // 速度降至阈值以下，彻底释放轴控制权
        if (fabs(pAxis->jog_tp.curr_vel) < SPEED_THRESHOLD) {
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
static bool check_all_axes_idle(void) {
    for (int i = 0; i < SYS_AXES_NUM_JOG; i++) {
        if (jog_ctrls[i].state != JOG_STATE_IDLE) {
            return false;
        }
    }
    return true;
}
