#include "jog.h"
#include "jog_control.h"
#include "hal_driver.h"
#include <math.h>

void handle_jog_cmd() {
    // 急停不归我管,触发了就置真.
    //　急停状态，立即停止
    if (var_chan[0].is_estop) {
        stop_jog_cont(&var_axis[AXIS_X], 0.001, true);
        stop_jog_cont(&var_axis[AXIS_Y], 0.001, true);
        stop_jog_cont(&var_axis[AXIS_Z], 0.001, true);
    }
    // 获取当前位置值
    var_axis[AXIS_X].jog_tp.curr_pos = var_axis[AXIS_X].cmd_pos;
    var_axis[AXIS_Y].jog_tp.curr_pos = var_axis[AXIS_Y].cmd_pos;
    var_axis[AXIS_Z].jog_tp.curr_pos = var_axis[AXIS_Z].cmd_pos;

    //    printf("jog_tp.curr_pos: %f %f %f\n", var_axis[AXIS_X].jog_tp.curr_pos, var_axis[AXIS_Y].jog_tp.curr_pos, var_axis[AXIS_Y].jog_tp.curr_pos);

    switch (sys->nc_event) {
        case SYS_EVENT_NONE:
            break;
        case SYS_EVENT_RESET:
            stop_jog_cont(&var_axis[AXIS_X], 0.001, false);
            stop_jog_cont(&var_axis[AXIS_Y], 0.001, false);
            stop_jog_cont(&var_axis[AXIS_Z], 0.001, false);
            break;

        case SYS_EVENT_X_POS_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_X], &par_axis[AXIS_X], JOG_DIR_POSITIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_X_POS_RELEASE:
            stop_jog_cont(&var_axis[AXIS_X], 0.001, false);
            break;
        case SYS_EVENT_X_NEG_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_X], &par_axis[AXIS_X], JOG_DIR_NEGATIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_X_NEG_RELEASE:
            stop_jog_cont(&var_axis[AXIS_X], 0.001, false);
            break;

        case SYS_EVENT_Y_POS_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_Y], &par_axis[AXIS_Y], JOG_DIR_POSITIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_Y_POS_RELEASE:
            stop_jog_cont(&var_axis[AXIS_Y], 0.001, false);
            break;
        case SYS_EVENT_Y_NEG_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_Y], &par_axis[AXIS_Y], JOG_DIR_NEGATIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_Y_NEG_RELEASE:
            stop_jog_cont(&var_axis[AXIS_Y], 0.001, false);
            break;

        case SYS_EVENT_Z_POS_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_Z], &par_axis[AXIS_Z], JOG_DIR_POSITIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_Z_POS_RELEASE:
            stop_jog_cont(&var_axis[AXIS_Z], 0.001, false);
            break;
        case SYS_EVENT_Z_NEG_PRESS:
            if (sys->is_switching_mode) break; // 换挡过程中，不允许操作
            start_jog_cont(&var_axis[AXIS_Z], &par_axis[AXIS_Z], JOG_DIR_NEGATIVE, 0.001, var_chan[0].feed_override);
            break;
        case SYS_EVENT_Z_NEG_RELEASE:
            stop_jog_cont(&var_axis[AXIS_Z], 0.001, false);
            break;
        default:
            break;
    }
    if (fabs(var_axis[AXIS_X].jog_tp.curr_vel - 0.0) < 1e-7 && fabs(var_axis[AXIS_Y].jog_tp.curr_vel - 0.0) < 1e-7 && fabs(var_axis[AXIS_Z].jog_tp.curr_vel - 0.0) < 1e-7
        && spindle.mode == DRV_MODE_DISABLED) {
        sys->nc_event = SYS_EVENT_NONE;
        // 三个轴速度为0时, 轴移动完成.
        var_chan[0].is_moving = false;
    } else {
        // 三个轴速度不为0时, 轴正在移动.
        var_chan[0].is_moving = true;
    }

    // 输出当前位置,供人机交互显示
    var_axis[AXIS_X].cmd_pos = var_axis[AXIS_X].jog_tp.curr_pos;
    var_axis[AXIS_Y].cmd_pos = var_axis[AXIS_Y].jog_tp.curr_pos;
    var_axis[AXIS_Z].cmd_pos = var_axis[AXIS_Z].jog_tp.curr_pos;

    // 输出当前速度值,供人机交互显示
    var_axis[AXIS_X].cmd_vel = var_axis[AXIS_X].jog_tp.curr_vel;
    var_axis[AXIS_Y].cmd_vel = var_axis[AXIS_Y].jog_tp.curr_vel;
    var_axis[AXIS_Z].cmd_vel = var_axis[AXIS_Z].jog_tp.curr_vel;

    //    printf("var_axis[AXIS_X].cmd_pos: %f %f %f\n", var_axis[AXIS_X].jog_tp.curr_pos, var_axis[AXIS_Y].jog_tp.curr_pos, var_axis[AXIS_Y].jog_tp.curr_pos);
}
