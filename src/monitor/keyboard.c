#include "keyboard.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include "pmc_io_map.h"
#include "global.h"

// 倍率限制
#define OVERRIDE_MIN   0
#define OVERRIDE_MAX   200
#define OVERRIDE_STEP  10

// 当前状态
static struct termios g_old_termios;
static int g_feed_override   = 100;
static int g_rapid_override  = 100;
static int g_status_first       = 1;   // print_status 是否首次打印
static int g_spindle_override    = 100;
static int g_pending_nc_mode     = -1;  // 键盘请求的目标 NC_MODE，-1 表示无待切换模式
static int g_jog_pressed[3]      = {0, 0, 0};  // 各轴点动按键状态: 0=松开, 1=正向按下, -1=负向按下

// 模式名称，索引对应 NC_MODE 枚举（与 emcmot.cpp mode_names 对齐）
static const char* mode_names[] = {
    "编辑",   // MODE_EDIT = 0
    "AUTO",   // MODE_MEM  = 1
    "MDI",    // MODE_MDI  = 2
    "DNC",    // MODE_DNC  = 3
    "手轮",   // MODE_MPG  = 4
    "手动",   // MODE_JOG  = 5
    "回零",   // MODE_HOMING = 6
    "空闲",   // MODE_IDLE = 7
};

static int  adjust_override(int current, int delta);
static int  read_key(void);
static void process_key(int key);
static void scan_key_to_event(int key);
static void print_help(void);
static void print_status(void);

// 键盘监控主函数
void hal_keyboard_minitor(void) {
    int key;
    key = read_key();

    // 解析点动按键（带按下/松开状态追踪）
    int jog_axis = -1, jog_dir = 0;
    switch (key) {
    case 'u': case 'U': jog_axis = 0; jog_dir =  1; break;
    case 'j': case 'J': jog_axis = 0; jog_dir = -1; break;
    case 'i': case 'I': jog_axis = 1; jog_dir =  1; break;
    case 'k': case 'K': jog_axis = 1; jog_dir = -1; break;
    case 'o': case 'O': jog_axis = 2; jog_dir =  1; break;
    case 'l': case 'L': jog_axis = 2; jog_dir = -1; break;
    }

    if (jog_axis >= 0) {
        // 点动键：方向变化时先松旧方向再按新方向；按键重复则忽略
        if (g_jog_pressed[jog_axis] != jog_dir) {
            if (g_jog_pressed[jog_axis] != 0) {
                scan_key_to_event(KEY_EV_X_POS_RELEASE + jog_axis * 4
                    + ((g_jog_pressed[jog_axis] > 0) ? 0 : 2));
            }
            scan_key_to_event(KEY_EV_X_POS_PRESS + jog_axis * 4
                + ((jog_dir > 0) ? 0 : 2));
            g_jog_pressed[jog_axis] = jog_dir;
        }
    } else {
        // 非点动键：松开所有已按下的点动轴
        for (int i = 0; i < 3; i++) {
            if (g_jog_pressed[i] != 0) {
                scan_key_to_event(KEY_EV_X_POS_RELEASE + i * 4
                    + ((g_jog_pressed[i] > 0) ? 0 : 2));
                g_jog_pressed[i] = 0;
            }
        }
        if (key > 0) {
            process_key(key);
        }
    }
    // 仅在有按键输入或点动轴按下时刷新状态，避免空闲时反复刷屏
    if (key > 0 || g_jog_pressed[0] || g_jog_pressed[1] || g_jog_pressed[2]) {
        print_status();
    }
}

// 初始化终端
int init_keyboard(void) {
    struct termios new_termios;

    if (tcgetattr(STDIN_FILENO, &g_old_termios) < 0) {
        perror("tcgetattr failed");
        return -1;
    }

    new_termios = g_old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN]  = 0;
    new_termios.c_cc[VTIME] = 1;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    print_help();
    print_status();
    return 0;
}

// 恢复终端设置
static void restore_keyboard(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
}

// 读取单个按键
static int read_key(void) {
    unsigned char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1) {
        return ch;
    }
    return -1;
}

// 打印帮助信息
static void print_help(void) {
    g_status_first = 1;  // 清屏后重置，下次 print_status 从头开始
    printf("\033[2J\033[H");
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║                     \033[1;36mCNC 键盘控制监控\033[0m                             ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║                          \033[1;33m按键说明\033[0m                                ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ [模式] 1:JOG  2:DNC  3:手轮  4:回零  5:编辑  6:自动  7:MDI      ║\n");
    printf("║ [倍率] Q/A:进给+/-10%%  W/S:快速+/-10%%  E/D:主轴+/-10%%            ║\n");
    printf("║ [点动] U/J:+X/-X  I/K:+Y/-Y  O/L:+Z/-Z                           ║\n");
    printf("║ [主轴] Z:正转  X:停止  C:反转                                    ║\n");
    printf("║ [运行] Enter:运行  Space:暂停  R:复位  Esc:急停                  ║\n");
    printf("║ [帮助] h/H:按键点位  p/P:系统信息                                ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
}

// 打印当前状态，支持原地刷新覆盖旧内容
static void print_status(void) {
    if (!g_status_first) {
        printf("\033[u");   // 恢复光标到上次状态块起始位置
    }
    printf("\033[s");       // 保存当前光标位置，供下次刷新使用
    printf("\033[J");       // 清除光标到屏幕末尾
    printf("[============== CNC HMI ===================================\033[0m]\n");
    // 显示模式：键盘刚请求切换但 motion 线程尚未处理时，显示目标模式
    {
        NC_MODE display_mode = rt_data->current_mode;
        if (g_pending_nc_mode >= 0) {
            if (rt_data->current_mode == g_pending_nc_mode) {
                g_pending_nc_mode = -1;  // 切换已完成
            } else {
                display_mode = (NC_MODE)g_pending_nc_mode;
            }
        }
        printf("[状态: \033[1;32m%s\033[0m] ",
               (display_mode <= MODE_IDLE) ? mode_names[display_mode] : "未知");
    }
    printf("[进给: \033[1;33m%3d%%\033[0m] ", g_feed_override);
    printf("[快速: \033[1;33m%3d%%\033[0m] ", g_rapid_override);
    printf("[主轴: \033[1;33m%3d%%\033[0m]\n", g_spindle_override);
    printf("[译码行: \033[1;33m%3d\033[0m]", var_chan[0][VAR_CHAN_DCD_ROW].value.i);
    printf("[运行行: \033[1;33m%3d\033[0m]", var_chan[0][VAR_CHAN_RUN_ROW].value.i);
    printf("[程序完成: %s\033[0m]\n", var_chan[0][VAR_CHAN_IS_PROGEND].value.i ? "\033[1;32m是" : "\033[1;37m否");
    printf("[急停: %s\033[0m]", rt_data->sys_state[SYS_IS_ESTOP_ON] ? "\033[1;31m是" : "\033[1;32m否");
    printf("[复位: %s\033[0m]", var_chan[0][VAR_CHAN_IS_RESETTING].value.i ? "\033[1;32m是" : "\033[1;37m否");
    printf("[运行: %s\033[0m]", var_chan[0][VAR_CHAN_IS_RUNNING].value.i ? "\033[1;32m是" : "\033[1;37m否");
    printf("[回零: %s\033[0m]", var_chan[0][VAR_CHAN_IS_HOMING].value.i ? "\033[1;32m是" : "\033[1;37m否");
    printf("[移动: %s\033[0m]\n", var_chan[0][VAR_CHAN_IS_MOVING].value.i ? "\033[1;32m是" : "\033[1;37m否");
    printf("[X轴位:\033[1;33m%9.3f\033[0m 补:\033[1;35m%+.3f\033[0m] "
           "[Y轴位:\033[1;33m%9.3f\033[0m 补:\033[1;35m%+.3f\033[0m] "
           "[Z轴位:\033[1;33m%9.3f\033[0m 补:\033[1;35m%+.3f\033[0m]\n",
           rt_data->axis_data[AXIS_X].motor_cmd_pos, rt_data->axis_data[AXIS_X].pitch_comp_val,
           rt_data->axis_data[AXIS_Y].motor_cmd_pos, rt_data->axis_data[AXIS_Y].pitch_comp_val,
           rt_data->axis_data[AXIS_Z].motor_cmd_pos, rt_data->axis_data[AXIS_Z].pitch_comp_val);
    printf("[事件数量: %d\033[0m]\n", event_queue_get_size(event_queue));
    printf("[===========================================================\033[0m]\n");
    g_status_first = 0;
    fflush(stdout);
}

// 处理按键事件
static void process_key(int key) {
    switch (key) {
    // 模式切换 1-7
    case '1': scan_key_to_event(KEY_EV_JOG);  break;
    case '2': scan_key_to_event(KEY_EV_DNC);  break;
    case '3': scan_key_to_event(KEY_EV_MPG);  break;
    case '4': scan_key_to_event(KEY_EV_HOME); break;
    case '5': scan_key_to_event(KEY_EV_EDIT); break;
    case '6': scan_key_to_event(KEY_EV_MEM);  break;
    case '7': scan_key_to_event(KEY_EV_MDI);  break;
    // 帮助与状态
    case 'h': case 'H': print_help();    break;
    case 'p': case 'P': print_status();  break;
    // 进给倍率 Q/A
    case 'q': case 'Q': scan_key_to_event(KEY_EV_FO_INC); break;
    case 'a': case 'A': scan_key_to_event(KEY_EV_FO_DEC); break;
    // 快速倍率 W/S
    case 'w': case 'W': scan_key_to_event(KEY_EV_RO_INC); break;
    case 's': case 'S': scan_key_to_event(KEY_EV_RO_DEC); break;
    // 主轴倍率 E/D
    case 'e': case 'E': scan_key_to_event(KEY_EV_SO_INC); break;
    case 'd': case 'D': scan_key_to_event(KEY_EV_SO_DEC); break;
    // 主轴控制
    case 'z': scan_key_to_event(KEY_EV_SPINDLE_FWD);  break;
    case 'x': scan_key_to_event(KEY_EV_SPINDLE_STOP); break;
    case 'c': scan_key_to_event(KEY_EV_SPINDLE_REV);  break;
    // 运行控制
    case '\n': case '\r': scan_key_to_event(KEY_EV_RERUN); break;
    case ' ':             scan_key_to_event(KEY_EV_PAUSE); break;
    case 'r': case 'R':   scan_key_to_event(KEY_EV_RESET); break;
    case 27:              scan_key_to_event(KEY_EV_ESTOP); break;
    default:
        break;
    }
}

// 扫描键盘产生事件
// 模式切换遵循 set_operating_mode() 的守卫逻辑：急停拍下时不允许切换模式
static void scan_key_to_event(int key) {
    PMC_EVENT event;
    memset(&event, 0, sizeof(event));

    switch (key) {
    // === 模式切换（急停拍下时禁止，遵循 set_operating_mode()） ===
    case KEY_EV_EDIT:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_EDIT;
        g_pending_nc_mode = MODE_EDIT;
        break;
    case KEY_EV_MEM:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_MEM;
        g_pending_nc_mode = MODE_MEM;
        break;
    case KEY_EV_MDI:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_MDI;
        g_pending_nc_mode = MODE_MDI;
        break;
    case KEY_EV_DNC:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_DNC;
        g_pending_nc_mode = MODE_DNC;
        break;
    case KEY_EV_MPG:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_MPG;
        g_pending_nc_mode = MODE_MPG;
        break;
    case KEY_EV_JOG:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_JOG;
        g_pending_nc_mode = MODE_JOG;
        break;
    case KEY_EV_HOME:
        if (rt_data->sys_state[SYS_IS_ESTOP_PRESS]) return;
        event.id = EV_MSG_MODE;
        event.mode_payload.mode = CNC_MODE_ZRN;
        g_pending_nc_mode = MODE_HOMING;
        break;

    // === 运行控制 ===
    case KEY_EV_RERUN:
        event.id = EV_MSG_AUTO_RUN;
        event.run_payload.run_state = 1;
        break;
    case KEY_EV_PAUSE:
        event.id = EV_MSG_AUTO_RUN;
        event.run_payload.run_state = 0;
        break;
    case KEY_EV_RESET:
        event.id = EV_MSG_RESET;
        event.run_payload.run_state = 1;
        break;
    case KEY_EV_ESTOP:
        event.id = EV_MSG_RESET;
        event.run_payload.run_state = 1;
        break;

    // === 倍率调节 ===
    case KEY_EV_FO_INC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 0;
        event.override_payload.val = adjust_override(g_feed_override, OVERRIDE_STEP);
        g_feed_override = event.override_payload.val;
        break;
    case KEY_EV_FO_DEC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 0;
        event.override_payload.val = adjust_override(g_feed_override, -OVERRIDE_STEP);
        g_feed_override = event.override_payload.val;
        break;
    case KEY_EV_RO_INC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 1;
        event.override_payload.val = adjust_override(g_rapid_override, OVERRIDE_STEP);
        g_rapid_override = event.override_payload.val;
        break;
    case KEY_EV_RO_DEC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 1;
        event.override_payload.val = adjust_override(g_rapid_override, -OVERRIDE_STEP);
        g_rapid_override = event.override_payload.val;
        break;
    case KEY_EV_SO_INC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 2;
        event.override_payload.val = adjust_override(g_spindle_override, OVERRIDE_STEP);
        g_spindle_override = event.override_payload.val;
        break;
    case KEY_EV_SO_DEC:
        event.id = EV_MSG_OVERRIDE;
        event.override_payload.override_type = 2;
        event.override_payload.val = adjust_override(g_spindle_override, -OVERRIDE_STEP);
        g_spindle_override = event.override_payload.val;
        break;

    // === 主轴控制 ===
    case KEY_EV_SPINDLE_FWD:
        event.id = EV_MSG_SPINDLE;
        event.spindle_payload.direction = 1;
        break;
    case KEY_EV_SPINDLE_STOP:
        event.id = EV_MSG_SPINDLE;
        event.spindle_payload.direction = 0;
        break;
    case KEY_EV_SPINDLE_REV:
        event.id = EV_MSG_SPINDLE;
        event.spindle_payload.direction = -1;
        break;

    // === 点动：轴按键 ===
    case KEY_EV_X_POS_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 0;
        event.feedaxis_key_payload.direction = 1;
        break;
    case KEY_EV_X_NEG_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 0;
        event.feedaxis_key_payload.direction = -1;
        break;
    case KEY_EV_Y_POS_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 1;
        event.feedaxis_key_payload.direction = 1;
        break;
    case KEY_EV_Y_NEG_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 1;
        event.feedaxis_key_payload.direction = -1;
        break;
    case KEY_EV_Z_POS_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 2;
        event.feedaxis_key_payload.direction = 1;
        break;
    case KEY_EV_Z_NEG_PRESS:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 2;
        event.feedaxis_key_payload.direction = -1;
        break;
    case KEY_EV_X_POS_RELEASE:
    case KEY_EV_X_NEG_RELEASE:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 0;
        event.feedaxis_key_payload.direction = 0;
        break;
    case KEY_EV_Y_POS_RELEASE:
    case KEY_EV_Y_NEG_RELEASE:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 1;
        event.feedaxis_key_payload.direction = 0;
        break;
    case KEY_EV_Z_POS_RELEASE:
    case KEY_EV_Z_NEG_RELEASE:
        event.id = EV_MSG_FEEDAXIS_KEY;
        event.feedaxis_key_payload.axis_num  = 2;
        event.feedaxis_key_payload.direction = 0;
        break;

    default:
        return;
    }
    event_queue_push(event_queue, &event);
}

// 调整倍率
static int adjust_override(int current, int delta) {
    int new_val = current + delta;
    if (new_val < OVERRIDE_MIN) new_val = OVERRIDE_MIN;
    if (new_val > OVERRIDE_MAX) new_val = OVERRIDE_MAX;
    return new_val;
}
