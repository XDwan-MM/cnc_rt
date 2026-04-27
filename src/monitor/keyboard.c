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

// 全局变量
static struct termios g_old_termios;
// 倍率限制
#define OVERRIDE_MIN 0
#define OVERRIDE_MAX 200
#define OVERRIDE_STEP 10
// 当前状态
static int g_feed_override = 100;
static int g_rapid_override = 100;
static int g_spindle_override = 100;
const char* mode_char[] = {"空闲", "编辑", "记忆", "MDI", "DNC", "手轮", "JOG", "快速", "回零", "报警"};
static int adjust_override(int current, int delta);
static int read_key(void);
static void process_key(int key);
static void scan_key_to_event(int key);
static void print_help();
static void print_status();

// 键盘监控主函数
void hal_keyboard_minitor(void) {
    int key;
    key = read_key();
    if (key > 0) {
        process_key(key);
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
    new_termios.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
    new_termios.c_cc[VMIN] = 0;   // 非阻塞
    new_termios.c_cc[VTIME] = 1;  // 100ms超时

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    // 打印帮助信息
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
    printf("\033[2J\033[H");  // 清屏并移动光标到左上角
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║                     \033[1;36mCNC 键盘控制监控\033[0m                             ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║                          \033[1;33m按键说明\033[0m                                ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ [模式] 1:JOG  2:快速  3:手轮  4:回零  5:编辑  6:自动  7:MDI      ║\n");
    printf("║ [倍率] Q/A:进给+/-10%%  W/S:快速+/-10%%  E/D:主轴+/-10%%            ║\n");
    printf("║ [点动] U/J:+X/-X  I/K:+Y/-Y  O/L:+Z/-Z                           ║\n");
    printf("║ [主轴] Z:正转  X:停止  C:反转                                    ║\n");
    printf("║ [运行] Enter:运行  Space:暂停  R:复位  Esc:急停                  ║\n");
    printf("║ [帮助] h/H:按键点位  p/P:系统信息                                ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
}
    
// 打印当前状态，存在多线程数据竞争，此处不保证数据有效性
static void print_status(void) {
    printf("\r\033[K");  // 清除当前行
    printf("[============== CNC HMI ===================================\033[0m]\n");
    printf("[状态: \033[1;32m%s\033[0m] ",
           (var_chan[0][VAR_CHAN_MODE].value.i < 10) ? mode_char[var_chan[0][VAR_CHAN_MODE].value.i] : "未知");
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
    printf("[事件数量: %d\033[0m]\n", event_queue_get_size(event_queue));
    printf("[===========================================================\033[0m]\n");
    fflush(stdout);
}

// 处理按键事件
static void process_key(int key) {
    switch (key) {
        // 帮助与状态
        case 'h': case 'H': print_help(); break;
        case 'p': case 'P': print_status(); break;
        // 模式切换 1-7
        case '1': scan_key_to_event(KEY_EV_JOG);    break;  // JOG
        case '2': scan_key_to_event(KEY_EV_RAPID);  break;  // 快速
        case '3': scan_key_to_event(KEY_EV_MPG);    break;  // 手轮
        case '4': scan_key_to_event(KEY_EV_HOME);   break;  // 回零
        case '5': scan_key_to_event(KEY_EV_EDIT);   break;  // 编辑
        case '6': scan_key_to_event(KEY_EV_MEM);    break;  // 自动(记忆)
        case '7': scan_key_to_event(KEY_EV_MDI);    break;  // MDI
        // 进给倍率 Q/A
        case 'q': case 'Q': scan_key_to_event(KEY_EV_FO_INC); break;
        case 'a': case 'A': scan_key_to_event(KEY_EV_FO_DEC); break;
        // 快速倍率 W/S
        case 'w': case 'W': scan_key_to_event(KEY_EV_RO_INC); break;
        case 's': case 'S': scan_key_to_event(KEY_EV_RO_DEC); break;
        // 主轴倍率 E/D
        case 'e': case 'E': scan_key_to_event(KEY_EV_SO_INC); break;
        case 'd': case 'D': scan_key_to_event(KEY_EV_SO_DEC); break;
        // 点动控制 I/K O/L P/;
        case 'u': case 'U': scan_key_to_event(KEY_EV_X_POS_PRESS); break;  // +X
        case 'j': case 'J': scan_key_to_event(KEY_EV_X_NEG_PRESS); break;  // -X
        case 'i': case 'I': scan_key_to_event(KEY_EV_Y_POS_PRESS); break;  // +Y
        case 'k': case 'K': scan_key_to_event(KEY_EV_Y_NEG_PRESS); break;  // -Y
        case 'o': case 'O': scan_key_to_event(KEY_EV_Z_POS_PRESS); break;  // +Z
        case 'l': case 'L': scan_key_to_event(KEY_EV_Z_NEG_PRESS); break;  // -Z
        // 主轴控制 Z/X/C
        case 'z': case 'Z': scan_key_to_event(KEY_EV_SPINDLE_FWD); break;  // 正转
        case 'x': case 'X': scan_key_to_event(KEY_EV_SPINDLE_STOP); break; // 停止
        case 'c': case 'C': scan_key_to_event(KEY_EV_SPINDLE_REV); break;  // 反转
        // 运行控制
        case '\n': case '\r': scan_key_to_event(KEY_EV_RERUN); break;  // Enter: 运行
        case ' ':             scan_key_to_event(KEY_EV_PAUSE); break;  // Space: 暂停
        case 'r': case 'R':   scan_key_to_event(KEY_EV_RESET); break;  // R: 复位
        case 27:              scan_key_to_event(KEY_EV_ESTOP); break;  // Esc: 急停退出
        default:
            break;
    }
}

// 扫描键盘产生事件
static void scan_key_to_event(int key) {
    PMC_EVENT event;
    switch(key){
        case KEY_EV_EDIT:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 1;
            break;
        case KEY_EV_MEM:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 2;
            break;
        case KEY_EV_MDI:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 3;
            break;
        case KEY_EV_JOG:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 4;
            break;
        case KEY_EV_MPG:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 5;
            break;
        case KEY_EV_HOME:
            event.id = EV_MSG_MODE;
            event.mode_payload.mode = 6;
            break; 
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
        // todo：急停
        case KEY_EV_ESTOP:
            event.id = EV_MSG_RESET;
            event.run_payload.run_state = 1;
            break;
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
        case KEY_EV_X_POS_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 0;
            event.feedaxis_key_payload.direction = 1;
            break;
        case KEY_EV_X_NEG_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 0;
            event.feedaxis_key_payload.direction = -1;
            break;
        case KEY_EV_Y_POS_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 1;
            event.feedaxis_key_payload.direction = 1;
            break;
        case KEY_EV_Y_NEG_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 1;
            event.feedaxis_key_payload.direction = -1;
            break;
        case KEY_EV_Z_POS_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 2;
            event.feedaxis_key_payload.direction = 1;
            break;
        case KEY_EV_Z_NEG_PRESS:
            event.id = EV_MSG_FEEDAXIS_KEY;
            event.feedaxis_key_payload.axis_num = 2;
            event.feedaxis_key_payload.direction = -1;
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