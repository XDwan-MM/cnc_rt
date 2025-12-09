#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include "cnc_shm.h"
#include "shm_write.h"
#include "emc.h"

// 全局CNC状态
static CncState g_cnc_state;
CncState *cnc_state = &g_cnc_state;

// 原始终端设置
static struct termios orig_termios;

// 模式名称数组
static const char* mode_names[] = { "未知","报警", "回零", "MDI", "自动","JOG点动", "手轮", "快速",  "编辑", "DNC"};


// 状态名称数组
static const char* state_names[] = {
    "开启", "急停", "急停复位", "关闭"
};
// EMC_TASK_INTERP状态名称数组
static const char* interp_names[] = {"未知", "空闲", "正在读取 G 代码", "已暂停", "等待"};

// EMC_TASK 执行状态名称数组
static const char* exec_names[] = {"未知0", "错误", "完成", "等待运动完成", "等待运动队列", "等待 IO", "未知6", "等待运动和 IO", "等待延时","等待系统命令","等待主轴定向完成"};

// 清屏
static void clear_screen() {
    printf("\033[2J\033[1;1H");
}

// 初始化CNC状态
void cnc_state_init(CncState *state) {
    state->current_mode = MODE_EDIT;
    state->spindle_state = SPINDLE_STOP;
    state->feed_override = 100;
    state->rapid_override = 100;
    state->spindle_override = 100;
    state->coolant_on = false;
    state->cycle_start = false;
    state->feed_hold = false;
    state->reset = false;
    state->emg = false;
    // 初始化轴向点动状态
    state->jog_x_pos = false;
    state->jog_x_neg = false;
    state->jog_y_pos = false;
    state->jog_y_neg = false;
    state->jog_z_pos = false;
    state->jog_z_neg = false;
    strcpy(state->last_event_msg, "系统已启动");
    state->is_running = true;
}

// 设置终端为非阻塞模式
static void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);  // 关闭回显和规范模式
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// 恢复终端设置
static void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// 检查是否有按键按下
static int kbhit() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// 获取按键
static int getch() {
    int ch = getchar();
    return ch;
}

// 显示界面
static void display_ui(CncState *state) {
   // clear_screen();

    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║              \033[1;36mCNC 控制系统测试程序\033[0m (PID: %d)                  ║\n", getpid());
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    
    // 显示最新事件
    printf("║ >> 最新事件: \033[1;33m%-50s\033[0m ║\n", state->last_event_msg);
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    
    // 显示当前模式
    printf("║ 当前模式: [\033[1;36m %-8s \033[0m] 当前任务状态：[\033[1;36m %-8s \033[0m]║\n",
           mode_names[emcStatus->task.mode],state_names[emcStatus->task.state]);
    printf("║ 当前执行状态: [\033[1;36m %-8s \033[0m] 当前interpState状态：[\033[1;36m %-8s \033[0m]║\n",
           exec_names[emcStatus->task.execState],interp_names[emcStatus->task.interpState]);
    // 主轴状态
    printf("║ 主轴状态: ");
    if (emcStatus->motion.spindle[0].spindle_state == SPINDLE_CW)
        printf("\033[32m正转 (CW) \033[0m");
    else if (state->spindle_state == SPINDLE_CCW) 
        printf("\033[32m反转 (CCW)\033[0m");
    else 
        printf("\033[31m停止      \033[0m");
    
    printf("           冷却液: %s                   ║\n", 
           emcStatus->io.coolant_on ? "\033[34mON \033[0m" : "OFF");
    
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    
    // 倍率显示
    printf("║ 进给倍率: \033[1;32m%3d%%\033[0m    快速倍率: \033[1;32m%3d%%\033[0m    主轴倍率: \033[1;32m%3d%%\033[0m        ║\n",
           state->feed_override, state->rapid_override, state->spindle_override);
    
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    
    // 状态标志
    printf("║ 循环启动: %s   进给保持: %s   复位: %s   急停: %s        ║\n",
           state->cycle_start ? "\033[32mON \033[0m" : "OFF",
           state->feed_hold ? "\033[33mON \033[0m" : "OFF",
           state->reset ? "\033[33mON \033[0m" : "OFF",
           state->emg ? "\033[1;31mON \033[0m" : "OFF");
    
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    
    // 显示轴向点动状态
    printf("║ 轴向点动: X[%s%s] Y[%s%s] Z[%s%s]                               ║\n",
           state->jog_x_neg ? "\033[32m-\033[0m" : "-",
           state->jog_x_pos ? "\033[32m+\033[0m" : "+",
           state->jog_y_neg ? "\033[32m-\033[0m" : "-",
           state->jog_y_pos ? "\033[32m+\033[0m" : "+",
           state->jog_z_neg ? "\033[32m-\033[0m" : "-",
           state->jog_z_pos ? "\033[32m+\033[0m" : "+");
    
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║                          \033[1;33m按键说明\033[0m                              ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ [模式] 1:JOG  2:快速  3:手轮  4:回零  5:编辑  6:自动  7:MDI      ║\n");
    printf("║ [倍率] Q/A:进给+/-10%%  W/S:快速+/-10%%  E/D:主轴+/-10%%          ║\n");
    printf("║ [点动] I/K:+X/-X  O/L:+Y/-Y  P/;:+Z/-Z  (JOG模式下有效)          ║\n");
    printf("║ [主轴] Z:正转  X:停止  C:反转                                    ║\n");
    printf("║ [运行] Enter:运行  Space:暂停  R:复位  Esc:急停退出              ║\n");
    printf("║ [功能] 8:冷却开  9:冷却关                                        ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
}

// 限制值在范围内
static int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 处理按键输入
static void handle_key(int key, CncState *state) {
    switch (key) {
        // 模式切换 1-7
        case '1':
            state->current_mode = MODE_JOG;
            strcpy(state->last_event_msg, "切换到 JOG点动 模式");
            break;
        case '2':
            state->current_mode = MODE_RAPID;
            strcpy(state->last_event_msg, "切换到 快速 模式");
            break;
        case '3':
            state->current_mode = MODE_MPG;
            strcpy(state->last_event_msg, "切换到 手轮 模式");
            break;
        case '4':
            state->current_mode = MODE_HOMING;
            strcpy(state->last_event_msg, "切换到 回零 模式");
            break;
        case '5':
            state->current_mode = MODE_EDIT;
            strcpy(state->last_event_msg, "切换到 编辑 模式");
            break;
        case '6':
            state->current_mode = MODE_MEM;
            strcpy(state->last_event_msg, "切换到 自动 模式");
            break;
        case '7':
            state->current_mode = MODE_MDI;
            strcpy(state->last_event_msg, "切换到 MDI 模式");
            break;
            
        // 进给倍率 Q/A
        case 'q': case 'Q':
            state->feed_override = clamp(state->feed_override + 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "进给倍率: %d%%", state->feed_override);
            break;
        case 'a': case 'A':
            state->feed_override = clamp(state->feed_override - 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "进给倍率: %d%%", state->feed_override);
            break;
            
        // 快速倍率 W/S
        case 'w': case 'W':
            state->rapid_override = clamp(state->rapid_override + 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "快速倍率: %d%%", state->rapid_override);
            break;
        case 's': case 'S':
            state->rapid_override = clamp(state->rapid_override - 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "快速倍率: %d%%", state->rapid_override);
            break;
            
        // 主轴倍率 E/D
        case 'e': case 'E':
            state->spindle_override = clamp(state->spindle_override + 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "主轴倍率: %d%%", state->spindle_override);
            break;
        case 'd': case 'D':
            state->spindle_override = clamp(state->spindle_override - 10, 0, 200);
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "主轴倍率: %d%%", state->spindle_override);
            break;
            
        // 主轴控制 Z/X/C
        case 'z': case 'Z':
            state->spindle_state = SPINDLE_CW;
            strcpy(state->last_event_msg, "主轴正转 (CW)");
            break;
        case 'x': case 'X':
            state->spindle_state = SPINDLE_STOP;
            strcpy(state->last_event_msg, "主轴停止");
            break;
        case 'c': case 'C':
            state->spindle_state = SPINDLE_CCW;
            strcpy(state->last_event_msg, "主轴反转 (CCW)");
            break;
            
        // 运行控制
        case '\n': case '\r':  // Enter - 运行
            state->cycle_start = true;
            state->feed_hold = false;
            strcpy(state->last_event_msg, "循环启动");
            break;
        case ' ':  // Space - 暂停
            state->feed_hold = !state->feed_hold;
            state->cycle_start = false;
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "进给保持: %s", state->feed_hold ? "ON" : "OFF");
            break;
        case 'r': case 'R':  // R - 复位
            state->reset = true;
            state->cycle_start = false;
            state->feed_hold = false;
            strcpy(state->last_event_msg, "系统复位");
            // 复位后清除标志
            usleep(100000);
            state->reset = false;
            break;
        case 27:  // Esc - 急停
            state->emg = true;
            state->is_running = false;
            strcpy(state->last_event_msg, "!! 急停触发 !!");
            break;
            
        // 冷却液控制
        case '8':
            state->coolant_on = true;
            strcpy(state->last_event_msg, "冷却液: ON");
            break;
        case '9':
            state->coolant_on = false;
            strcpy(state->last_event_msg, "冷却液: OFF");
            break;
            
        // 轴向点动控制 (JOG模式下有效)
        case 'i': case 'I':  // +X 正向
            state->jog_x_pos = !state->jog_x_pos;
            if (state->jog_x_pos) state->jog_x_neg = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "+X 点动: %s", state->jog_x_pos ? "ON" : "OFF");
            break;
        case 'k': case 'K':  // -X 负向
            state->jog_x_neg = !state->jog_x_neg;
            if (state->jog_x_neg) state->jog_x_pos = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "-X 点动: %s", state->jog_x_neg ? "ON" : "OFF");
            break;
        case 'o': case 'O':  // +Y 正向
            state->jog_y_pos = !state->jog_y_pos;
            if (state->jog_y_pos) state->jog_y_neg = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "+Y 点动: %s", state->jog_y_pos ? "ON" : "OFF");
            break;
        case 'l': case 'L':  // -Y 负向
            state->jog_y_neg = !state->jog_y_neg;
            if (state->jog_y_neg) state->jog_y_pos = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "-Y 点动: %s", state->jog_y_neg ? "ON" : "OFF");
            break;
        case 'p': case 'P':  // +Z 正向
            state->jog_z_pos = !state->jog_z_pos;
            if (state->jog_z_pos) state->jog_z_neg = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "+Z 点动: %s", state->jog_z_pos ? "ON" : "OFF");
            break;
        case ';': case ':':  // -Z 负向
            state->jog_z_neg = !state->jog_z_neg;
            if (state->jog_z_neg) state->jog_z_pos = false;  // 互斥
            snprintf(state->last_event_msg, sizeof(state->last_event_msg), 
                     "-Z 点动: %s", state->jog_z_neg ? "ON" : "OFF");
            break;
            
        default:
            break;
    }
}

// 主函数：键盘输入CNC测试
void test_keyboard_cnc() {
    // 初始化状态
    cnc_state_init(cnc_state);
    
    // 设置终端为原始模式
    enable_raw_mode();
    
    printf("CNC 键盘控制测试程序已启动...\n");
    sleep(1);
    
    // 主循环
    while (cnc_state->is_running) {
        // 显示界面
        display_ui(cnc_state);
        
        // 检查按键输入
        if (kbhit()) {
            int key = getch();
            handle_key(key, cnc_state);
        }
        
        // 短暂延时，降低CPU占用
        usleep(2000000);  // 5000ms
    }
    
    // 显示退出信息
    display_ui(cnc_state);
    printf("\n\033[1;41m 系统已停止 (急停触发) \033[0m\n");
    printf("按任意键退出...\n");
    
    // 恢复终端设置
    disable_raw_mode();
    
    // 等待最后一个按键
    getchar();
}
