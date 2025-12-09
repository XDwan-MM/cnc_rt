#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

// ---定义报警级别 ---
typedef enum {
    Alarm_INFO, //信息
    Alarm_WARNING, //警告
    Alarm_ERROR, //错误
    Alarm_FATAL //致命错误
} AlarmSeverity;

// 定义报警列表的最大容量
#define MAX_ALARMS 10000

// 定义各字符串字段的长度
#define ALARM_CODE_LEN 16 //报警代码长度
#define COMPONENT_LEN 32 //组件名称长度
#define MESSAGE_LEN 128 //报警消息长度
#define DETAILS_LEN 256 //报警详情长度

// 统一的报警数据结构
typedef struct {
    char alarm_code[ALARM_CODE_LEN]; //报警代码
    char component[COMPONENT_LEN];   //组件名称
    char message[MESSAGE_LEN];       //报警消息
    AlarmSeverity severity;         //报警级别
    time_t timestamp;               //报警时间戳
    char details[DETAILS_LEN];      //报警详情
    int is_active; // 用于软删除或清除报警的标志位
} Alarm;

// 报警管理器核心结构体
typedef struct {
    pthread_mutex_t lock;                 // 用于保护所有共享数据的互斥锁

    // --- 核心机制：序列号 ---
    // 每当报警状态改变，此序列号就会增加。这是“命令”侧的序列号。
    unsigned long long command_serial_number;

    // --- 核心机制：循环缓冲区 ---
    Alarm alarm_buffer[MAX_ALARMS];       // 固定大小的报警存储数组
    int head;                             // 指向下一个可写入位置的索引（头部）
    int count;                            // 缓冲区中当前有效的报警数量
} AlarmManager;

// 创建全局唯一的报警管理器实例
static AlarmManager g_alarm_manager;

// --- 报警管理器的核心函数 ---

/**
 * @brief 初始化报警管理器。程序启动时必须调用一次。
 */
static void alarm_manager_init() {
    // 初始化互斥锁
    pthread_mutex_init(&g_alarm_manager.lock, NULL);
    // 初始化序列号和循环缓冲区参数
    g_alarm_manager.command_serial_number = 0;
    g_alarm_manager.head = 0;
    g_alarm_manager.count = 0;
    // 将整个缓冲区清零，确保没有脏数据
    memset(g_alarm_manager.alarm_buffer, 0, sizeof(g_alarm_manager.alarm_buffer));
}

/**
 * @brief 任何组件线程调用此函数来报告一个新报警。
 */
static void report_alarm(const char* code, const char* component, const char* message) {
    pthread_mutex_lock(&g_alarm_manager.lock); // --- 上锁，保护共享数据 ---

    // 检查此报警是否已经在活动列表中，防止重复添加
    int already_exists = 0;
    for (int i = 0; i < g_alarm_manager.count; i++) {
        // 计算在循环缓冲区中的实际索引
        int index = (g_alarm_manager.head - 1 - i + MAX_ALARMS) % MAX_ALARMS;
        if (g_alarm_manager.alarm_buffer[index].is_active &&
            strcmp(g_alarm_manager.alarm_buffer[index].alarm_code, code) == 0 &&
            strcmp(g_alarm_manager.alarm_buffer[index].component, component) == 0) {
            already_exists = 1;
            break;
        }
    }

    if (!already_exists) {
        // 在“头部”位置写入新的报警数据
        Alarm* new_alarm_slot = &g_alarm_manager.alarm_buffer[g_alarm_manager.head];
        strncpy(new_alarm_slot->alarm_code, code, ALARM_CODE_LEN - 1);
        strncpy(new_alarm_slot->component, component, COMPONENT_LEN - 1);
        strncpy(new_alarm_slot->message, message, MESSAGE_LEN - 1);
        new_alarm_slot->timestamp = time(NULL);
        new_alarm_slot->is_active = 1;

        printf("--- 新报警产生 ---\n  > [%s] %s: %s\n", component, code, message);

        // 移动头部指针，如果到达末尾则回到开头 (循环)
        g_alarm_manager.head = (g_alarm_manager.head + 1) % MAX_ALARMS;

        // 如果缓冲区未满，则计数加一
        if (g_alarm_manager.count < MAX_ALARMS) {
            g_alarm_manager.count++;
        }
        // 如果缓冲区已满，count不变，最旧的数据被自动覆盖

        // 关键步骤：报警状态已改变，递增序列号
        g_alarm_manager.command_serial_number++;
    }

    pthread_mutex_unlock(&g_alarm_manager.lock); // --- 解锁 ---
}

/**
 * @brief 清除一个指定的报警（通过将其标记为不活动）。
 */
static void clear_alarm(const char* code, const char* component) {
    pthread_mutex_lock(&g_alarm_manager.lock); // --- 上锁 ---

    int cleared = 0;
    // 遍历所有有效的报警
    for (int i = 0; i < g_alarm_manager.count; i++) {
        int index = (g_alarm_manager.head - 1 - i + MAX_ALARMS) % MAX_ALARMS;
        if (g_alarm_manager.alarm_buffer[index].is_active &&
            strcmp(g_alarm_manager.alarm_buffer[index].alarm_code, code) == 0 &&
            strcmp(g_alarm_manager.alarm_buffer[index].component, component) == 0) {

            // 这里我们采用“软删除”，只改变标志位，而不是重建数组，这样更高效
            g_alarm_manager.alarm_buffer[index].is_active = 0;
            cleared = 1;

            printf("--- 报警已清除 ---\n  > [%s] %s\n", component, code);
            break; // 找到并清除后退出循环
        }
    }

    if (cleared) {
        // 关键步骤：报警状态已改变，递增序列号
        g_alarm_manager.command_serial_number++;
    }

    pthread_mutex_unlock(&g_alarm_manager.lock); // --- 解锁 ---
}

/**
 * @brief HMI调用的函数，用于获取当前所有活动报警的副本。
 * @param buffer - 调用者提供的用于存储报警的数组。
 * @param buffer_size - 缓冲区的最大容量。
 * @return 实际复制到buffer中的活动报警数量。
 */
static int get_active_alarms(Alarm* buffer, int buffer_size) {
    pthread_mutex_lock(&g_alarm_manager.lock); // --- 上锁 ---

    int active_count = 0;
    // 从最新的报警开始遍历（即头部的前一个位置）
    for (int i = 0; i < g_alarm_manager.count && active_count < buffer_size; i++) {
        int index = (g_alarm_manager.head - 1 - i + MAX_ALARMS) % MAX_ALARMS;
        // 只复制那些仍然处于“活动”状态的报警
        if (g_alarm_manager.alarm_buffer[index].is_active) {
            memcpy(&buffer[active_count], &g_alarm_manager.alarm_buffer[index], sizeof(Alarm));
            active_count++;
        }
    }

    pthread_mutex_unlock(&g_alarm_manager.lock); // --- 解锁 ---
    return active_count;
}

// --- 使用方法 ---
//report_alarm("SV0401", component, "伺服过载");
// report_alarm("EX1002", "PLC", "气压不足");
//在main函数中需要执行alarm_manager_init();
/*人机交互调用方法*/
/*
// HMI维护的本地状态，用于与管理器状态进行比较
unsigned long long hmi_echo_serial_number = 0; // HMI侧的“回显”序列号
Alarm hmi_alarm_cache[MAX_ALARMS];             // HMI本地的报警列表缓存
int hmi_alarm_count = 0;

   // HMI主循环
     //   --- HMI的核心判断逻辑 ---
     //   1. 获取管理器当前的序列号（可封装成一个getter)
        unsigned long long manager_serial = g_alarm_manager.command_serial_number;

     //   2. 比较HMI的本地序列号与管理器的序列号
        if (manager_serial != hmi_echo_serial_number) {
            printf(">> 检测到状态变化 (Manager: %llu, HMI: %llu)。正在更新HMI...\n",
                   manager_serial, hmi_echo_serial_number);

            // 3. 只有在状态变化时，才去获取完整的报警列表
            hmi_alarm_count = get_active_alarms(hmi_alarm_cache, MAX_ALARMS);

            // 4. 更新HMI的本地序列号，使其与管理器同步
            hmi_echo_serial_number = manager_serial;

        } else {
            printf(">> 状态未变 (序列号: %llu)，无需更新数据。\n", manager_serial);
        }
*/

//任务控制器处理活动报警的相应方式
/*
        Alarm active_alarms_buffer[MAX_ALARMS];
        int count = get_active_alarms(active_alarms_buffer, MAX_ALARMS);

        AlarmSeverity highest_severity = INFO; // 默认为最低级别

        // 找出当前最高的报警级别
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                if (active_alarms_buffer[i].severity > highest_severity) {
                    highest_severity = active_alarms_buffer[i].severity;
                }
            }
        }

        // 根据最高级别执行动作
        switch (highest_severity) {
        case FATAL:
            // 触发紧急停止电路，这通常会操作硬件IO
            trigger_emergency_stop();
            break;
        case ERROR:
            // 向运动控制器发送指令
            motion_controller_feed_hold();
            spindle_controller_stop();
            break;
        case WARNING:
            // 点亮HMI上的警告灯
            hmi_set_warning_light(true);
            break;
        case INFO:
            // 如果之前有警告，现在可以熄灭警告灯
            hmi_set_warning_light(false);
            break;
        }
}
*/
