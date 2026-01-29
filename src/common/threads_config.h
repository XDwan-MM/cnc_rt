#ifndef THREADS_CONFIG_H
#define THREADS_CONFIG_H

#include <sched.h>

#define MIN_CPU_CORE_NUM 4  // 最小cpu核心数

// 结构体定义
typedef struct {
    int core_id;
    int policy;
    int priority;
    const char* thread_name;
} thread_config_t;

// 声明所有全局变量
extern thread_config_t plc_config;
extern thread_config_t task_config;
extern thread_config_t motion_config;
extern thread_config_t parser_config;

// 声明函数指针
extern void (*plc_func_ptr)();
extern void (*task_func_ptr)();
extern void (*motion_func_ptr)();
extern void (*parser_func_ptr)();

// 函数声明
void PLC_run();
void task_run();
void motion_run();
void parser_run();

#endif // THREADS_CONFIG_H
