#ifndef THREADS_CONTROLLER_H
#define THREADS_CONTROLLER_H

#include <sched.h>
#include <semaphore.h>
#include "threads_api.h"

#define debugflag 1
#define MIN_CPU_CORE_NUM 4 // 最小cpu核心数

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
extern thread_config_t path_plan_config;
extern thread_config_t pdo_config;
extern thread_config_t test_keyboard_cnc_config;

// 每个控制器线程的执行函数声明
void* soft_plc_thread_func(void* arg);
void* motion_controller_thread_func(void* arg);
void* task_controller_thread_func(void* arg);
void* path_plan_thread_func(void* arg);
void* pdo_thread_func(void*);
void* test_keyboard_cnc_thread_func(void*);

#endif // THREADS_CONTROLLER_H
