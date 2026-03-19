#ifndef THREADS_CONTROLLER_H
#define THREADS_CONTROLLER_H

#include <sched.h>
#include <semaphore.h>
#include "threads_api.h"
#include "threads_config.h"  // 使用统一的类型定义
#include "master.h"

// 声明所有全局变量
extern thread_config_t plc_config;
extern thread_config_t task_config;
extern thread_config_t motion_config;
extern thread_config_t path_plan_config;
extern thread_config_t pdo_config;

// 每个控制器线程的执行函数声明
void* soft_plc_thread_func(void* arg);
void* motion_controller_thread_func(void* arg);
void* task_controller_thread_func(void* arg);
void* path_plan_thread_func(void* arg);
void* pdo_thread_func(void*);

#endif // THREADS_CONTROLLER_H
