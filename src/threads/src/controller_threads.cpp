#include <stdio.h>
#include <unistd.h>
#include "threads_config.h"

// 软PLC线程的具体实现
void* soft_plc_thread_func(void* ) {
  // 可在该函数中调用软PLC主函数
    plc_func_ptr();
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void* ) {
    // 可在该函数中调用运动控制器主函数
    motion_func_ptr();
    return NULL;
}

// 任务控制器线程的具体实现
void* task_controller_thread_func(void* ) {
    // 可在该函数中调用任务控制器主函数
    task_func_ptr();
    return NULL;
}

// 解释器线程的具体实现
void* interpreter_thread_func(void* ) {
    // 可在该函数中调用解释器主函数
    parser_func_ptr();
    return NULL;
}
