#ifndef CONTROLLER_THREADS_H
#define CONTROLLER_THREADS_H

// 每个控制器线程的执行函数声明
void* soft_plc_thread_func(void* arg);
void* motion_controller_thread_func(void* arg);
void* task_controller_thread_func(void* arg);
void* interpreter_thread_func(void* arg);

#endif // CONTROLLER_THREADS_H
