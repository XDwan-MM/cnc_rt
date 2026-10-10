#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <pthread.h>

#include "threads_controller.h"

// 定义线程函数的指针类型
typedef void* (*thread_func_t)(void* arg);

// 通用线程管理结构体
typedef struct {
    pthread_t id;           // 线程ID
    thread_config_t config; // 线程配置
    thread_func_t function; // 线程要执行的函数
    void* arg;              // 传递给线程函数的参数 (可选)
    /* 这个线程是否真的创建成功了。threads_start() 是逐个创建的，中途失败会
     * 留下"后几个没建"。join 一个没建过的 pthread_t 是未定义行为，所以退出
     * 路径必须按这个标志挑着 join。由 threads_start() 在每个成功创建后置 1。 */
    int created;
} managed_thread_t;

/**
 * @brief 创建并启动一个被管理的线程
 *
 * @param thread 指向 managed_thread_t 实例的指针.
 *               该实例应已填充好 config 和 function 字段.
 * @return 0 表示成功, -1 表示失败.
 */
int create_managed_thread(managed_thread_t* thread);

/**
 * @brief 打印线程的调度和亲和性信息 (辅助函数)
 * @param thread_name 线程名称
 */
void print_thread_info(const char* thread_name);

#endif // THREAD_MANAGER_H
