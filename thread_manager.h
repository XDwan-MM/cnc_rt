#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <pthread.h>
#include <sched.h>

// 线程配置结构体
typedef struct {
    int core_id;
    int policy;
    int priority;
    const char* thread_name;
} thread_config_t;

// 定义线程函数的指针类型
typedef void* (*thread_func_t)(void* arg);

// 通用线程管理结构体
typedef struct {
    pthread_t id;             // 线程ID
    thread_config_t config;   // 线程配置
    thread_func_t function;   // 线程要执行的函数
    void *arg;                // 传递给线程函数的参数 (可选)
} managed_thread_t;

/**
 * @brief 创建并启动一个被管理的线程
 *
 * @param thread 指向 managed_thread_t 实例的指针.
 *               该实例应已填充好 config 和 function 字段.
 * @return 0 表示成功, -1 表示失败.
 */
int create_managed_thread(managed_thread_t *thread);

/**
 * @brief 打印线程的调度和亲和性信息 (辅助函数)
 * @param thread_name 线程名称
 */
void print_thread_info(const char* thread_name);


#endif // THREAD_MANAGER_H
