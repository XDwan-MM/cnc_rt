#include "thread_manager.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

void print_thread_info(const char* thread_name) {
    pthread_t self = pthread_self();
    int policy;
    struct sched_param param;

    // 1. 获取并打印调度策略和优先级
    pthread_getschedparam(self, &policy, &param);
    printf("[%s] 线程已启动，ID: %lu\n", thread_name, self);
    printf("[%s] 内部获取：策略 = %s, 优先级 = %d\n",
           thread_name,
           (policy == SCHED_FIFO) ? "SCHED_FIFO" :
           (policy == SCHED_RR)   ? "SCHED_RR" :
           (policy == SCHED_OTHER)? "SCHED_OTHER" : "未知",
           param.sched_priority);

    // 2. 获取并打印CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(self, sizeof(cpu_set_t), &cpuset) == 0) {
        for (int i = 0; i < sysconf(_SC_NPROCESSORS_CONF); i++) {
            if (CPU_ISSET(i, &cpuset)) {
                printf("[%s] 正在核心 %d 上运行\n", thread_name, i);
            }
        }
    } else {
        perror("pthread_getaffinity_np");
    }
}


int create_managed_thread(managed_thread_t *thread) {
    pthread_attr_t attr;
    struct sched_param param;
    int ret;

    if (!thread || !thread->function) {
        fprintf(stderr, "错误: 线程配置或函数指针为空。\n");
        return -1;
    }

    // 初始化线程属性
    ret = pthread_attr_init(&attr);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_init 失败: %s\n", thread->config.thread_name, strerror(ret));
        return -1;
    }

    // 设置调度策略继承方式
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_setinheritsched 失败: %s\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    }

    // 设置调度策略
    ret = pthread_attr_setschedpolicy(&attr, thread->config.policy);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_setschedpolicy 失败: %s (提示: 可能需要root权限)\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    }

    // 设置优先级
    param.sched_priority = thread->config.priority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_setschedparam 失败: %s\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    }

    // 设置CPU核心绑定
    if (thread->config.core_id >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(thread->config.core_id, &cpuset);
        ret = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if (ret) {
            fprintf(stderr, "[%s] pthread_attr_setaffinity_np 失败: %s\n", thread->config.thread_name, strerror(ret));
            goto error_cleanup;
        }
    }

    // 创建线程，注意这里将整个 managed_thread_t 结构体作为参数传入
    // 是为了让线程函数可以访问自己的配置信息。
    ret = pthread_create(&thread->id, &attr, thread->function, thread);
    if (ret) {
        fprintf(stderr, "[%s] pthread_create 失败: %s\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    }

    printf("线程 [%s] 已创建，目标核心: %d, 策略: %s, 优先级: %d\n",
           thread->config.thread_name, thread->config.core_id,
           (thread->config.policy == SCHED_RR) ? "SCHED_RR" :
           (thread->config.policy == SCHED_FIFO) ? "SCHED_FIFO" : "未知",
           thread->config.priority);

    pthread_attr_destroy(&attr);
    return 0;

error_cleanup:
    pthread_attr_destroy(&attr);
    return -1;
}
