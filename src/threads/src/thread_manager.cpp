#define _GNU_SOURCE // 必须放在所有 include 之前，用于启用 CPU 亲和性等 GNU 扩展
#include "thread_manager.h"
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void print_thread_info(const char* thread_name) {
    pthread_t self = pthread_self();
    int policy;
    struct sched_param param;

    // 1. 获取并打印调度策略和优先级
    if (pthread_getschedparam(self, &policy, &param) != 0) {
        perror("pthread_getschedparam");
        return;
    }

    printf("\n>>> [%s] 运行时检查 (ID: %lu) <<<\n", thread_name, self);
    printf("    策略: %s\n", (policy == SCHED_FIFO) ? "SCHED_FIFO (实时)" : (policy == SCHED_RR) ? "SCHED_RR (实时)" : (policy == SCHED_OTHER) ? "SCHED_OTHER (普通)" : "未知");
    printf("    优先级: %d\n", param.sched_priority);

    // 2. 获取并打印CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(self, sizeof(cpu_set_t), &cpuset) == 0) {
        printf("    运行核心: ");
        int count = 0;
        for (int i = 0; i < sysconf(_SC_NPROCESSORS_CONF); i++) {
            if (CPU_ISSET(i, &cpuset)) {
                printf("%d ", i);
                count++;
            }
        }
        printf("\n");
    } else {
        perror("pthread_getaffinity_np");
    }
    printf("----------------------------------------\n");
}

int create_managed_thread(managed_thread_t* thread) {
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
    } else {
    }
    // 设置调度策略继承方式 (必须显式设置，否则 setschedpolicy 可能无效)
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_setinheritsched 失败: %s\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    } else {
    }

    // 设置调度策略
    ret = pthread_attr_setschedpolicy(&attr, thread->config.policy);
    if (ret) {
        fprintf(stderr, "[%s] pthread_attr_setschedpolicy 失败: %s (提示: 实时策略可能需要 sudo)\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    } else {
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
        } else {
        }
    } else {
    }
    // 创建线程
    ret = pthread_create(&thread->id, &attr, thread->function, thread);
    if (ret) {
        fprintf(stderr, "[%s] pthread_create 失败: %s\n", thread->config.thread_name, strerror(ret));
        goto error_cleanup;
    } else {
    }
    printf("[Main] 线程 [%s] 创建成功 -> 核心: %d, 策略: %d, 优先级: %d\n", thread->config.thread_name, thread->config.core_id, thread->config.policy, thread->config.priority);

    // 线程分离：创建后自动回收资源，主线程不需要 join
    pthread_detach(thread->id);

    pthread_attr_destroy(&attr);
    return 0;

error_cleanup:
    pthread_attr_destroy(&attr);
    return -1;
}
