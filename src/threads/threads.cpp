#include "thread_manager.h"
#include "threads_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// 1. 定义所有线程的实例
managed_thread_t motion_controller;
managed_thread_t task_controller;
managed_thread_t soft_plc;
managed_thread_t path_plan;
managed_thread_t pdo;

int threads_start() {
    // 检查系统CPU核心数
    long num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    printf("系统可用CPU核心数: %ld\n", num_cores);
    if (num_cores < 4) {
        fprintf(stderr, "警告: 最好在至少4核的CPU上运行以避免核心冲突。\n");
    }

    // 2. 配置运动控制器线程 (高优先级，实时性要求最高)
    motion_controller.config = motion_config;
    motion_controller.function = motion_controller_thread_func;
    motion_controller.arg = NULL; // 如果需要可以传递特定参数

    // 3. 配置软PLC线程 (中等优先级)
    soft_plc.config = plc_config;
    soft_plc.function = soft_plc_thread_func;

    // 4. 配置任务控制器线程 (较低优先级)
    task_controller.config = task_config;
    task_controller.function = task_controller_thread_func;

    // 5. 配置解释器线程
    path_plan.config = path_plan_config;
    path_plan.function = path_plan_thread_func;

    // 6. 配置pdo线程
    pdo.config = pdo_config;
    pdo.function = pdo_thread_func;

    // 7. 依次创建并启动所有线程
    printf("\n--- 开始创建控制器线程 ---\n");
    if (create_managed_thread(&pdo) != 0) return EXIT_FAILURE;
    if (create_managed_thread(&soft_plc) != 0) return EXIT_FAILURE;
    if (create_managed_thread(&motion_controller) != 0) return EXIT_FAILURE;
    if (create_managed_thread(&path_plan) != 0) return EXIT_FAILURE;
    if (create_managed_thread(&task_controller) != 0) return EXIT_FAILURE;

    printf("--- 所有线程已创建 ---\n\n");

    return EXIT_SUCCESS;
}

void threads_joint() {
    pthread_join(motion_controller.id, NULL);
    printf("%s已退出\n", motion_controller.config.thread_name);
    pthread_join(soft_plc.id, NULL);
    printf("%s已退出\n", soft_plc.config.thread_name);
    pthread_join(task_controller.id, NULL);
    printf("%s已退出\n", task_controller.config.thread_name);
    pthread_join(path_plan.id, NULL);
    printf("%s已退出\n", path_plan.config.thread_name);
    pthread_join(pdo.id, NULL);
    printf("%s已退出\n", pdo.config.thread_name);
}
