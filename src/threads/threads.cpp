#include "thread_manager.h"
#include "threads_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hmisysapi.h>

// 1. 定义所有线程的实例
managed_thread_t motion_controller;
// managed_thread_t task_controller;
managed_thread_t soft_plc;
managed_thread_t path_plan;
managed_thread_t pdo;

int threads_start() {
    // 检查系统CPU核心数
    long num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    LOG(LOG_LEVEL_INFO, "系统可用CPU核心数: %ld", num_cores);
    if (num_cores < 4) {
        LOG(LOG_LEVEL_WARN, "请在至少4核的CPU上运行，避免核心冲突");
    }

    // 2. 配置运动控制器线程 (高优先级，实时性要求最高)
    motion_controller.config = motion_config;
    motion_controller.function = motion_controller_thread_func;
    motion_controller.arg = NULL; // 如果需要可以传递特定参数

    // 3. 配置软PLC线程 (中等优先级)
    soft_plc.config = plc_config;
    soft_plc.function = soft_plc_thread_func;

    // 4. 配置任务控制器线程 (较低优先级)
    // task_controller.config = task_config;
    // task_controller.function = task_controller_thread_func;

    // 5. 配置解释器线程
    path_plan.config = path_plan_config;
    path_plan.function = path_plan_thread_func;

    // 6. 配置pdo线程
    pdo.config = pdo_config;
    pdo.function = pdo_thread_func;

    // 7. 依次创建并启动所有线程
    if (create_managed_thread(&pdo) != 0) return EXIT_FAILURE;
    LOG(LOG_LEVEL_INFO, "创建pdo线程成功");
    if (create_managed_thread(&soft_plc) != 0) return EXIT_FAILURE;
    LOG(LOG_LEVEL_INFO, "创建plc线程成功");
    if (create_managed_thread(&motion_controller) != 0) return EXIT_FAILURE;
    LOG(LOG_LEVEL_INFO, "创建运动线程成功");
    if (create_managed_thread(&path_plan) != 0) return EXIT_FAILURE;
    LOG(LOG_LEVEL_INFO, "创建路径规划线程成功");
    // if (create_managed_thread(&task_controller) != 0) return EXIT_FAILURE;

    return EXIT_SUCCESS;
}

void threads_joint() {
    pthread_join(motion_controller.id, NULL);
    LOG(LOG_LEVEL_INFO, "退出运动线程");
    pthread_join(soft_plc.id, NULL);
    LOG(LOG_LEVEL_INFO, "退出plc线程");
    // pthread_join(task_controller.id, NULL);
    // LOG(LOG_LEVEL_INFO, "退出任务线程");
    pthread_join(path_plan.id, NULL);
    LOG(LOG_LEVEL_INFO, "退出路径规划线程");
    pthread_join(pdo.id, NULL);
    LOG(LOG_LEVEL_INFO, "退出pdo线程");
}
