#include "threads_controller.h"
#include "rt_sem.h"
#include <stdio.h>
#include "shm_write.h"
#include "master.h"
#include "taskapi.h"
#include "emcmot.h"

thread_config_t plc_config = {.core_id = 3, .policy = SCHED_RR, .priority = 32, .thread_name = "软PLC"};

thread_config_t task_config = {.core_id = 3, .policy = SCHED_RR, .priority = 30, .thread_name = "任务控制器"};

thread_config_t motion_config = {.core_id = 2, .policy = SCHED_RR, .priority = 39, .thread_name = "运动控制器"};

thread_config_t path_plan_config = {.core_id = 2, .policy = SCHED_RR, .priority = 20, .thread_name = "解释器线程"};

thread_config_t pdo_config = {.core_id = 3, .policy = SCHED_FIFO, .priority = 40, .thread_name = "PDO线程"};

thread_config_t test_keyboard_cnc_config = {.core_id = 0, .policy = SCHED_RR, .priority = 10, .thread_name = "键盘CNC测试线程"};

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&plc_sem);
        if(exit_flag) break;
        //printf("soft PLC\n");

        // 可在此处调用软PLC主函数
    }
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&motion_sem);
        if(exit_flag) break;
        //  printf("motion controller\n");
        // 可在此处调用运动控制器主函数
        emcmotTrajPlan();
    }

    return NULL;
}

// 任务控制器线程的具体实现
void* task_controller_thread_func(void*) {
    while (!exit_flag) {
        //   printf("task controller \n");
        // 可在此处调用任务控制器主函数
        taskmain();
        usleep(100);
    }
    return NULL;
}

// 路径规划线程的具体实现
void* path_plan_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&path_plan_sem);
        if(exit_flag) break;
        //   printf("parser\n");
        // 可在此处调用解释器主函数

        // 可在此处调用路径规划主函数
        emcmotPathPlan();

    }
    return NULL;
}

void* pdo_thread_func(void*)
{
    master_run();
}

void* test_keyboard_cnc_thread_func(void*)
{
    test_keyboard_cnc();
}

