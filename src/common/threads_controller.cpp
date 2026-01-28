#include "threads_controller.h"
#include "rt_sem.h"
#include <stdio.h>
//#include "shm_write.h"
#include "PLC/hal_task.h"
#include "emcmot.h"
#include "greecnc.h"
#include "hal_driver.h"
#include "master.h"
#include "parserapi.h"
#include "spindle.h"
#include "taskapi.h"

thread_config_t plc_config = {.core_id = 3, .policy = SCHED_RR, .priority = 32, .thread_name = "软PLC"};

thread_config_t task_config = {.core_id = 3, .policy = SCHED_RR, .priority = 25, .thread_name = "任务控制器"};

thread_config_t pdo_config = {.core_id = 3, .policy = SCHED_FIFO, .priority = 40, .thread_name = "PDO线程"};

thread_config_t motion_config = {.core_id = 2, .policy = SCHED_FIFO, .priority = 39, .thread_name = "运动控制器"};

thread_config_t path_plan_config = {.core_id = 2, .policy = SCHED_RR, .priority = 30, .thread_name = "解释器线程"};
thread_config_t test_keyboard_cnc_config = {.core_id = 0, .policy = SCHED_RR, .priority = 20, .thread_name = "键盘CNC测试线程"};
bool RESET = false;

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&plc_sem);

        if (exit_flag) break;
        struct timespec start, end;
        long elapsed_ns;

        // 记录开始时间（CLOCK_MONOTONIC：单调时钟，避免时间回拨）
        clock_gettime(CLOCK_REALTIME, &start);

        hal_task();

        // 记录结束时间
        clock_gettime(CLOCK_REALTIME, &end);

        // 计算纳秒级差值：秒转纳秒 + 剩余纳秒
        elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);

        //     printf("soft_plc_thread_func函数执行时间: %lld ns\n", elapsed_ns);
        //        printf("soft PLC\n");
    }
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&motion_sem);
        if (exit_flag) break;

        Hal_Driver_Update_All(true, false);

        if (sys->nc_event == SYS_EVENT_RESET || RESET || sys->nc_event == SYS_EVENT_PAUSE) {
            RESET = true;
            sp_sign = 0;
            //   printf("runhere ----  %d %d\n",sp_sign ,sp_stop_sem);
            if (sp_sign == 0 && !sp_stop_sem) {
                Spindle_Stop();
            }
            if (spindle.mode == DRV_MODE_DISABLED) {
                RESET = false;
            }
        }
        emcmotTrajPlan();
        if (sys->current_mode == MODE_JOG || sys->current_mode == MODE_MPG || sys->current_mode == MODE_EDIT) {
            emcSpindleControl();
        } else if (sys->current_mode == MODE_MEM || sys->current_mode == MODE_MDI || sys->current_mode == MODE_DNC) {
            emcmcodedeal();
        }

        Hal_Driver_Update_All(false, true);
        sem_post(&moton_sem_exec_done);
    }
    return NULL;
}

// 任务控制器线程的具体实现
void* task_controller_thread_func(void*) {
    usleep(5000);
    while (!exit_flag) {
        //   printf("task controller \n");
        // 可在此处调用任务控制器主函数
        taskmain();
        usleep(1000);
    }
    return NULL;
}

// 路径规划线程的具体实现
void* path_plan_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&path_plan_sem);
        if (exit_flag) break;
        //   printf("parser\n");
        // 可在此处调用解释器主函数
        //        struct timespec start, end;
        //        long long elapsed_ns;

        // 记录开始时间（CLOCK_MONOTONIC：单调时钟，避免时间回拨）
        //  clock_gettime(CLOCK_MONOTONIC, &start);
        parserRun();
        // 可在此处调用路径规划主函数
        emcmotPathPlan();
        // 记录结束时间
        //  clock_gettime(CLOCK_MONOTONIC, &end);

        // 计算纳秒级差值：秒转纳秒 + 剩余纳秒
        // elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);

        //  printf("path_plan_thread_func函数执行时间: %lld ns\n", elapsed_ns);
    }
    return NULL;
}

void* pdo_thread_func(void*) {

    master_run();
}

void* test_keyboard_cnc_thread_func(void*) {
#ifdef test_keyboard_cnc_ON
    // test_keyboard_cnc();
#endif
}
