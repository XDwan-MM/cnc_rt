#include "threads_controller.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include <stdio.h>
#include "hal_driver.h"
#include "task.h"
#include "emcmot.h"
#include "spindle.h"
#include "hal_task.h"
#include "pmc_input_scanner.h"
#include "pmc_output_handler.h"
#include <stdio.h>
#ifdef DEBUG_MODE_ENABLED
#include "master_sim.h"
#include "keyboard.h"
#endif

#ifdef DEBUG_MODE_ENABLED
// ============ 调试模式线程配置 ============
// 使用普通调度策略，不绑定CPU核心，可进行断点调试
thread_config_t plc_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "软PLC(调试)"};
thread_config_t task_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "任务控制器(调试)"};
thread_config_t pdo_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "PDO线程(调试)"};
thread_config_t motion_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "运动控制器(调试)"};
thread_config_t path_plan_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "解释器线程(调试)"};
#else
// ============ 实时模式线程配置 ============
// 使用实时调度策略，绑定CPU核心，需要root权限
thread_config_t plc_config = {.core_id = 3, .policy = SCHED_RR, .priority = 32, .thread_name = "软PLC"};
thread_config_t task_config = {.core_id = 3, .policy = SCHED_RR, .priority = 25, .thread_name = "任务控制器"};
thread_config_t pdo_config = {.core_id = 3, .policy = SCHED_FIFO, .priority = 40, .thread_name = "PDO线程"};
thread_config_t motion_config = {.core_id = 2, .policy = SCHED_FIFO, .priority = 39, .thread_name = "运动控制器"};
thread_config_t path_plan_config = {.core_id = 2, .policy = SCHED_RR, .priority = 30, .thread_name = "解释器线程"};
#endif

// todo：修改此处写法
bool RESET = false;

// 任务控制器线程的具体实现
void* task_controller_thread_func(void*) {
    while (!exit_flag) {
        // todo：修改此处写法    
    }
    return NULL;
}

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!exit_flag) {
        sem_wait(&plc_sem);
        if (exit_flag) break;
        #ifdef DEBUG_MODE_ENABLED
        hal_keyboard_minitor();
        #else
        hal_task();
        pmc_input_scan_cycle();
        pmc_a_input_scan_cycle();
        #endif
    }
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void*) {
    while (!exit_flag)
    {
        sem_wait(&motion_sem);
        if (exit_flag) break;
        Hal_Driver_Update_All(true, false);
        task_mq_proc();
        // todo：修改此处写法
        if (rt_data->nc_event == SYS_EVENT_RESET || RESET || rt_data->sys_state[SYS_IS_ESTOP_ON]) 
        {
            RESET = true;
            sp_sign = 0;
            if (sp_sign == 0 && !sp_stop_sem) {
                Spindle_Stop();
            }
            if (spindle.mode == DRV_MODE_DISABLED) {
                RESET = false;
            }
        }
        emcmotTrajPlan();
        pmc_output_feedback();
        Hal_Driver_Update_All(false, true);
        sem_post(&moton_sem_exec_done);
    }
    return NULL;
}

// 路径规划线程的具体实现
void* path_plan_thread_func(void*) {
    while (!exit_flag)
    {
        sem_wait(&path_plan_sem);
        if (exit_flag) break;
        emcmotPathPlan();
    }
    return NULL;
}

void* pdo_thread_func(void*) {
#ifdef DEBUG_MODE_ENABLED
    Master_Run_Simulated();  // 调试模式使用模拟主站
#else
    Master_Run();            // 实时模式使用真实主站
#endif
    return NULL;
}
