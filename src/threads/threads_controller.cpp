#include "threads_controller.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include <stdio.h>
#include "hal_driver.h"
#include "hal_io.h"
#include "regapi.h"
#include "global.h"
#include "task.h"
#include "emcmot.h"
#include "plctask.h"
#include "pmc_a_input_scanner.h"
#include "pmc_g_input_scanner.h"
#include "pmc_output_handler.h"
#include "master.h"
#include <hmisysapi.h>
#include <stdio.h>
#include <time.h>
#include <alarmapi.h>
#include "hmi_scanner.h"
#ifdef DEBUG_MODE_ENABLED
#include "master_sim.h"
#endif

#ifdef DEBUG_MODE_ENABLED
// ============ 调试模式线程配置 ============
// 使用普通调度策略，不绑定CPU核心，可进行断点调试
thread_config_t plc_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "软PLC(调试)"};
// thread_config_t task_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "任务控制器(调试)"};
thread_config_t pdo_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "PDO线程(调试)"};
thread_config_t motion_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "运动控制器(调试)"};
thread_config_t path_plan_config = {.core_id = -1, .policy = SCHED_OTHER, .priority = 0, .thread_name = "解释器线程(调试)"};
#else
// ============ 实时模式线程配置 ============
// 使用实时调度策略，绑定CPU核心，需要root权限
thread_config_t plc_config = {.core_id = 3, .policy = SCHED_RR, .priority = 32, .thread_name = "软PLC"};
// thread_config_t task_config = {.core_id = 3, .policy = SCHED_RR, .priority = 25, .thread_name = "任务控制器"};
thread_config_t pdo_config = {.core_id = 3, .policy = SCHED_FIFO, .priority = 40, .thread_name = "PDO线程"};
thread_config_t motion_config = {.core_id = 2, .policy = SCHED_FIFO, .priority = 39, .thread_name = "运动控制器"};
thread_config_t path_plan_config = {.core_id = 2, .policy = SCHED_RR, .priority = 30, .thread_name = "解释器线程"};
#endif

// 线程首末时间
static long long start = 0;
static long long end = 0;

// 获取单调时间（纳秒）
static inline long long get_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!thread_exit_flag) {
        sem_wait(&plc_sem);
        if (thread_exit_flag) break;

        long long start_time = get_ns();

        #ifndef DEBUG_MODE_ENABLED
        // 将IO和MCP输入数据写至X变量
        uint32_t io_input = 0;
        Hal_Io_Read_Di(&hal_io_module[0], &io_input);
        Hal_Io_Update(&hal_io_module[0], 1, 0);
        for (int i = 0; i < 4; i++) {
            setRegValue8(REG_X, i, (char)(io_input & 0xFF));
            io_input >>= 8;
        }
        uint16_t mcp_input = 0;
        for (int i = 0; i < 11; i++) {
            GM_TxPdoEntry_Read(MCPDI[i], &mcp_input, 2);
            setRegValue8(REG_X, i * 2 + 4, (char)(mcp_input & 0xFF));
            setRegValue8(REG_X, i * 2 + 5, (char)((mcp_input >> 8) & 0xFF));
        }
        #endif

        // Plc执行器逻辑
        PlcTask::instance().execute();

        #ifndef DEBUG_MODE_ENABLED
        // 将Y变量数据写至IO和MCP输出
        uint32_t io_output = 0;
        for (int i = 0; i < 4; i++) {
            char value = 0;
            getRegValue8(REG_Y, i, &value);
            io_output |= (uint32_t)(unsigned char)value << (i * 8);
        }
        Hal_Io_Write_Do(&hal_io_module[0], io_output);
        Hal_Io_Update(&hal_io_module[0], 0, 1);
        uint16_t mcp_output = 0;
        for (int i = 0; i < 7; i++) {
            char value1 = 0;
            getRegValue8(REG_Y, i * 2 + 4, &value1);
            char value2 = 0;
            getRegValue8(REG_Y, i * 2 + 5, &value2);
            mcp_output = (uint32_t)(unsigned char)value1 | (uint32_t)(unsigned char)value2 << 8;
            GM_RxPdoEntry_Write(MCPDO[i], mcp_output);
        }
        #endif
        
        pmc_g_input_scan_cycle();
        pmc_a_input_scan_cycle();

        #ifndef DEBUG_MODE_ENABLED
        long long end_time = get_ns();
        if(end_time - start_time > 1e6 * 0.9 ) {
            setAlarm(ALARM_PC, ALARM_ERROR, 1);
            LOG(LOG_LEVEL_ERR, "PLC线程耗时 %10f ms", (end_time - start_time) / 1e6);
        }
        #endif
    }
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void*) {
    while (!thread_exit_flag)
    {
        sem_wait(&motion_sem);
        if (thread_exit_flag) break;
        // printf("motion_controller_thread_func");
        start = get_ns();
        Hal_Driver_Update_All(true, false);
        hmi_events_scan();
        task_mq_proc();
        emcmotTrajPlan();
        pmc_output_feedback();
        Hal_Driver_Update_All(false, true);
        end = get_ns();
        // printf("%10f", (end - start) / 1e6);
        #ifndef DEBUG_MODE_ENABLED
        if(IsGreaterThan((end - start) / 1e6, 1.5)){
            int res = setAlarm(ALARM_SY, ALARM_ERROR, 1);
            if(res != -8) LOG(LOG_LEVEL_ERR, "实时超时 %10f", (end - start) / 1e6);
        }
        #endif
        sem_post(&moton_sem_exec_done);
    }
    return NULL;
}

// 路径规划线程的具体实现
void* path_plan_thread_func(void*) {
    while (!thread_exit_flag)
    {
        // if(gcode_queue_is_full(gcode_queue)){
        //     usleep(1);
        //     return NULL;
        // }
        // todo：不需要信号量
        sem_wait(&path_plan_sem);
        if (thread_exit_flag) break;
        emcmotPathPlan();
    }
    return NULL;
}

// 任务控制器线程的具体实现
// void* task_controller_thread_func(void*) {
//     while (!thread_exit_flag) {
//         // todo：修改此处写法    
//     }
//     return NULL;
// }

void* pdo_thread_func(void*) {
    #ifdef DEBUG_MODE_ENABLED
        Master_Run_Simulated();  // 调试模式使用模拟主站
    #else
        Master_Run();            // 实时模式使用真实主站
    #endif
    return NULL;
}
