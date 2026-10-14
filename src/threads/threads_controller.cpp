#include "threads_controller.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include <stdio.h>
#include "regapi.h"
#include "global.h"
#include "task.h"
#include "emcmot.h"
#include "plctask.h"
#include "ladtracesampler.h"
#include "servosamplesampler.h"
#include "plc_a_signal_scanner.h"
#include "plc_g_signal_scanner.h"
#include "plc_x_signal_scanner.h"
#include "plc_f_signal_handler.h"
#include "rt_hal_adapter.h"
#include <hmisysapi.h>
#include <stdio.h>
#include <time.h>
#include <alarmapi.h>
#include "hmi_scanner.h"
#include "alarm.h"

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
static int rt_overtime_count = 0;
static int plc_overtime_count = 0;

#define OVERTIME_THR 10

// 获取单调时间（纳秒）
static inline long long get_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* 缓冲尺寸的唯一来源是适配层的 RT_HAL_IO_IMAGE_MAX（与 REG_X/REG_Y 同长）；
 * 启动时适配层拿 HAL 实算的段长与它比较，超出直接拒绝启动。
 * 映像只写 [0, x_size) 这一段，**不**对 REG_X 全量写：PLC 的 regX 用运行时
 * 梯形图决定的动态下标，静态界定不了使用范围，全量写会冲掉梯形图自己的 X 用途。 */
#define RT_IO_IMAGE_MAX  RT_HAL_IO_IMAGE_MAX

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        sem_wait(&plc_sem);
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) break;

        long long start_time = get_ns();

        /* 本拍周期号：开工取走、干完原样写回；PDO 线程的完成屏障靠它确认
         * 完成的确实是第 N 拍（光看信号量计数区分不了迟到的第 N 拍）。 */
        const uint32_t plc_cycle = __atomic_load_n(&plc_cycle_target, __ATOMIC_ACQUIRE);

        uint32_t x_size = 0, y_size = 0;
        if (rt_hal_io_image_size(&x_size, &y_size) != HAL_OK) x_size = y_size = 0;
        /* 调试模式下尺寸只取不写（面板/IO 由 HMI 直供 REG_X/REG_Y），
         * 取它同时验证 HAL 侧映像已就绪。 */
        (void)x_size; (void)y_size;

        /* X 映像：HAL 在本拍 begin 之后已备好，这里只取不碰总线；
         * 只把 [0, x_size) 那段写进 REG_X。
         * **只在真机做**：调试模式下 X 的来源是内存桩里的常量，而 HMI 按 X 区约定
         * （regdata.h 的 regX「机床 -> PLC」）把面板按键直接写进这批寄存器，
         * 每拍照搬常量会把 HMI 的按键冲掉。 */
#ifndef DEBUG_MODE_ENABLED
        static uint8_t x_image[RT_IO_IMAGE_MAX];
        if (rt_hal_io_fetch_inputs(x_image, sizeof(x_image)) == HAL_OK) {
            for (uint32_t i = 0; i < x_size; ++i)
                setRegValue8(REG_X, (int)i, (char)x_image[i]);
        }
#endif

        // Plc执行器逻辑
        PlcTask::instance().execute();
        LadTraceSampler::instance().execute();

        /* Y 映像：只读物理 IO 那一段，其余显式补 0（那些寄存器本来也没有物理输出）。
         * 同 X 侧**只在真机做**：调试模式下发进桩没有回程，HMI 读不到指示灯状态；
         * 输出留在 REG_Y 让 HMI 按 regY「PLC -> 机床」自己读。 */
#ifndef DEBUG_MODE_ENABLED
        static uint8_t y_image[RT_IO_IMAGE_MAX];
        for (uint32_t i = 0; i < y_size; ++i) {
            char value = 0;
            getRegValue8(REG_Y, (int)i, &value);
            y_image[i] = (uint8_t)value;
        }
        for (uint32_t i = y_size; i < sizeof(y_image); ++i) y_image[i] = 0;
        (void)rt_hal_io_publish_outputs(y_image, sizeof(y_image));
#endif

        plc_g_signal_scan_cycle();
        plc_a_signal_scan_cycle();

        #ifndef DEBUG_MODE_ENABLED
        long long end_time = get_ns();
        if(end_time - start_time > 1e6 * 0.9 ) {
            plc_overtime_count ++;
            if(IsGreaterThan(plc_overtime_count, OVERTIME_THR)){
                int res = setAlarm(ALARM_PC, ALARM_ERROR, 1);
                if(res != -8) LOG(LOG_LEVEL_ERR, "PLC线程耗时 %10f ms", (end_time - start_time) / 1e6);
            }
        }else plc_overtime_count = 0;
        #endif

        /* 完成屏障：放在本拍最后，且**无论超时与否都要发**（超时已报 ALARM_PC，
         * 但 PDO 线程仍要往下走完这一拍）。周期号原样写回，PDO 据此确认完成的是本拍。 */
        __atomic_store_n(&plc_done_cycle, plc_cycle, __ATOMIC_RELEASE);
        sem_post(&plc_done_sem);
    }
    return NULL;
}

// 运动控制器线程的具体实现
void* motion_controller_thread_func(void*) {
    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE))
    {
        sem_wait(&motion_sem);
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) break;
        /* 本拍周期号：开工取走、收工写回，让 PDO 线程确认完成的确实是本拍。 */
        const uint32_t motion_cycle = __atomic_load_n(&motion_cycle_target, __ATOMIC_ACQUIRE);
        // printf("motion_controller_thread_func");
        start = get_ns();
        /* 总线读写在 PDO 线程的 begin/commit 里；本线程只消费快照、发布请求。 */
        hmi_events_scan();
        // plc_x_signal_scan_cycle();
        task_mq_proc();
        emcmotTrajPlan();
        /* HAL 报警检查放在 emcmotTrajPlan 之后：先处理完本拍 NC 事件（含复位请求），
         * 再看 HAL 那边进展到哪。 */
        alarm_poll_hal();
        /* 每拍把轴使能意图同步给 HAL。 */
        hal_axis_intent_poll();
        plc_f_signal_handler_cycle();
        ServoSampleSampler::instance().execute();
        end = get_ns();
        #ifndef DEBUG_MODE_ENABLED
        if(IsGreaterThan((end - start) / 1e6, 0.9)){
            rt_overtime_count ++;
            if(IsGreaterThan(rt_overtime_count, OVERTIME_THR)){
                int res = setAlarm(ALARM_SY, ALARM_ERROR, 1);
                if(res != -8) LOG(LOG_LEVEL_ERR, "运控线程超时 %10f", (end - start) / 1e6);
            } 
        }else rt_overtime_count = 0;
        #endif
        __atomic_store_n(&motion_done_cycle, motion_cycle, __ATOMIC_RELEASE);
        sem_post(&moton_sem_exec_done);
    }
    return NULL;
}

// 路径规划线程的具体实现
void* path_plan_thread_func(void*) {
    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE))
    {
        // if(gcode_queue_is_full(gcode_queue)){
        //     usleep(1);
        //     return NULL;
        // }
        // todo：不需要信号量
        sem_wait(&path_plan_sem);
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) break;
        emcmotPathPlan();
    }
    return NULL;
}

// PDO线程的具体实现
void* pdo_thread_func(void*) {
    /* 整圈循环在适配层里：调试与实时走同一条路径，差别只在链接的驱动层
     * （真主站 / rt_hal_sim_backend 的桩），这里没有 ifdef 分叉。 */
    const int rc = rt_hal_pdo_loop();
    /* 循环退出后本线程不再 post motion_sem，运动线程会卡在 sem_wait 上、
     * alarm_poll_hal 不再跑，总线故障只能由这里自己报出去。 */
    /* 正常停止（rc == HAL_ERROR_STOPPED，即已请求停止）不算报警：
     * 否则每次正常退出都会留下报警 122，把真正的故障淹掉。 */
    if (rc && rc != HAL_ERROR_STOPPED) alarm_report_hal_error(rc);
    /* 正常停止 = 主线程已置 thread_exit_flag；除此之外任何退出（哪怕 rc 是 0 或
     * STOPPED）都必须告诉主线程：本线程不再 post 信号量，只能由主线程统一收尾。 */
    if (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&pdo_unexpected_exit, 1, __ATOMIC_RELEASE);
        LOG(LOG_LEVEL_ERR, "PDO 循环意外退出：rc=%d（未收到关闭请求）", rc);
    }
    return NULL;
}

// 任务控制器线程的具体实现
// void* task_controller_thread_func(void*) {
//     while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
//         // todo：修改此处写法    
//     }
//     return NULL;
// }
