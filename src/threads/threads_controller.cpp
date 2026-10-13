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

/* 映像段长不再硬编码，start 后问适配层要（HAL 按各设备 Entry 位长算出）。
 *
 * 但只写 [0, x_size) 这一段，**不**对 REG_X 全量写 512 字节：PLC 用动态下标
 * （plctask.cpp 的 regX[mapping.src.idx]），索引来自运行时加载的梯形图，
 * 静态界定不了它的使用范围，全量写会清掉梯形图自己的 X 用途。
 * 这条边界要以真机梯形图为准。 */
/* 缓冲尺寸的唯一来源是适配层的 RT_HAL_IO_IMAGE_MAX（它与 REG_X/REG_Y 同长）。
 * 启动时适配层会拿 HAL 实算的段长和它比，超了直接拒绝启动。 */
#define RT_IO_IMAGE_MAX  RT_HAL_IO_IMAGE_MAX

// 软PLC线程的具体实现
void* soft_plc_thread_func(void*) {
    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        sem_wait(&plc_sem);
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) break;

        long long start_time = get_ns();

        /* 本拍周期号：开工时取走，干完原样写回。PDO 线程的完成屏障靠它确认
         * "完成的确实是第 N 拍"，光看信号量计数无法区分迟到的第 N 拍。 */
        const uint32_t plc_cycle = __atomic_load_n(&plc_cycle_target, __ATOMIC_ACQUIRE);

        uint32_t x_size = 0, y_size = 0;
        if (rt_hal_io_image_size(&x_size, &y_size) != HAL_OK) x_size = y_size = 0;
        /* 调试模式下这两个尺寸只取不写（面板/IO 由 HMI 直供 REG_X/REG_Y），
         * 但仍要取——它同时验证 HAL 侧映像已就绪，是启动期的健康检查。 */
        (void)x_size; (void)y_size;

        /* X 映像：HAL 在本拍 begin 之后已经备好，这里只取不碰总线。
         * 取回来的是完整缓冲，但只把 [0, x_size) 那一段写进 REG_X。
         *
         * **只在真机做这件事。** 调试模式下 X 的来源是内存桩里的常量
         * （rt_hal_sim_backend 的 s_panel_in / s_io_in 初始化后再无写入者），
         * 而 HMI 按 X 区约定（regdata.h 的 regX「机床 -> PLC」）把面板按键直接
         * 写进这批寄存器。每拍照搬桩的常量，等于把 HMI 写进去的按键冲掉——
         * 面板因此完全没反应。基线调试模式不写这一段，所以能工作。 */
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

        /* Y 映像：只读物理 IO 那一段，其余补 0。补 0 而不是读 REG_Y 的剩余部分，
         * 是为了让"未使用寄存器不参与输出映像"这件事显式——旧路径里那些寄存器
         * 本来也没有对应的物理输出。
         *
         * 同 X 侧：**只在真机做**。调试模式下发进桩就没了（桩的 s_panel_out /
         * s_io_out 写进去没有任何回程），HMI 也就读不到指示灯状态。留在 REG_Y
         * 让 HMI 按 regY「PLC -> 机床」自己读，与基线调试模式一致。 */
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

        /* 完成屏障，必须放在本拍最后，且**无论超时与否都要发**——超时已经报了
         * ALARM_PC，但 PDO 线程仍要往下走完这一拍，不能让整个周期卡死在这里。
         * 周期号原样写回：PDO 线程据此确认完成的是本拍，而不是迟到的上一拍。 */
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
        /* 本拍周期号：开工取走、收工写回，让 PDO 线程能确认完成的确实是本拍
         * （只看信号量计数区分不了迟到的上一拍）。 */
        const uint32_t motion_cycle = __atomic_load_n(&motion_cycle_target, __ATOMIC_ACQUIRE);
        // printf("motion_controller_thread_func");
        start = get_ns();
        /* 总线的读写在 PDO 线程的 begin/commit 里，本线程只消费快照、发布请求。 */
        hmi_events_scan();
        // plc_x_signal_scan_cycle();
        task_mq_proc();
        emcmotTrajPlan();
        /* HAL 侧的报警检查放在 emcmotTrajPlan 之后：它先处理完本拍的 NC 事件
         * （含复位请求），这里再去看 HAL 那边进展到哪了。 */
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
    /* 整圈循环在适配层里。调试与实时走同一条路径，差别只在链接进来的驱动层
     * （实时是真主站，调试是 rt_hal_sim_backend 的桩），这里没有 ifdef 分叉。 */
    const int rc = rt_hal_pdo_loop();
    /* 循环退出后本线程不再 post motion_sem，运动线程会卡在 sem_wait 上，
     * alarm_poll_hal 也就不会再跑。所以错误必须由这里自己报出去——
     * 否则总线故障只会留下一行 stderr，界面上什么都看不到。 */
    /* 正常停止不算报警：thread_exit_flag 置位后循环返回的就是 HAL_ERROR_STOPPED
     * （"已请求停止"）。原来只要 rc 非零就报，于是每次正常退出都留一条报警 122，
     * 真正该看的故障反而被它淹掉。总线故障仍然报得出去——那种情况 rc 不是 STOPPED，
     * 而且上面那段注释说的理由依然成立。 */
    if (rc && rc != HAL_ERROR_STOPPED) alarm_report_hal_error(rc);
    /* 正常停止 = 主线程已经置了 thread_exit_flag。除此之外任何退出（哪怕 rc 是 0 或
     * STOPPED）都必须告诉主线程：本线程不再 post 信号量了，运动/PLC 线程会卡住，
     * 只能由主线程统一收尾（置退出标志 → 唤醒等待者 → join → 关 HAL）。 */
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
