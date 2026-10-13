#include <stdio.h>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include "shmapi.h"
#include "rt_sem.h"
#include "plctask.h"
#include "threads_api.h"
#include "threads_controller.h"
#include "debug_mode.h"
#include "global.h"
#include "comp_data.h"
#include "parser.h"
#include <hmisysapi.h>
#include "stored_stroke_check.h"
#include "rt_hal_adapter.h"
#include "log.h"
#include <QFile>
#include <QTextStream>
#include "sysapi.h"
#include <alarmapi.h>
#include "spl_control.h"

static char version[64];
static int sys_start_rt = 0;
static int sys_close_rt = 0;

/* 把机床参数域里的换算量交给适配层。值来自共享内存参数。 */
static void fill_hal_params(RtHalAxisParams params[RT_HAL_AXIS_COUNT]) {
    /* 四个 RT 轴槽，顺序与 HAL 逻辑轴号一致（X=0 Y=1 Z=2 主轴=3） */
    static const int rt_slot[RT_HAL_AXIS_COUNT] = {AXIS_X, AXIS_Y, AXIS_Z, AXIS_SP};
    memset(params, 0, sizeof(RtHalAxisParams) * RT_HAL_AXIS_COUNT);
    if (!rt_data) return;

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const RT_AXIS* a = &rt_data->axis_data[rt_slot[i]];
        /* 分子或分母为 0 = 参数没填：不装载、挂台账、禁止运动 */
        if (a->pm_pulse == 0 || a->pm_munit == 0) {
            RT_LOG(WARN, "轴 %d 的电子齿轮比为空（pm_munit=%g pm_pulse=%d），保持占位值并禁止运动",
                   rt_slot[i], a->pm_munit, a->pm_pulse);
            continue;
        }
        params[i].loaded = 1;
        /* 当量 = 用户单位/计数 = |分子|(mm) / 分母(计数)；取绝对值，符号只表方向 */
        params[i].scaling.units_per_count = std::fabs(a->pm_munit) / (double)a->pm_pulse;
        /* 每转指令脉冲数：驱动器速度对象若是 pulse/s，HAL 拿它把 rpm 折成脉冲/秒
         * （速度对象与位置命令用的是同一套指令脉冲单位）。rpm 单位时 HAL 不用它。 */
        params[i].scaling.command_pulses_per_rev = a->pm_pulse;
        params[i].scaling.enc_off = a->enc_off;
        /* 方向：分子为负 = 反向 */
        params[i].scaling.command_invert  = (a->pm_munit < 0) ? 1 : 0;
        params[i].scaling.feedback_invert = (a->pm_munit < 0) ? 1 : 0;
        /* 每转反馈计数：优先 MT_PPR，为 0 退回 pm_pulse；只主轴（模循环）用得到 */
        params[i].scaling.feedback_pulses_per_rev = a->mt_ppr > 0 ? a->mt_ppr : a->pm_pulse;
        /* 主轴转速上限：HAL 拿它做下发前的量程检查，超了就拒收。
         * 不接的话 HAL 只能用占位值 1000 rpm —— 编 S1500 会被拒收、主轴不转，
         * 而且只有一行 stderr，很难查。参数里没填时留 0，由适配层保持占位值。 */
        if (i == RT_HAL_AXIS_SP)
            params[i].max_speed = std::fabs(rt_data->axis_data[AXIS_SP].spdl_max_speed);
    }
}

// Ctrl + C 终止程序触发
void sigint_handler([[maybe_unused]] int sig) {
    RT_LOG(INFO, "收到 Ctrl + C 按键信号");
    main_exit_flag = 1;
}

int main() {  
    // 初始化实时日志
    RT_LOG_INIT();
    RT_LOG(INFO, "********************************************************");
    RT_LOG(INFO, "初始化实时日志成功");
    // 打印当前模式
    printMode();
    // 打印版本信息
    get_rt_version(version, sizeof(version));
    // 初始化共享内存
    if(!shmInit()) {
        RT_LOG(ERROR, "共享内存初始化失败");
        return -1;
    } else  {
        errno = 0;
        RT_LOG(INFO,  "共享内存初始化成功");
        LOG(LOG_LEVEL_INFO, "共享内存初始化成功");
    }
    // 等待实时开始初始化信号
    RT_LOG(INFO, "等待实时启动信号");
    LOG(LOG_LEVEL_INFO, "等待实时启动信号");
    // 如果想脱离界面进行调试，请解除注释
    // #ifdef DEBUG_MODE_ENABLED
    // setSysCommInt(SYS_START_RT, 1, 0);
    // #endif
    while (true) {
        getSysCommInt(SYS_START_RT, &sys_start_rt, 0);
        if(sys_start_rt) break;
        sleep(1);
    }
    RT_LOG(INFO, "获取到实时启动信号");
    LOG(LOG_LEVEL_INFO, "获取到实时启动信号");
    // 初始化实时全局变量
    if(rt_global_init() != GNC_OPERATOR_SUCCESS){
        RT_LOG(ERROR,  "全局变量初始化失败");
        LOG(LOG_LEVEL_ERR, "全局变量初始化失败");
        return -1;
    };
    RT_LOG(INFO,  "初始化全局变量成功");
    LOG(LOG_LEVEL_INFO, "初始化全局变量成功");
    spl_context_init(&spl_context);
    // 初始化PLC单例
    PlcTask::instance();
    RT_LOG(INFO,  "初始化PLC单例成功");
    LOG(LOG_LEVEL_INFO, "初始化PLC单例成功");
    // 同步补偿参数(同步补偿参数和建立螺距补偿表)
    comp_data_init();
    RT_LOG(INFO, "同步补偿参数和建立螺补表成功");
    if (comp_data_load_saved_compensation() != 0) {
        RT_LOG(WARN, "读取断电保存误差补偿值失败，使用当前实时补偿初始值");
        LOG(LOG_LEVEL_WARN, "读取断电保存误差补偿值失败，使用当前实时补偿初始值");
    }
    // 初始化译码器
    parser_new();
    RT_LOG(INFO, "初始化译码器成功");
    LOG(LOG_LEVEL_INFO, "初始化译码器成功");
    // 初始化硬件抽象层：装配 profile（含从机床参数取的换算量）并创建 HAL context。
    {
        RtHalAxisParams hal_params[RT_HAL_AXIS_COUNT];
        fill_hal_params(hal_params);
        char hal_err[192] = {0};
        if (rt_hal_create(hal_params, hal_err, sizeof(hal_err)) != HAL_OK) {
            RT_LOG(ERROR, "HAL 配置装配/创建失败: %s", hal_err);
            LOG(LOG_LEVEL_ERR, "HAL 配置装配/创建失败: %s", hal_err);
            return -1;
        }
    }
    RT_LOG(INFO, "初始化硬件抽象层成功");
    LOG(LOG_LEVEL_INFO, "初始化硬件抽象层成功");
    //初始化行程检查功能
    Axis_Limit_Manager_Init();
    // 启动主站。
    // 实时模式是真实主站，调试模式是 rt_hal_sim_backend。
    {
        char hal_err[192] = {0};
        if (rt_hal_start(hal_err, sizeof(hal_err)) != HAL_OK) {
            RT_LOG(ERROR, "EtherCAT主站启动失败: %s", hal_err);
            LOG(LOG_LEVEL_ERR, "EtherCAT主站启动失败: %s", hal_err);
            /* 失败回收已经创建的资源。*/
            rt_hal_shutdown();
            return -1;
        }
    }
    
    RT_LOG(INFO, "启动EtherCAT主站");
    LOG(LOG_LEVEL_INFO, "启动EtherCAT主站");
    // 创建信号量
    init_rt_sem();
    // 启动各个线程，pdo线程跑 HAL 周期循环，并给各实时线程发送信号量
    if (threads_start()) {
        RT_LOG(ERROR, "创建线程失败");
        LOG(LOG_LEVEL_ERR, "线程创建失败");
        /* 先置退出意图并唤醒所有可能已在跑的线程，再 join，最后停主站。 */
        __atomic_store_n(&thread_exit_flag, 1, __ATOMIC_RELEASE);
        for (int i = 0; i < 3; i++) post_rt_sem_all();
        threads_joint();
        rt_hal_shutdown();
        destroy_rt_sem();
        rt_global_delete();
        RT_LOG(ERROR, "实时端初始化失败，已回收资源并退出");
        RT_LOG_CLOSE();
        return -1;
    }
    RT_LOG(INFO, "创建所有线程成功");
    LOG(LOG_LEVEL_INFO, "创建所有线程成功");
    // 发送实时初始化完成信号 (通知界面：实时系统初始化完成)
    setSysCommInt(SYS_START_RT_FINISH, 1, 0);
    RT_LOG(INFO, "发送实时启动完成信号");
    LOG(LOG_LEVEL_INFO, "发送实时启动完成信号");

    // 注册ctrl+C
    signal(SIGINT, sigint_handler);

    // 主线程等待实时关闭信号
    RT_LOG(INFO, "等待实时关闭信号");
    LOG(LOG_LEVEL_INFO, "等待实时关闭信号");
    
    // 主循环
    /* 故障收尾时不要假装成功：依赖退出码的调用方要能区分正常关闭与故障终止。 */
    int fault_exit = 0;

    #ifndef DEBUG_MODE_ENABLED
    while (!main_exit_flag) {
        // 等待实时关闭信号
        getSysCommInt(SYS_CLOSE_RT, &sys_close_rt, 0);
        if(sys_close_rt) break;
        // PDO 线程意外退出（报警已经由它报过）：这里负责收尾，否则各线程会一直卡住
        if (__atomic_load_n(&pdo_unexpected_exit, __ATOMIC_ACQUIRE)) {
            LOG(LOG_LEVEL_ERR, "PDO 意外退出：按故障收尾（停机流程由主线程执行）");
            fault_exit = 1;
            break;
        }
        sleep(1);
    }
    #else
    while (!main_exit_flag) {
        if (__atomic_load_n(&pdo_unexpected_exit, __ATOMIC_ACQUIRE)) {
            LOG(LOG_LEVEL_ERR, "PDO 意外退出：按故障收尾（停机流程由主线程执行）");
            fault_exit = 1;
            break;
        }
        sleep(1);
    }
    #endif
    // 16. 开始关闭实时系统
    RT_LOG(INFO, "获取到实时关闭信号");
    LOG(LOG_LEVEL_INFO, "获取到实时关闭信号");
    // 17. 通知所有线程退出（原子写：工作线程在别的核上读它）
    __atomic_store_n(&thread_exit_flag, 1, __ATOMIC_RELEASE);
    RT_LOG(INFO, "关闭线程");
    LOG(LOG_LEVEL_INFO, "关闭线程");
    // 唤醒所有阻塞在信号量上的线程
    for (int i = 0; i < 3; i++) {
        post_rt_sem_all();
    }
    // 释放线程
    threads_joint();
#ifndef DEBUG_MODE_ENABLED
    /* 主站归 HAL 所有，下面第 219 行的 rt_hal_shutdown() 负责停掉并销毁 context，
     * 这里不再单独调 Master_Close()。 */
    RT_LOG(INFO, "关闭主站");
    LOG(LOG_LEVEL_INFO, "关闭主站");
#endif
    // 20. 保存断电误差补偿值
    // 此时实时线程已经全部退出。
    if (comp_data_save_current_compensation() != 0) {
        RT_LOG(WARN, "写入断电保存误差补偿值失败");
        LOG(LOG_LEVEL_WARN, "写入断电保存误差补偿值失败");
    }
    // 21. 销毁实时信号量
    destroy_rt_sem();
    // 停止并销毁 HAL context。
    rt_hal_shutdown();
    RT_LOG(INFO, "关闭主站");
    LOG(LOG_LEVEL_INFO, "关闭主站");
    // 释放全局变量
    rt_global_delete();
    RT_LOG(INFO, "释放全局变量");
    LOG(LOG_LEVEL_INFO, "释放全局变量");
    // 23. 通知界面：实时关闭完成
    setSysCommInt(SYS_CLOSE_RT_FINISH, 1, 0);
    RT_LOG(INFO, "发送实时关闭完成信号");
    LOG(LOG_LEVEL_INFO, "发送实时关闭完成信号");
    // 24. 退出
    RT_LOG(INFO,  "退出实时程序");
    LOG(LOG_LEVEL_INFO, "退出实时程序");
    RT_LOG(INFO, "********************************************************");
    // 关闭实时日志
    RT_LOG_CLOSE();
    /* 故障收尾过 → 非零，正常关闭 → 0。 */
    return fault_exit ? 1 : 0;
}
