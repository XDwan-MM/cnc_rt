#include <stdio.h>
#include <cstdio>
#include <cerrno>
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include "rt_sem.h"
#include "src/plc/plctask.h"
#include "threads_api.h"
#include "threads_controller.h"
#include "debug_mode.h"
#include "global.h"
#include "comp_data.h"
#include "parser.h"
#include <hmisysapi.h>
#include "stored_stroke_check.h"
#include "master.h"
#include "log.h"
#include <QFile>
#include <QTextStream>
#include <hmisysapi.h>
#include "sysapi.h"
#ifdef DEBUG_MODE_ENABLED
#include "keyboard.h"
#endif

static char version[64];
static int sys_start_rt = 0;
static int sys_close_rt = 0;

// Ctrl + C 终止程序触发
void sigint_handler([[maybe_unused]] int sig) {
    LOG(LOG_LEVEL_INFO, "收到 Ctrl + C 按键信号");
    exit_flag = 1;
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
    // 初始化实时全局变量
    if(rt_global_init() != GNC_OPERATOR_SUCCESS){
        RT_LOG(ERROR,  "全局变量初始化失败");
        LOG(LOG_LEVEL_ERR, "全局变量初始化失败");
        return -1;
    };
    RT_LOG(INFO,  "初始化全局变量成功");
    LOG(LOG_LEVEL_INFO, "初始化全局变量成功");
    // 初始化PLC单例
    PlcTask::instance();
    RT_LOG(INFO,  "初始化PLC单例成功");
    LOG(LOG_LEVEL_INFO, "初始化PLC单例成功");
#ifdef DEBUG_MODE_ENABLED
    // 注入螺距误差补偿测试数据 (单位: um)
    // X轴: 11个测量点, 正向[0..10] + 反向[11..21]
    {
        float test_pitch[PITCH_COUNT_MAX] = {0};
        float fwd[] = {1, 10, 20};
        float rev[] = {-1, -10, -20};
        memcpy(test_pitch + 0,  fwd, sizeof(fwd));
        memcpy(test_pitch + 3, rev, sizeof(rev));
        setAllPitChData(test_pitch);
        RT_LOG(INFO, "调试模式: 已注入螺距补偿测试数据");
        LOG(LOG_LEVEL_INFO, "调试模式: 已注入螺距补偿测试数据");
    }
#endif
    // 初始化螺距误差补偿(包括同步补偿参数，和建立螺距补偿表)
    comp_data_init();
    RT_LOG(INFO, "初始化螺距误差补偿成功");
    // 初始化译码器
    parser_new();
    RT_LOG(INFO, "初始化译码器成功");
    LOG(LOG_LEVEL_INFO, "初始化译码器成功");
    // 初始化硬件抽象层
    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();
    RT_LOG(INFO, "初始化硬件抽象层成功");
    LOG(LOG_LEVEL_INFO, "初始化硬件抽象层成功");
    //初始化行程检查功能
    Axis_Limit_Manager_Init();
    signal(SIGINT, sigint_handler);
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
    // 启动主站
#ifndef DEBUG_MODE_ENABLED
    if (Master_Start() < 0) {
        RT_LOG(ERROR, "EtherCAT主站启动失败，请检查ENI文件是否正确");
        LOG(LOG_LEVEL_ERR, "EtherCAT主站启动失败，请检查ENI文件是否正确");
        return -1; // 直接退出，不再启动线程
    }
    RT_LOG(INFO, "启动EtherCAT主站");
    LOG(LOG_LEVEL_INFO, "启动EtherCAT主站");
#endif
    // 创建信号量，需要放在线程启动（threads_start函数）前
    init_rt_sem();
    // 启动各个线程，pdo线程运行master_run，给各实时线程发送信号量
    if (threads_start()) {
        RT_LOG(ERROR, "创建线程失败");
        LOG(LOG_LEVEL_ERR, "线程创建失败");
        exit(1);
    }
    RT_LOG(INFO, "创建所有线程成功");
    LOG(LOG_LEVEL_INFO, "创建所有线程成功");
    // 发送实时初始化完成信号
    setSysCommInt(SYS_START_RT_FINISH, 1, 0);
    RT_LOG(INFO, "发送实时启动完成信号");
    LOG(LOG_LEVEL_INFO, "发送实时启动完成信号");
    // 主循环
    RT_LOG(INFO, "等待实时关闭信号");
    LOG(LOG_LEVEL_INFO, "等待实时关闭信号");
    #ifndef DEBUG_MODE_ENABLED
    while (!main_exit_flag) {
        // 等待实时关闭信号
        getSysCommInt(SYS_CLOSE_RT, &sys_close_rt, 0);
        if(sys_close_rt) break;
        sleep(1);
    }
    #else
    while (!exit_flag) {
        sleep(1);
    }
    #endif
    RT_LOG(INFO, "获取到实时关闭信号");
    LOG(LOG_LEVEL_INFO, "获取到实时关闭信号");
    // 发送实时关闭完成信号
    setSysCommInt(SYS_CLOSE_RT_FINISH, 1, 0);
    RT_LOG(INFO, "发送实时关闭完成信号");
    LOG(LOG_LEVEL_INFO, "发送实时关闭完成信号");
    thread_exit_flag = 1;
    RT_LOG(INFO, "收到退出信号，关闭线程");
    LOG(LOG_LEVEL_INFO, "收到退出信号，关闭线程");
    // 唤醒所有阻塞在信号量上的线程，让它们能够检查 thread_exit_flag 并退出
    for (int i = 0; i < 3; i++) {
        post_rt_sem_all();
    }
    // 释放线程
    threads_joint();
    destroy_rt_sem();
    // 关闭主站
    int error_code = ethercat_close();
    if ( error_code < 0) {
        RT_LOG(ERROR,  "关闭EtherCAT失败，错误码 %d", error_code);
        LOG(LOG_LEVEL_ERR, "关闭EtherCAT失败，错误码 %d", error_code);
        return -1;
    }
    RT_LOG(INFO,  "退出实时程序");
    LOG(LOG_LEVEL_INFO, "退出实时程序");
    RT_LOG(INFO, "********************************************************");
    // 关闭实时日志
    RT_LOG_CLOSE();
    return 0;
}

