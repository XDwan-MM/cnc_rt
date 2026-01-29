#include "stdio.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include <cstdio>
#include "rt_sem.h"
#include "threads_api.h"
#include "threads_controller.h"
#include "debug_mode.h"

void sigint_handler(int sig) {
    exit_flag = 1;
}

#ifndef DEBUG_MODE_ENABLED
// 仅在实时模式下需要启动真实主站
int Master_Start()
{
    const char * eni_path = "./ENI/gugao.xml";
    if (ethercat_init(eni_path) < 0) {
        printf("EtherCAT 启动失败！\n");
        return -1;
    }
    return 1;
}
#endif

void printMode() {
#ifdef DEBUG_MODE_ENABLED
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║         调试模式 (DEBUG MODE)            ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║  - 使用普通线程，可进行断点调试          ║\n");
    printf("║  - 使用模拟主站，无需EtherCAT硬件        ║\n");
    printf("║  - 无需root权限                          ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
#else
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║         实时模式 (REALTIME MODE)         ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║  - 使用实时线程 (SCHED_FIFO/RR)          ║\n");
    printf("║  - 连接真实EtherCAT主站                  ║\n");
    printf("║  - 需要root权限                          ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
#endif
}

int main()
{
    // 打印当前模式
    printMode();
    // 初始化共享内存
    if(!shmInit()) {
        printf("共享内存初始化失败\n");
    }
    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();
    signal(SIGINT, sigint_handler);

#ifndef DEBUG_MODE_ENABLED
    // 仅实时模式启动真实主站
    Master_Start();
#endif

    init_rt_sem(); //需要放在线程启动（threads_start()函数）前
    if (threads_start()) {
        printf("\n线程创建失败，主线程退出。\n");
        exit(true);
    }
    while (!exit_flag) {
        sleep(1);
    }
    threads_joint();

    return 0;
}
