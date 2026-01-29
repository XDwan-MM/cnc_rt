#include <stdio.h>
#include <cstdio>
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include "rt_sem.h"
#include "threads_api.h"
#include "threads_controller.h"
#include "debug_mode.h"

// Ctrl + C 终止程序触发
void sigint_handler([[maybe_unused]] int sig) {
    exit_flag = 1;
}

#ifndef DEBUG_MODE_ENABLED
int Master_Start() {
    const char * eni_path = "./ENI/gugao.xml";
    if (ethercat_init(eni_path) < 0) {
        printf("EtherCAT 启动失败！\n");
        return -1;
    }
    return 1;
}
#endif

void printMode() {
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
#ifdef DEBUG_MODE_ENABLED
    printf("║         调试模式 (DEBUG MODE)            ║\n");
#else
    printf("║         实时模式 (REALTIME MODE)         ║\n");
#endif
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
}

int main() {
    // 打印当前模式
    printMode();
    // 初始化共享内存
    if(!shmInit()) {
        printf("共享内存初始化失败\n");
        return -1;
    }

    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();

    signal(SIGINT, sigint_handler);

#ifndef DEBUG_MODE_ENABLED
    // 仅实时模式启动真实主站
    if (Master_Start() < 0) {
        printf("\n[严重错误] EtherCAT 主站启动失败！程序将终止。\n");
        printf("原因: 可能是 DMA 驱动未加载 (/dev/Gree_DMA0...) 或 XML 路径错误。\n");
        return -1; // 直接退出，不再启动线程
    }
#endif

    init_rt_sem(); //需要放在线程启动（threads_start()函数）前
    if (threads_start()) {
        printf("\n线程创建失败，主线程退出。\n");
        exit(1);
    }
    while (!exit_flag) {
        sleep(1);
    }
    threads_joint();

    return 0;
}
