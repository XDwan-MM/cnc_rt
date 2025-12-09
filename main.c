#include "threads_api.h"
#include "rt_sem.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "threads_controller.h"
#include "time.h"
#include "master.h"
#include "taskapi.h"
#include "shm.h"

// 模拟插补周期 s
#define CYCLE_TIME 1

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif


void sigint_handler() {
    post_rt_sem();
    exit_flag = 1;
}


// 系统初始化
void sys_data_init() {

    /************** 初始化全局数据 **************/
    // 1. 机床参数
    param_init();

    // 2. 硬件抽象层数据
    hal_init();

    // 3. plc数据
    plc_init();

    // 4. 系统数据
    sys_init();

    // 5. 加工数据
    tp_init();

    // 路径规划初始化
    char dcd_filepath[512];
    snprintf(dcd_filepath, sizeof(dcd_filepath), "%s/files/dcd.json", PROJECT_ROOT_DIR);
    gcode_queue->size = parse_dcd_file(dcd_filepath, gcode_queue);
}
int main() {
    signal(SIGINT, sigint_handler);
    sys_data_init();
    initTask();
    printf("正在启动 EtherCAT 主站...\n");
    if (ethercat_init() < 0) {
        printf("EtherCAT 启动失败！\n");
        return -1;
    }

    printf("EtherCAT 已启动，按 Ctrl+C 退出...\n");

    init_rt_sem(); //需要放在线程启动（threads_start()函数）前


    if (threads_start()) {
        printf("\n线程创建失败，主线程退出。\n");
        exit(true);
    }

    // HAL manager初始化
    printf("=========== HAL Driver Manager初始化:Hal_Driver_Manager_Init ==================\n");
    Hal_Driver_Manager_Init();

    // HAL初始化
    printf("============ HAL Driver初始化:Hal_Driver_Array_Init ================\n");
    Hal_Driver_Array_Init();

    // HAL初始化
    printf("============ HAL Io 初始化:Hal_Io_Array_Init ================\n");
    //    Hal_Io_Array_Init();



    while (!exit_flag)
    {
        sleep(1);
    }

    printf("正在关闭 EtherCAT...\n");
    ethercat_close();

    // 销毁信号量
    destroy_rt_sem();
    printf("已结束\n");

    return 0;
}
