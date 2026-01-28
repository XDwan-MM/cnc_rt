#include "stdio.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include <cstdio>
#include "rt_sem.h"
#include "threads_api.h"
#include "threads_controller.h"

void sigint_handler(int sig) {
    exit_flag = 1;
}
int Master_Start()
{
    const char * eni_path = "./ENI/gugao.xml";
    if (ethercat_init(eni_path) < 0) {
        printf("EtherCAT 启动失败！\n");
        return -1;
    }
    return 1;
}

int main()
{
    // 初始化共享内存
    if(!shmInit()) {
        printf("共享内存初始化失败\n");
    }
    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();
    signal(SIGINT, sigint_handler);

    Master_Start();
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
