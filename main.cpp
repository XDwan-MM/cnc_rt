#include "stdio.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include <cstdio>
#include "rt_sem.h"

void sigint_handler(int sig) {
    exit_flag = 1;
}
int Master_Start()
{
//    const char * eni_path = "./ENI/speed.xml";
    if (ethercat_init() < 0)
    {
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
    Master_Start();
    return 0;
}
