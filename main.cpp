#include "stdio.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"


int Master_Start()
{
//    const char * eni_path = "./ENI/speed.xml";
//    if (ethercat_init(eni_path) < 0)
//    {
//        printf("EtherCAT 启动失败！\n");
//        return -1;
//    }
    return 1;
}

int main()
{
    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();
    Master_Start();
    return 1;
}
