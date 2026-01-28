#ifndef _ETHERCAT_H
#define _ETHERCAT_H

// =============== 支持 C++ 编译 ===============
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>



// 注意：GreeMaster 的头文件如果是纯 C 的，也要确保不会被 C++ 混淆
#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"

// =============== 程序配置参数 ===============
#define XMLPATH "./ENI/gugao.xml"  //ENI文件路径
#define SERVONUM 4                      //电机数
#define IO_NUM 1                        //IO模块数
#define CONTROL_NUM 1                   //控制面板数
#define CYCLE_TIME 1000                 //PDO线程循环周期

#define DC_EN 1                         //DC使能
#define REG_PTH_EN 0                    //是否开启REG线程，0:不开启，1:开启
#define SDO_PTH_EN 0                    //是否开启SDO线程，0:不开启，1:开启


// 可以直接访问这些全局变量

extern int slave_num;
extern int enable_data;
extern int mode_change;
extern sem_t master_sem;
extern Slave_info* slave_list;
// =============== 外部函数声明 ===============
/**
 * @brief: EtherCAT初始化函数
 * @return: 0: 成功，非0: 失败
 * @note：失败时的返回值可以在GREE_EtherCat主站函数库使用手册v1.1.403中查询到是什么意义
 */
extern int ethercat_init(void);
/**
 * @brief: EtherCAT关闭函数
 * @return: 0: 成功，非0: 失败
 * @note：失败时的返回值可以在主站使用手册中查询到是什么意义
 */
extern int ethercat_close(void);

// =============== 结束 C++ 兼容 ===============
#ifdef __cplusplus
}
#endif

#endif // _ETHERCAT_H
