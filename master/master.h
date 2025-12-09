#ifndef MASTER_H
#define MASTER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>

#include <GreeMasterAPI.h>
#include <gm_errcode.h>
#include "hal_driver.h"
#include "handwheel.h"

// ===============程序配置参数===============
#define XMLPATH "/home/gree/Desktop/newCNC_zz/V2.0/handwheel_3drivers/examples/ENI/3drivers.xml"   // XML文件路径
#define SLAVENUM 3              // 从站数量，目前理论上支持的最大从站数量为512
#define CYCLE_TIME 1000         // 主站通讯周期（us）
#define WAIT_TIME 20            // 信号等待时间（s）
#define IRQ_TIMEOUT 5000        // 周期中断超时时间(ms)

#define DC_EN 1                 // 是否使能DC信号
#define PARAM_CHANG 0           // 是否需要修改主站参数
#define MAN_FMMU 0              // 是否需要手动输入fmmu
#define REG_PTH_EN 0            // 是否开启reg线程
#define OP_PRE_REG_EN 0         // 是否开启握手前reg交互
#define SDO_PTH_EN 0            // 是否开启sdo线程

#define RT_POLICY SCHED_FIFO    // 线程实时调度策略
#define MAIN_PRIORITY 40        // 主线程实时优先级（0-99）
#define REG_PRIORITY 50         // REG线程实时优先级
#define SDO_PRIORITY 50         // SDO线程实时优先级

// 以下为开发时测试需要的宏
#define TEST_MODE_OP_INTER 0

// ==============全局变量声明===============
extern sem_t exit_sem;
extern sem_t op_sem;
extern int InterruptFlag;
extern Slave_info* slave_list;
extern int slave_num;

// ==============外部变量声明===============
extern TxEntry_Unit StatusWord[MAX_DRIVERS];
extern TxEntry_Unit CurPos[MAX_DRIVERS];
extern RxEntry_Unit OperationMode[MAX_DRIVERS];
extern RxEntry_Unit ControlWord[MAX_DRIVERS];
extern simple_tp_t simple_tp[5];

// ==============函数声明===============
// 主站初始化与关闭
int ethercat_init(void);
int ethercat_close(void);

// PDO控制线程
void master_run();

// SDO通用函数
int write_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry);
int read_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry);

// 线程函数

void* reg_thread_func(void* arg);
void* sdo_thread_func(void* arg);

// FMMU手动设置函数
Fmmu_Manul* FMMU_Manul_Set_Func(void);

#endif // MASTER_H
