#ifndef _ETHERCAT_H
#define _ETHERCAT_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdbool.h>
#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"
#include "slave_list.h"
// ===============程序配置参数===============    
#define CYCLE_TIME 1000         // 主站通讯周期（us）
#define WAIT_TIME 120            // 信号等待时间（s）（包括preop信号、op信号、eeprom信号、pdo映射获取信号等其他需要等待的场景）
#define IRQ_TIMEOUT 5000        // 周期中断超时时间(ms)（主站通讯周期超时，当超过此超时时间还未收到主站信息时，自动判定错误，结束程序）
#define SLAVENUM 4

#define DC_EN 1                 // 是否使能DC信号
#define PARAM_CHANG 0          // 是否需要修改主站参数
#define MAN_FMMU 0              // 是否需要手动输入fmmu
#define REG_PTH_EN 0            // 是否开启reg线程
#define OP_PRE_REG_EN  0        // 是否开启握手前reg交互，如开启请确保已经开启了reg线程
#define SDO_PTH_EN  0           // 是否开启sdo线程

#define RT_POLICY SCHED_FIFO    // 线程实时调度策略，例如 FIFO
#define MAIN_PRIORITY 40        // 主线程实时优先级（0-99，99为最高）
#define REG_PRIORITY 50         // 未生效，REG线程实时优先级（0-99，99为最高）
#define SDO_PRIORITY 50         // 未生效，SDO线程实时优先级（0-99，99为最高）
// 以下为开发时测试需要的宏，作为展示例程时视为全0（关闭）
#define TEST_MODE_OP_INTER 0

enum
{
    MOTOR_START = 0,
    MOTOR_RUN,
    MOTOR_STOP
};

enum { CREAR_ERROR_0 = 0, CREAR_ERROR_1 };

// 假设 Err_info 已定义
// #include "your_system_header.h" 

// ==========================================
// 1. 传递给主线程的结构体
// ==========================================
typedef struct {
    int Model;   // 报错模块
    int Fms;     // 报错状态机
    int Code;    // 报错索引码
    int errslave;
    int ErrorID; // 预定义的枚举ID (可选)
} ErrorContext;

// ==========================================
// 2. 全局变量声明 (extern)
// ==========================================

// 【关键】声明全局错误上下文，其他 .c 文件可以直接访问
extern ErrorContext g_current_error_ctx;


// ==========================================
// 3. 公共接口声明
// ==========================================

void error_module_init(void);
void error_module_destroy(void);

typedef enum _EcatMasterError
{
    ECAT_INIT_ERROR = 0x1001,   /*!< 主站初始化模块错误 */
    ECAT_CONTROL_PANEL_CONNECT_LOST = 0x1002,   /*!< 控制面板断连 */
    ECAT_IO_CONNECT_LOST = 0x1003,   /*!< IO模块断连 */
    ECAT_SLAVE3_CONNECT_LOST = 0x1004,   /*!< 从站三断连 */
    ECAT_SLAVE4_CONNECT_LOST = 0x1005,   /*!< 从站四断连 */
    ECAT_SLAVE5_CONNECT_LOST = 0x1006,   /*!< 从站五断连 */
    ECAT_SLAVE6_CONNECT_LOST = 0x1007,   /*!< 从站六断连 */
    ECAT_TOPOLOGY_ERROR = 0x1008,   /*!< 主站拓扑模块错误 */
    ECAT_ACTIVE_ERROR = 0x1009,   /*!< 主站激活模块错误 */
    ECAT_DC_ERROR = 0x100A,   /*!< 主站DC模块错误 */
    ECAT_PDO_ERROR = 0x100B,   /*!< 主站PDO模块错误 */
    ECAT_RT_ERROR = 0x100C,   /*!< 主站RT模块错误 */
    UNKOWN_ERROR = 0xFFFF    /*!< 未知错误 */
} EcatMasterError;

extern int slave_num;
extern int enable_data;
extern int current_mode;// 0 : 速度模式，1 ： 位置模式
extern sem_t master_sem;
// =============== 外部函数声明 ===============
/**
 * @brief: EtherCAT初始化函数
 * @return: 0: 成功，非0: 失败
 * @note：失败时的返回值可以在GREE_EtherCat主站函数库使用手册v1.1.403中查询到是什么意义
 */
extern int ethercat_init();
/**
 * @brief: EtherCAT关闭函数
 * @return: 0: 成功，非0: 失败
 * @note：失败时的返回值可以在主站使用手册中查询到是什么意义
 */
extern int ethercat_close(void);
#endif
