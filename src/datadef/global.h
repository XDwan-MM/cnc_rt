/********************************************************************
 * @file        global.h // 原为 shm.h
 * @brief       实时内部全局数据
 * @version     V1.01
 * @date        2026/01/05
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __GLOABL_H__
#define __GLOABL_H__

#include <semaphore.h>
#include "gcode_queue.h"
#include "mcode_queue.h"
#include "plan_queue.h"
#include "event_queue.h"
#include "rt_var.h"
#include "rt_par.h"
#include "mem.h"
#include "homing.h"
#include "trajectory.h"
#include "hal_driver.h"
#include "hal_io.h"

#define IRQ_TIMEOUT 5000        // 周期中断超时时间(ms)（主站通讯周期超时，当超过此超时时间还未收到主站信息时，自动判定错误，结束程序）
#define TIME_MINUS_2_S 60
#define TIME_US_2_S 1000000

/**
 * @brief 长度单位转换：微米(um) 转 毫米(mm)
 */
#define CONV_UM_TO_MM(um) ((double)(um) / 1000.0)
/**
 * @brief 速度单位转换：(mm/min) 转 (mm/s)
 */
#define CONV_MM_MIN_TO_MM_S(v_min) ((double)(v_min) / 60.0)
/**
 * @brief 加工加减速时间常数(ms) 转换为 加工加速度(mm/s^2)
 * @param T_MS 加工加减速时间常数，单位：ms
 * @return 返回加速度值，单位：mm/s^2；若输入 <= 0 则返回 0.0
 */
#define CONV_T_MS_TO_ACCEL(T_MS) (((double)(T_MS) > 0.0) ? (50000.0 / (3.0 * (double)(T_MS))) : 0.0)
/**
 * @brief 加工加减速捷度时间常数(ms) 转换为 加工捷度(mm/s^3)
 * @param T_MS 加工加减速捷度时间常数，单位：ms
 * @return 返回加工捷度，单位：mm/s^3；若输入 <= 0 则返回 0.0
 */
#define CONV_T_MS_TO_JERK(T_MS) (((double)(T_MS) > 0.0) ? (1000000.0 / (double)(T_MS)) : 0.0)

#ifdef __cplusplus
extern "C" {
#endif


extern sem_t exit_sem;                         // 退出信号
extern sem_t op_sem;                           // 已进入op信号，用于线程间通信
extern int InterruptFlag;                      // 打断标志位
extern Slave_info* slave_list;                 // 定义从站信息结构体以存储解析结果

extern TxEntry_Unit DI0[MAX_IO_MODULES];
extern TxEntry_Unit DI1[MAX_IO_MODULES];
extern RxEntry_Unit DO0[MAX_IO_MODULES];
extern RxEntry_Unit DO1[MAX_IO_MODULES];

extern TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
extern RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式
extern TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
extern TxEntry_Unit CurVel[MAX_DRIVERS];          // 当前速度
extern TxEntry_Unit CurTor[MAX_DRIVERS];          // 当前转矩

extern RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
extern RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置
extern RxEntry_Unit PurVel[MAX_DRIVERS];          // 目标速度
extern RxEntry_Unit PurTor[MAX_DRIVERS];          // 目标转矩

extern RxEntry_Unit ErrorNumb[MAX_DRIVERS];       // 报警码

extern TxEntry_Unit MCPDI[11];
extern RxEntry_Unit MCPDO[7];

// var 变量
extern RT_VAR* var_data;
extern VAR_AXIS* var_axis;          // 坐标轴变量
extern VAR_CHANNEL* var_chan;       // 通道变量
extern VAR_SYSTEM var_sys;          // 系统变量
// par 参数
extern PAR* par_data;
extern PAR_NCU par_ncu;            // NC 参数
extern PAR_MAC par_mac;            // 机床用户参数
extern PAR_CHAN* par_chan;         // 通道参数
extern PAR_AXIS* par_axis;         // 坐标轴参数
extern PAR_ACMP* par_acmp;         // 误差补偿参数
extern PAR_CFG par_cfg;            // 设备接口参数
extern PAR_TABLE par_table;        // 刀具和刀库参数
// 系统数据
extern SYS_DATA *sys;
extern EVENT_QUEUE* event_queue;    // 事件队列
// 加工过程数据
extern GCODE_QUEUE *gcode_queue;    // 译码队列
extern MCODE_QUEUE *mcode_queue;    // 译码队列
extern PLAN_QUEUE *pretreat_queue;  // 预处理队列
extern PLAN_QUEUE *plan_queue;      // 插补队列
extern COORD_TP *cycle_tp;
extern HOMING_TP *homing_tp;

GNC_RES rt_global_init();
GNC_RES tp_reset();
GNC_RES sys_reset();
// 更新共享内存par数据
GNC_RES rt_par_update();    // 每个插补周期都会更新的par数据
// 复位更新共享内存var数据
GNC_RES rt_par_reset();     // 按下复位后更新的par数据
// 更新共享内存var数据
GNC_RES shm_var_update();
// 更新共享内存程序相关数据
GNC_RES shm_var_update_prog();

#ifdef __cplusplus
}
#endif

#endif
