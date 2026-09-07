/********************************************************************
 * @file        global.h // 原为 shm.h
 * @brief       实时内部全局数据
 * @version     V1.01
 * @date        2026/01/05
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef GLOBAL_H
#define GLOBAL_H

#include <semaphore.h>
#include "plan_queue.h"
#include "event_queue.h"
#include "rt_data.h"
#include "rerun.h"
#include "reset.h"
#include "auto.h"
#include "homing.h"
#include "trajectory.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "jog.h"
#include "ferror_control.h"
#include "stored_stroke_check.h"
#include <varapi.h>
#include <parapi.h>

#define IRQ_TIMEOUT (5000)        // 周期中断超时时间(ms)（主站通讯周期超时，当超过此超时时间还未收到主站信息时，自动判定错误，结束程序）
#define TIME_MINUS_2_S (60.0)
#define TIME_US_2_S (1000000.0)
#define OVERRIDE_PERCENT (100.0)

/**
 * @brief 时间单位转换：微秒(us) 转 秒(s)
 */
#define CONV_US_TO_S(us) ((double)(us) / 1000000.0)

/**
 * @brief 长度单位转换：微米(um) 转 毫米(mm)
 */
#define CONV_UM_TO_MM(um) ((double)(um) / 1000.0)

/**
 * @brief 速度单位转换：(mm/min) 转 (mm/s)
 */
#define CONV_MM_MIN_TO_MM_S(v_min) ((double)(v_min) / 60.0)

/**
 * @brief 速度单位转换：(mm/s) 转 (mm/min)
 */
#define CONV_MM_S_TO_MM_MIN(v_s) ((double)(v_s) * 60.0)

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
extern int update_pos_flag;                    // 理论位置读取驱动器位置
extern int rc;
extern PdoWarn PdoWarnInfo; //pdo警告信息结构体
extern int slavernum;
extern uint16_t* statis;
extern uint32_t* pos;
extern uint32_t* pos_cmd;
extern uint16_t* error_code;

extern TxEntry_Unit DI0[MAX_IO_MODULES];
extern TxEntry_Unit DI1[MAX_IO_MODULES];
extern RxEntry_Unit DO0[MAX_IO_MODULES];
extern RxEntry_Unit DO1[MAX_IO_MODULES];

extern TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
extern TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
extern TxEntry_Unit CurVel[MAX_DRIVERS];          // 当前速度
extern TxEntry_Unit CurTor[MAX_DRIVERS];          // 当前转矩
extern TxEntry_Unit ErrorNumb[MAX_DRIVERS];       // 报警码

extern RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
extern RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置
extern RxEntry_Unit PurVel[MAX_DRIVERS];          // 目标速度
extern RxEntry_Unit PurTor[MAX_DRIVERS];          // 目标转矩
extern RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式


extern TxEntry_Unit MCPDI[11];
extern RxEntry_Unit MCPDO[7];

// 共享内存
extern VarData* var_data;                                    // var 变量
extern VarUnion (*var_axis)[VAR_AXIS_NUM];                   // 坐标轴变量
extern VarUnion (*var_chan)[VAR_CHAN_NUM];                   // 通道变量
extern VarUnion* var_sys;                                    // 系统变量
extern ParData* par_data;                                    // par 参数 
extern ParamUnit *par_ncu;                                   // NC 参数
extern ParamUnit *par_mac;                                   // 机床用户参数
extern ParamUnit (*par_chan)[PARAMAN_FILE_CHAN_MAX];         // 通道参数
extern ParamUnit (*par_axis)[PARAMAN_FILE_AXIS_MAX];         // 坐标轴参数
extern ParamUnit (*par_acmp)[PARAMAN_FILE_ACMP_MAX];         // 误差补偿参数
extern ParamUnit (*par_cfg)[PARAMAN_FILE_CFG_MAX];           // 设备接口参数
extern ParamUnit *par_table;                                 // 刀具和刀库参数

// 实时数据
extern RT_DATA* rt_data;       
extern RERUN_DATA* rerun_data;
extern RESET_DATA* nc_reset;
extern EVENT_QUEUE* event_queue;        // 事件队列         
extern PLAN_QUEUE *parser_queue;        // 译码队列
extern PLAN_QUEUE *pretreat_queue;      // 预处理队列
extern PLAN_QUEUE *plan_queue;          // 插补队列
extern COORD_TP *cycle_tp;              // 循环轨迹规划
extern HOMING_TP *homing_tp;            // 回零轨迹规划

// 读写共享内存锁
extern pthread_mutex_t shm_mutex;

GNC_RES rt_global_init();
GNC_RES rt_global_delete();
GNC_RES tp_reset();
void shm_var_update();

#ifdef __cplusplus
}
#endif

#endif /* GLOBAL_H */
