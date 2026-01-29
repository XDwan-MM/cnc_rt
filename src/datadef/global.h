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

#include "gcode_queue.h"
#include "mcode_queue.h"
#include "plan_queue.h"
#include "event_queue.h"
#include "rt_var.h"
#include "rt_par.h"
#include "mem.h"
#include "homing.h"
#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TIME_MINUS_2_S 60
#define TIME_US_2_S 1000000

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
// 加工过程数据
extern GCODE_QUEUE *gcode_queue;        // 译码队列
extern MCODE_QUEUE *mcode_queue;        // 译码队列
extern PLAN_QUEUE *pretreat_queue;      // 预处理队列
extern PLAN_QUEUE *plan_queue;          // 插补队列
extern EVENT_QUEUE* event_queue;        // 事件队列
extern MEM_EXEC_STATE mem_exec_state;
extern COORD_TP *pausing_tp;
extern COORD_TP *cycle_tp;
extern bool is_paused_in_block;
extern HOMING_TP *homing_tp;
extern TRAJ_PARAMS* traj_data;
extern TRAJ_PARAMS* free_traj_data;
extern bool is_paused_in_block;

// 初始化实时var数据
GNC_RES rt_var_init(); 
// 初始化共享内存var数据
GNC_RES shm_var_init();
// 更新共享内存var数据
GNC_RES shm_var_update();
// 更新共享内存程序相关数据
GNC_RES shm_var_update_prog();
// 初始化实时par数据
GNC_RES rt_par_init(); 
// 初始化共享内存par数据
GNC_RES shm_par_init();
// 更新共享内存par数据
GNC_RES rt_par_update();    // 每个插补周期都会更新的par数据
// 复位更新共享内存var数据
GNC_RES rt_par_reset();     // 按下复位后更新的par数据
GNC_RES sys_init();
GNC_RES sys_reset();
GNC_RES tp_init();
GNC_RES tp_reset();
GNC_RES traj_data_init();

#ifdef __cplusplus
}
#endif

#endif
