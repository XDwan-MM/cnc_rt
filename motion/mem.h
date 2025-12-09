#ifndef __MEM_H__
#define __MEM_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        mem.h
 * @brief       记忆模式
 * @version     V1.00
 * @date        2025/10/23
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "./datadef/common.h"
#include "./kinematics/trajectory.h"
#include "pathplan.h"
#include "./datadef/plan_queue.h"
#include "./datadef/shm.h"

#define END_TOL 1E-4

typedef enum mem_exec_state
{
    EXEC_IDLE = 0,       // 空闲
    EXEC_RUNNING,        // 自动加工中
    EXEC_PAUSING,        // 正在暂停
    EXEC_CANCELING,      // 取消运动
}MEM_EXEC_STATE;

// traj plan 运动规划
typedef struct{
    PLAN_BLOCK *plan_block;  // 规划块
    VEL_PROFILE *vel_profile; // 速度曲线
    int accum_cycle; // 累计的插补周期
    double accum_distance; // 累计插补距离
    double remain_distance; // 剩余插补距离
    double current_distance; // 当前插补距离
    double target_distance; // 目标距离
    double current_vel; // 当前速度
    double target_vel; // 目标速度
}MEM_TP;

extern PARAMS *params;  // 机床参数
extern HAL_DATA *hal; // 硬件抽象层数据

extern MEM_EXEC_STATE mem_exec_state;
extern MEM_TP *pausing_tp;
extern MEM_TP *cycle_tp;
extern bool is_paused_in_block;
extern PLAN_QUEUE *plan_queue;
extern VELPLAN_TYPE velplan_type; 

void plan_tp_init();
void plan_tp_reset();
void handle_run_cmd();


#ifdef __cplusplus
}
#endif

#endif