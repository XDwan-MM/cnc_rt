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

#include "plan_queue.h"
#include "trajectory.h"

#define MEM_END_TOL 1E-3
#define LOOK_AHEAD_SIZE 200  // 速度前瞻的最大段数

typedef enum mem_exec_state
{
    EXEC_IDLE = 0,       // 空闲
    EXEC_RUNNING,        // 自动加工中
    EXEC_PAUSING,        // 正在暂停/复位
}MEM_EXEC_STATE;

// traj plan 运动规划
typedef struct{
    PLAN_BLOCK *plan_block;         // 规划块
    VEL_PROFILE *vel_profile;       // 速度曲线
    MEM_EXEC_STATE mem_exec_state;
    TRAJ_PARAMS* traj_data;
    int interp_cycle;               // 累计的插补周期
    double current_distance;        // 单个插补周期当前距离
    double target_distance;         // 单个插补周期目标距离 target_distance = accum_distance - current_distance
    double accum_distance;          // 速度曲线累计插补距离
    double remain_distance;         // 速度曲线剩余插补距离
    double current_vel;             // 当前速度
    double target_vel;              // 目标速度
    double max_vel;                 // 最大允许速度
    bool is_paused_inside_block;    // 在当段减速到0
    bool is_event_trigger;          // 触发重新计算
}COORD_TP;

GNC_RES plan_tp_init();
void plan_tp_reset();
void handle_run_cmd();

#ifdef __cplusplus
}
#endif

#endif
