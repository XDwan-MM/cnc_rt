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

#define END_TOL 1E-5

typedef enum mem_exec_state {
    EXEC_IDLE = 0,  // 空闲
    EXEC_RUNNING,   // 自动加工中
    EXEC_PAUSING,   // 正在暂停
    EXEC_CANCELING, // 取消运动
    EXEC_WAITING,   // 等待中
} MEM_EXEC_STATE;

// traj plan 运动规划
typedef struct {
    PLAN_BLOCK* plan_block;   // 规划块
    VEL_PROFILE* vel_profile; // 速度曲线
    int interp_cycle;         // 累计的插补周期
    double current_distance;  // 当前插补距离
    double target_distance;   // 目标距离（target_distance = interp_distance - current_distance）
    double current_vel;       // 当前速度
    double target_vel;        // 目标速度
    double accum_distance;    // 累计插补距离
    double remain_distance;   // 剩余插补距离
} COORD_TP;

GNC_RES plan_tp_init();
void plan_tp_reset();
void handle_run_cmd();

#ifdef __cplusplus
}
#endif

#endif
