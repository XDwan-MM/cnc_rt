#ifndef __HOMING_H__
#define __HOMING_H__

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************
 * @file        homing.h
 * @brief       回零
 * @version     V1.00
 * @date        2025/11/05
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "plan_queue.h"
#include "trajectory.h"

// 绝对式回零状态
typedef enum{
    ABS_EXEC_IDLE = 0,              // 空闲
    ABS_EXEC_HOMING,                // 回零中
    ABS_EXEC_RESETTING,             // 复位中
}ABS_AXIS_HOMING_STATE;

typedef enum{
    AXIS_HOMING_NEGATIVE = 0,
    AXIS_HOMING_POSITIVE,
}AXIS_HOMING_DIR;

typedef struct{
    PLAN_BLOCK* homing_block;
    VEL_PROFILE* homing_profile;
    ABS_AXIS_HOMING_STATE homing_state;
    TRAJ_PARAMS* traj_data;
    AXIS_HOMING_DIR homing_direction;
    int interp_cycle;               // 累计的插补周期
    double homing_distance;         // 回零距离
    double accum_distance;          // 速度曲线累计插补距离
    double remain_distance;         // 速度曲线剩余插补距离
    double current_distance;        // 单周期当前插补距离
    double target_distance;         // 单周期目标距离
    double current_vel;             // 当前速度
    double target_vel;              // 目标速度
    double max_vel;                 // 最大允许速度
    bool is_event_trigger;          // 触发重新计算
    bool is_finish_at_the_end;
    bool is_resetting;
    bool is_ref_on;                 // 回零状态（1回零，0停止）
}HOMING_TP;

GNC_RES homing_tp_init();
void homing_tp_delete();
void homing_tp_reset();
void handle_homing_cmd();
void homing_handle_estop();

#ifdef __cplusplus
}
#endif

#endif
