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

#define HOME_TOL 1E-5 // 回零偏差，单位mm（与驱动器分辨率相关，与跟随误差相关）

// 绝对式回零状态
typedef enum {
    ABS_EXEC_IDLE = 0, // 空闲
    ABS_EXEC_HOMING,   // 回零中
    ABS_EXEC_CANCELING,
} ABS_AXIS_HOMING_STATE;

typedef enum {
    AXIS_HOMING_NEGATIVE = 0,
    AXIS_HOMING_POSITIVE,
} AXIS_HOMING_DIR;

typedef struct {
    PLAN_BLOCK* homing_block;
    VEL_PROFILE* homing_profile;
    ABS_AXIS_HOMING_STATE homing_state;
    int interp_cycle;        // 累计的插补周期
    double accum_distance;   // 累计插补距离
    double remain_distance;  // 剩余插补距离
    double current_distance; // 当前插补距离
    double current_vel;      // 当前速度
    double target_vel;       // 目标速度
    double target_distance;  // 目标距离
    AXIS_HOMING_DIR homing_direction;
    bool is_axis_homed;
} HOMING_TP;

GNC_RES homing_tp_init(HOMING_TP* homing_tp);
void homing_tp_reset(HOMING_TP* homing_tp);
void handle_homing_cmd();

#ifdef __cplusplus
}
#endif

#endif
