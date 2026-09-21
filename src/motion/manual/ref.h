#ifndef __REF_H__
#define __REF_H__

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************
 * @file        ref.h
 * @brief       回参考点
 * @version     V1.01
 * @date        2026/09/12
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "plan_queue.h"
#include "trajectory.h"

// 绝对式回零状态
typedef enum{
    REF_ABS_EXEC_IDLE = 0,    // 空闲
    REF_ABS_EXEC_MOVING,      // 回零中
    REF_ABS_EXEC_PAUSING,     // 暂停中
    REF_ABS_EXEC_RESETTING,   // 复位中
}ABS_AXIS_REF_STATE;

typedef enum{
    REF_END,            // 结束
    REF_START,          // 开始
    REF_MOVING,         // 运行
    REF_PAUSED,         // 暂停 
    REF_ARRIVED,        // 到达
    REF_BREAKPOINT,     // 中断
}REF_STAGE;

typedef enum{
    REF_DO_MOVE_CMD = 0,
    REF_DO_NULL_CMD,
}REF_CMD;

typedef struct{
    VEL_PROFILE* profile;
    TRAJ_PARAMS* traj_data;         
    int interp_cycle;               // 累计的插补周期
    double accum_distance;          // 速度曲线累计插补距离
    double remain_distance;         // 速度曲线剩余插补距离
    double current_distance;        // 单周期当前插补距离
    double target_distance;         // 单周期目标距离
    double current_vel;             // 当前速度
    double target_vel;              // 目标速度
    int direction;                  // 回零方向 0负 1正
}REF_DATA;

typedef struct{
    PLAN_BLOCK ref_block;
    ABS_AXIS_REF_STATE ref_state;
    REF_STAGE current_stage; 
    REF_STAGE target_stage; 
    REF_CMD ref_cmd;
    REF_DATA ref_data;
}REF_TP;

// 手动回零
GNC_RES ref_tp_init();
void ref_tp_delete();
void ref_tp_reset();
void handle_ref_cmd();
void ref_handle_estop();

// 回参考点状态
void set_ref_state(REF_TYPE ref_type, AXIS_TYPE axis_type);
void update_ref_state();
void reset_ref_state();
void auto_set_ref_state(REF_TYPE ref_type, int axis_type);
bool check_ref_arrive(REF_TYPE ref_type, int axis_type);

#ifdef __cplusplus
}
#endif

#endif
