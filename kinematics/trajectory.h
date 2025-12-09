#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        trajectory.h
 * @brief       轨迹规划
 * @version     V1.00
 * @date        2025/10/15
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include "./datadef/common.h"
#include "./math/cnc_math.h"
#include "./datadef/plan_queue.h"

#define TOL 1E-6;

typedef struct profile
{
    double t[7];
    double v[7];
    double s[7];
    double vs;
    double ve;
    double vpeak;
    double ata;
    double atd;
    double jta;
    double jtd;
    double t_cycle;
    double s1_;
    double s5_;
}VEL_PROFILE;

typedef struct round_profile
{
    double T[7];
    double S[7];
    double vpeak;
    double ata;
    double atd;
    double jta;
    double jtd;
    double T_cycle;
    double de;
}R_VEL_PROFILE;


/// @brief 计算加减速曲线
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param plan_block 插补块
/// @return 加减速曲线
VEL_PROFILE* cal_vel_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block);

/// @brief 计算插补距离
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param vel_profile 加减速曲线
/// @param accum_cycle 累计插补时间（非负数）
/// @return 累计插补距离
double cal_traj_distance(VELPLAN_TYPE velplan_type, VEL_PROFILE* vel_profile, int accum_cycle);

/// @brief 计算单加速/单减速段距离
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param vs 起始速度（非负数）
/// @param ve 终点速度（非负数）
/// @param amax 加速度（非负数）
/// @param jmax 加加速度（非负数）
/// @return 单加速/单减速段距离
double cal_cri_length(VELPLAN_TYPE velplan_type, double vs, double ve, double amax, double jmax);

/// @brief 计算减速段的末速度
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param vs 起始速度（非负数）
/// @param amax 加速度（非负数）
/// @param jmax 加加速度（非负数）
/// @param decline_s 减速距离（非负数）
/// @return 终点速度（非负数）
double cal_end_vel(VELPLAN_TYPE velplan_type, double vs, double amax, double jmax, double decline_s);

/// @brief 计算前瞻速度
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param plan_queue 前瞻队列
/// @return 
void cal_look_ahead(VELPLAN_TYPE velplan_type, PLAN_QUEUE* plan_queue);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_H