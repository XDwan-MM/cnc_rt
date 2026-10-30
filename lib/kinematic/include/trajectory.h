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

#include "cnc_math.h"
#include "plan_queue.h"
#include "gncdatadef.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOL 1E-6;
#define MAX_WINDOW_SIZE 50   // 最大支持指令平滑周期

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

// 插补数据
typedef struct{
    double s;
    double v;
    double a;
    double j;
}TRAJ_PARAMS;

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

typedef struct {
    double buffer[MAX_WINDOW_SIZE]; // 环形历史数据缓冲区
    int capacity;                   // 设定的滤波窗口长度 N
    int head;                       // 当前写入位置指针
    double sum;                     // 当前窗口内数据的累加和
    int count;                      // 系统启动阶段已存入的有效点数
} FIR_MA;

int fir_ma_init(FIR_MA** fir_ma, int window_size);
void fir_ma_delete(FIR_MA** fir_ma);
void fir_ma_reset(FIR_MA *filter, int window_size);
double fir_ma_update(FIR_MA *filter, double input);

/// @brief 计算加减速曲线
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param plan_block 插补块
/// @return 加减速曲线
int vp_cal_vel_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time);

/// @brief 计算插补距离
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param vel_profile 加减速曲线
/// @param interp_cycle 累计插补时间（非负数）
/// @return 累计插补距离
void vp_cal_traj_data(VELPLAN_TYPE velplan_type, VEL_PROFILE* vel_profile, int interp_cycle, TRAJ_PARAMS* traj_data, double cycle_time);

/// @brief 计算单加速/单减速段距离
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param plan_block 加减速段信息，vs ve acc jerk
/// @return 单加速/单减速段距离
double vp_cal_decel_dist(VELPLAN_TYPE velplan_type, double vs, double ve, double amax, double jmax);

/// @brief 计算减速段的末速度
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param vs 起始速度（非负数）
/// @param amax 加速度（非负数）
/// @param jmax 加加速度（非负数）
/// @param decline_s 减速距离（非负数）
/// @return 终点速度（非负数）
double vp_cal_decel_vel(VELPLAN_TYPE velplan_type, double vs, double amax, double jmax, double decline_s);

/// @brief 计算前瞻速度
/// @param velplan_type 加减速类型：T_SHAPE, S_SHAPE, SIN_SHAPE
/// @param plan_queue 前瞻队列
/// @param cycle_time 
/// @param radius_error 基于圆弧插补的容许误差
/// @param circular_min_vel 基于圆弧插补的下限速度
/// @return 
void vp_cal_bid_lookahead(VELPLAN_TYPE velplan_type, PLAN_QUEUE* plan_queue, double cycle_time, double ecen_acc, double corner_dec_factor, double x_corner_vel_thr, double y_corner_vel_thr, double z_corner_vel_thr);

/// @brief 圆弧曲线的加速度进行的速度控制
/// @param cur_block 
/// @param chord_error 圆弧弓高误差
/// @param cycle_time 
/// @return double radius_error, double circular_min_vel,
double vp_cal_circular_max_vel(PLAN_BLOCK* cur_block, double chord_error, double max_fv_arc_radius, double radius_r_max_fv, double circular_min_vel, double cycle_time);

/// @brief nurbs曲线的加速度进行的速度控制
/// @param cur_block 
/// @param chord_error 轮廓误差
/// @param cycle_time 
/// @return 
double vp_cal_nurbs_max_vel(PLAN_BLOCK* cur_block, double chord_error, double cycle_time);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_H