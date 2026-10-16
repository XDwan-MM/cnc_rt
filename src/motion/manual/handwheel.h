#ifndef HANDWHEEL_H
#define HANDWHEEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <GreeMasterAPI.h>
#include "global.h"

typedef enum {
    HANDWHEEL_AXIS_OFF = 0,
    HANDWHEEL_AXIS_X   = 1,
    HANDWHEEL_AXIS_Y   = 2,
    HANDWHEEL_AXIS_Z   = 4,
    HANDWHEEL_AXIS_4   = 8,
    HANDWHEEL_AXIS_5   = 16
} handwheel_axis_t;

typedef enum {
    HANDWHEEL_RATE_OFF   = 0,
    HANDWHEEL_RATE_X1    = 1,
    HANDWHEEL_RATE_X10   = 2,
    HANDWHEEL_RATE_X100  = 4
} handwheel_rate_t;

typedef struct{
    //假定
    uint32_t count;
    uint32_t rate;
    uint32_t old_counts;
    int32_t drivers_data;
    double vel_limit;
    double jaccel_limit;
    double pos_cmd;
} motion_joint_t;

typedef struct{
    motion_joint_t joint;
    double pos_cmd;     /* position command - 目标位置命令 */
    double max_vel;     /* velocity limit - 速度限制 */
    double max_acc;     /* acceleration limit - 加速度限制 */
    int enable;         /* if zero, motion stops ASAP */
    double curr_pos;    /* current position - 当前计算位置 */
    double curr_vel;    /* current velocity - 当前速度 */
    int active;         /* non-zero if motion in progress */
    int first_pass;
    AXIS_TYPE axis;
} simple_tp_t_handwheel;

extern int32_t ratio;
extern simple_tp_t_handwheel simple_tp[5];
void Handlewheels_Motion(simple_tp_t_handwheel *tp);
void Simple_Tp_Update(simple_tp_t_handwheel *tp);
int Get_Multiplier_Switch(int output);
int Get_Axis_Switch(int output);
void handle_mpg_cmd();
void handwheel_handle_estop();

/*
 * 重建手轮位置基准：丢弃规划器里尚未输出的位移，下一周期按实时数据中的
 * wcs_cmd_pos_cur 重新对齐并从当前硬件计数重新开始（first_pass）。
 * 由 emcmot 在重新进入手轮模式时调用（离开手轮后其他模式可能已移动过机床）。
 */
void handwheel_resync_position(void);

#ifdef __cplusplus
}
#endif

#endif
