#ifndef HANDWHEEL_H
#define HANDWHEEL_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"


#define period 0.001
#define TINY_DP(max_acc,period) (max_acc*period*period*0.001)

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
} simple_tp_t;

extern int32_t ratio;
extern simple_tp_t simple_tp[5];
void Handlewheels_Motion(simple_tp_t *tp);
void Simple_Tp_Update(simple_tp_t *tp);
int Get_Multiplier_Switch(int output);
int Get_Axis_Switch(int output);

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif

#endif
