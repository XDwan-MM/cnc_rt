#ifndef MPG_PLANNER_H
#define MPG_PLANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "datatype.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * 单轴 MPG 轨迹状态。位置量沿用 wcs_cmd_pos_cur 的坐标单位；速度、加速度
 * 分别沿用 mpg_spd、max_feed_acc 的单位。count 是本周期硬件计数，
 * previous_count 是已消费计数；multiplier 是面板倍率的数值（1/10/100）。
 * target_position 是计数转换后的待达位置，reference_position 是停止距离
 * 限制和异常恢复使用的位置基准；max_velocity/max_acceleration 是本周期限制。
 * current_position/current_velocity 是轨迹积分状态；active 反映当前非零运动，
 * first_cycle 使切换轴或倍率后的首个周期只同步计数、不产生位移。
 * 该类型仅供 MPG 实现与白盒单元测试使用，不属于 mpg.h 公共接口。
 */
typedef struct
{
    uint32_t count;                 /* 最近一次读取的无符号手轮硬件计数 */
    uint32_t previous_count;        /* 上次消费计数；无符号差支持计数器回绕 */
    uint32_t multiplier;            /* X1/X10/X100 对应的计数倍率 */
    AXIS_TYPE axis;                 /* 本规划状态绑定的进给轴 */
    double target_position;         /* 由手轮计数累计得到的目标位置 */
    double reference_position;      /* 停止距离限幅所用的位置参考 */
    double max_velocity;            /* 当前允许的速度上限 */
    double max_acceleration;        /* 当前允许的加速度上限 */
    double current_position;        /* 规划器积分得到的当前位置 */
    double current_velocity;        /* 带加速度约束的当前速度 */
    bool active;                    /* 速度高于静止阈值时为 true */
    bool first_cycle;               /* 首周期只同步计数，不生成位移 */
} mpg_planner_t;

/* 消费本周期计数，按方向限位、倍率及停止距离约束更新目标位置。 */
void mpg_planner_update_target(mpg_planner_t *planner);

/* 推进速度斜坡和位置积分；无效参数使速度归零并丢弃待执行目标。
 * 位置无效时仅从有限的共享指令位置或参考位置恢复，无基准时保留无效状态。 */
void mpg_planner_step(mpg_planner_t *planner, double period);

#ifdef __cplusplus
}
#endif

#endif /* MPG_PLANNER_H */
