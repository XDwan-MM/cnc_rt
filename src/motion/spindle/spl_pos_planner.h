/**
 * @file spl_pos_planner.h
 * @brief 单轴周期同步位置规划器。
 */
#ifndef SPL_POS_PLANNER_H
#define SPL_POS_PLANNER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    double current_pos; /**< 当前规划位置，驱动器位置单位。 */
    double current_vel; /**< 当前规划速度，驱动器位置单位/秒。 */
    double target_pos;  /**< 目标位置，驱动器位置单位。 */
    double max_vel;     /**< 最大规划速度，驱动器位置单位/秒。 */
    double max_acc;     /**< 最大规划加速度，驱动器位置单位/秒²。 */
    bool active;        /**< true 表示尚未到达目标位置。 */
} spl_pos_planner_t;

/**
 * 按最大速度和最大加速度更新一个周期的目标位置。
 * 规划器使用剩余距离计算减速速度，保证目标位置附近平滑停车。
 */
bool spl_pos_planner_update(spl_pos_planner_t *planner, double period);

#ifdef __cplusplus
}
#endif

#endif /* SPL_POS_PLANNER_H */
