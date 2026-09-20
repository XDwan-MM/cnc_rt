/**
 * @file spl_vel_planner.h
 * @brief 主轴速度规划器。
 */

#ifndef SPL_VEL_PLANNER_H
#define SPL_VEL_PLANNER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    double target_vel;  /**< 目标速度，单位：转/分钟，带方向。 */
    double current_vel; /**< 当前规划速度，单位：转/分钟。 */
    double max_acc;     /**< 最大加速度，单位：转/分钟/秒。 */
    bool enable;        /**< true 跟踪目标速度，false 减速到零。 */
    bool active;        /**< true 表示仍在加速、减速或旋转。 */
} spl_vel_planner_t;

/**
 * @brief 更新一个控制周期的主轴速度。
 * @param planner 速度规划器状态。
 * @param period 控制周期，单位：秒，必须大于零。
 * @return true 表示本周期后已到达目标速度，否则返回 false。
 */
bool spl_vel_planner_update(spl_vel_planner_t *planner, double period);

#ifdef __cplusplus
}
#endif

#endif /* SPL_VEL_PLANNER_H */
