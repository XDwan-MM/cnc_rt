#include <stdlib.h>
#include <stdbool.h>
#include "jog_control.h"

void start_jog_cont(const PAR_AXIS* axis_para, simple_tp_t *jog_tp, int direction, const double feed_override, const double run_period)
{
    /* 输入参数检查 */
    if ((axis_para == NULL)
        || (jog_tp == NULL)
        || (feed_override < 0.0)
        || (feed_override > 3.0))
    {
        return ;
    }
    /* 检查运行方向 */
    if (direction == 0) {
        return;
    }
    /* 根据方向设置JOG目标位置（增量至极限(到限位)） */
    if (direction > 0) {
        /* 正向JOG：目标位置设为最大限位 */
        jog_tp->pos_cmd = axis_para->plmt;
    } else {
        /* 负向JOG：目标位置设为最小限位 */
        jog_tp->pos_cmd = axis_para->nlmt;
    }

    /* 设置速度限制*/
    // jog_lowspd实时这边的单位是：mm/s
    // feed_override实时这边统一范围为（0.0～1.5）
    jog_tp->max_vel = axis_para->jog_lowspd * feed_override;

    /* 设置加速度限制 */
    // feed_acc实时这边统一的单位是mm/s^2
    jog_tp->max_acc = axis_para->feed_acc;

    /* 使能轨迹规划器 */ // 让JOG动起来
    jog_tp->enable = 1;

    /* 更新轨迹规划器 */
    simple_tp_update(jog_tp, run_period);
}

void start_jog_rapid_cont(const PAR_AXIS* axis_para, simple_tp_t *rapid_jog_tp, int direction, const double feed_override, const double run_period)
{
    /* 输入参数检查 */
    if ((axis_para == NULL)
        || (rapid_jog_tp == NULL)
        || (feed_override < 0.0)
        || (feed_override > 3.0))
    {
        return ;
    }
    /* 检查运行方向 */
    if (direction == 0) {
        return;
    }
    /* 根据方向设置JOG目标位置（增量至极限(到限位)） */
    if (direction > 0) {
        /* 正向JOG：目标位置设为最大限位 */
        rapid_jog_tp->pos_cmd = axis_para->plmt;
    } else {
        /* 负向JOG：目标位置设为最小限位 */
        rapid_jog_tp->pos_cmd = axis_para->nlmt;
    }

    /* 设置速度限制*/
    // jog_fastspd实时这边的单位是：mm/s
    // feed_override实时这边统一范围为（0.0～1.5）
    rapid_jog_tp->max_vel = axis_para->jog_fastspd * feed_override;

    /* 设置加速度限制 */
    // rapid_acc实时这边统一的单位是mm/s^2
    rapid_jog_tp->max_acc = axis_para->rapid_acc;

    /* 使能轨迹规划器 */ // 让JOG动起来
    rapid_jog_tp->enable = 1;

    /* 更新轨迹规划器 */
    simple_tp_update(rapid_jog_tp, run_period);
}

void stop_jog_cont(simple_tp_t *jog_tp, const double run_period, bool immediate)
{
    if (jog_tp == NULL) return;

    //  bool aborted = 0;
    //  if (runing_axis->jog_tp.enable) {
    //    aborted = 1;
    //  }
    /* jog轨迹规划器断使能 */
    jog_tp->enable = 0;
    if(immediate)
    {
        jog_tp->curr_vel = 0.0;
    }
    simple_tp_update(jog_tp, run_period);
}
