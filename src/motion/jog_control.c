/********************************************************************
 * @file        jog_control.c
 * @brief       JOG控制模块
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include "jog_control.h"

void start_jog_cont(VAR_AXIS *runing_axis, const PAR_AXIS *axis_para, int direction, double run_period, const double feed_override)
{
    if ((runing_axis == NULL) || (axis_para == NULL) || (feed_override < 0.0) || (feed_override > 3.0)) return ;
  /* 检查运行方向 */
  if (direction == 0) {
    return;
  }
  /* 根据方向设置JOG目标位置（增量至极限(到限位)） */
  if (direction > 0) {
    /* 正向JOG：目标位置设为最大限位 */
    runing_axis->jog_tp.pos_cmd = axis_para->plmt;
  } else {
    /* 负向JOG：目标位置设为最小限位 */
    runing_axis->jog_tp.pos_cmd = axis_para->nlmt;
  }

  /* 设置速度限制*/
  runing_axis->jog_tp.max_vel = (axis_para->jog_lowspd * feed_override) / 60.0;

  /* 设置加速度限制 */
  runing_axis->jog_tp.max_acc = axis_para->feed_acc;

  /* 使能轨迹规划器 */ // 让JOG动起来
  runing_axis->jog_tp.enable = 1;

  /* 标记JOG激活 */
  //    runing_axis->jog_active = 1;
  /* 更新轨迹规划器 */
  simple_tp_update(&runing_axis->jog_tp, run_period);
}

void start_jog_rapid_cont(VAR_AXIS *runing_axis, const PAR_AXIS *axis_para, int direction, double run_period, const double rapid_override)
{
  if ((runing_axis == NULL) || (axis_para == NULL) || (rapid_override < 0.0) || (rapid_override > 2.0))return;
  /* 检查运行方向 */
  if (direction == 0) {
    return;
  }

  /* 根据方向设置JOG目标位置（增量至极限(到限位)） */
  if (direction > 0) {
    /* 正向JOG：目标位置设为最大限位 */
    runing_axis->jog_tp.pos_cmd = axis_para->plmt;
  } else {
    /* 负向JOG：目标位置设为最小限位 */
    runing_axis->jog_tp.pos_cmd = axis_para->nlmt;
  }

  /* 设置速度限制*/
  runing_axis->jog_tp.max_vel = (axis_para->jog_fastspd * rapid_override) / 60.0;

  /* 设置加速度限制 */
  runing_axis->jog_tp.max_acc = axis_para->rapid_acc;

  /* 使能jog轨迹规划器 */ // 让JOG动起来
  runing_axis->jog_tp.enable = 1;

  /* 标记JOG激活 */
  //    axis->jog_active = 1;

  /* 更新轨迹规划器 */
  simple_tp_update(&runing_axis->jog_tp, run_period);
}

void stop_jog_cont(VAR_AXIS *runing_axis, double run_period, bool immediate)
{
  if (runing_axis == NULL) return;

//  bool aborted = 0;
//  if (runing_axis->jog_tp.enable) {
//    aborted = 1;
//  }
/* jog轨迹规划器断使能 */
  runing_axis->jog_tp.enable = 0;
  if(immediate)
  {
    runing_axis->jog_tp.curr_vel = 0.0;
  }
  simple_tp_update(&runing_axis->jog_tp, run_period);
}
