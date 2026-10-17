#ifndef __AUTO_GCODE_H__
#define __AUTO_GCODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************
 * @file        auto_gcode.h
 * @brief       自动模式
 * @version     V1.00
 * @date        2026/07/22
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "plan_queue.h"
#include "trajectory.h"

#define VEL_EPSILON 1E-4     // 速度误差

void auto_update_pos_cmd();
void auto_get_pos_cmd();
void auto_execute_pos_cmd();
void auto_update_dwell_cmd();
void auto_get_dwell_cmd();
void auto_execute_dwell_cmd();
void auto_get_null_cmd();
void auto_update_null_cmd();
void auto_execute_null_cmd();
bool auto_pos_cmd_is_finished();
bool auto_dwell_cmd_is_finished();
void cycle_tp_reset_pos_cmd_data();
void cycle_tp_reset_dwell_cmd_data();

#ifdef __cplusplus
}
#endif

#endif
