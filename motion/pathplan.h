#ifndef __PATH_PLAN_H__
#define __PATH_PLAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        pathplan.h
 * @brief       轨迹规划
 * @version     V1.00
 * @date        2025/10/16
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include <stdlib.h>
#include "./datadef/common.h"
#include "./datadef/plan_queue.h"
#include "./math/cnc_math.h"
#include "./kinematics/trajectory.h"

#define PRE_TREAT_SIZE 200   // 预处理的最大点数
#define LOOK_AHEAD_SIZE 200  // 速度前瞻的最大段数

extern PARAMS *params; 
extern GCODE_QUEUE *gcode_queue;    // 译码队列
extern PLAN_QUEUE *pretreat_queue;  // 预处理队列
extern PLAN_QUEUE *plan_queue;      // 插补队列

GNC_RES dcd_to_block(GCODE_QUEUE *gcode_queue, int size, PLAN_QUEUE* pl_queue, PARAMS* param);
GNC_RES dcd_look_ahead(PLAN_QUEUE* pl_queue, PARAMS* param, VELPLAN_TYPE type);

void block_cal_pos(INTERP_TYPE type, PLAN_BLOCK* current_block, POS* start_pos, POS* end_pos);
void block_cal_vel(PLAN_BLOCK* current_block, INTERP_TYPE next_type, double next_vel);
void block_cal_length(PLAN_BLOCK* current_block);
void block_cal_vector(PLAN_BLOCK* current_block);
void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector);

//处理译码结果为block
GNC_RES chan_add_block();

/// @brief 速度前瞻
/// @param vs 起始速度
/// @return GNC_DATA_EMPTY 插补数据为空
GNC_RES chan_look_ahead(double vs);


#ifdef __cplusplus
}
#endif

#endif