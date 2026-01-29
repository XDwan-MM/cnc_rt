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
#include "plan_queue.h"

#define PRE_TREAT_SIZE 200   // 预处理的最大点数
#define LOOK_AHEAD_SIZE 200  // 速度前瞻的最大段数

/// @brief 将译码结果转换成插补段
/// @return 
GNC_RES chan_add_block();

/// @brief 速度前瞻
/// @param vs 起始速度
/// @return GNC_DATA_EMPTY 插补数据为空
GNC_RES chan_look_ahead(double vs);

void block_cal_length(PLAN_BLOCK* current_block);
void block_cal_vector(PLAN_BLOCK* current_block);

#ifdef __cplusplus
}
#endif

#endif
