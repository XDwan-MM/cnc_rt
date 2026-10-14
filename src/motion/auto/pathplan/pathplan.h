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

#define PRE_TREAT_SIZE 1000     // 预处理的最大点数
#define segmentnum 10	        //最小分段长度
#define maxsegmentnum 40        //最大分段长度

/// @brief 将译码结果转换成插补段(parser_queue → pretreat_queue)
GNC_RES chan_add_block();
/// @brief 预处理：剔除异常点、滤波等(pretreat_queue → treat_queue)
GNC_RES pretreat_block();
/// @brief 一致性处理：nubrs、贝塞尔、hermit、圆弧过渡等平滑处理(treat_queue → unif_queue:nurbs_queue……)
GNC_RES unif_block();

void block_cal_length(PLAN_BLOCK* current_block);
void block_cal_vector(PLAN_BLOCK* current_block);

#ifdef __cplusplus
}
#endif

#endif
