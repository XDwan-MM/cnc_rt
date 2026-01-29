#ifndef __EMCMOT_H__
#define __EMCMOT_H__

/******************************************************************** 
 * @file        emcmot.h
 * @brief       NC的运动控制
 * @version     V1.00
 * @date        2025/11/12
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include "global.h"

#ifdef __cplusplus
extern "C" {
#endif

// 路径规划：解释器译码 → 处理成block → 平滑、拟合
void emcmotPathPlan();

// 轨迹规划：速度前瞻 → 速度控制 → 插补 → 补偿 → 更新hal → 反馈status
void emcmotTrajPlan();


#ifdef __cplusplus
}
#endif

#endif
