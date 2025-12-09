#ifndef __EMCMOT_H__
#define __EMCMOT_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        emcmot.h
 * @brief       NC的运动控制
 * @version     V1.00
 * @date        2025/11/12
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include "./datadef/common.h"
#include "./homing.h"
#include "./mem.h"
#include "./utils/json_write.h"
#include "./datadef/shm.h"

extern PARAMS *params;  // 机床参数
extern HAL_DATA *hal; // 硬件抽象层数据
extern SYS_DATA *sys;

extern HOMING_TP *x_homing_tp;
extern HOMING_TP *y_homing_tp;
extern HOMING_TP *z_homing_tp;

extern MEM_EXEC_STATE mem_exec_state;
extern MEM_TP *pausing_tp;
extern MEM_TP *cycle_tp;

extern char* log_file;

// 路径规划：解释器译码 → 处理成block → 平滑、拟合
void emcmotPathPlan();

// 轨迹规划：速度前瞻 → 速度控制 → 插补 → 补偿 → 更新hal → 反馈status
void emcmotTrajPlan();


#ifdef __cplusplus
}
#endif

#endif