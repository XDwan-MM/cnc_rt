#ifndef JSON_WRITE_H
#define JSON_WRITE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        json_write.h
 * @brief       保存为json
 * @version     V1.00
 * @date        2025/10/20
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/


#include "./datadef/common.h"
#include "./datadef/plan_queue.h"
#include "./kinematics/trajectory.h"
#include "./motion/homing.h"
#include "./motion/mem.h"

GNC_RES write_plan_queue_file(const char *filepath, PLAN_QUEUE* pl_queue);

GNC_RES write_vel_profile_file(const char *filepath, VEL_PROFILE* profile);

GNC_RES write_interp_s_file(const char *filepath, double s);

GNC_RES write_all_data_file(const char *filepath);

extern PARAMS *params;  // 机床参数
extern HAL_DATA *hal; // 硬件抽象层数据
extern PLC_REG *plc;
extern SYS_DATA *sys;
extern MEM_EXEC_STATE mem_exec_state;
extern MEM_TP *pausing_tp;
extern MEM_TP *cycle_tp;
extern bool is_paused_in_block;
extern PLAN_QUEUE *plan_queue;
extern VELPLAN_TYPE velplan_type; 
extern HOMING_TP *x_homing_tp;
extern HOMING_TP *y_homing_tp;
extern HOMING_TP *z_homing_tp;

#ifdef __cplusplus
}
#endif

#endif // JSON_WRITE_H