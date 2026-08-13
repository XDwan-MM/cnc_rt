#ifndef EMCMOT_H
#define EMCMOT_H

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

/// @brief 路径规划：
/// parser_run_mode 解释器译码
/// chan_add_block 添加block
/// pretreat_block 预处理
/// unif_block 一致性处理 smooth_block_by_bezier、generate_approx_spline_blk
void emcmotPathPlan();

/// @brief 轨迹规划：
/// chan_do_period 通道周期状态机：复位、报警和通知扫描、手动、回零和程序运行、坐标处理
/// smx_do_period 运动同步控制：主轴同步、刚性攻丝/频率攻丝同步、重叠控制
/// axis_write_out 轴输出与伺服处理：获取通道周期生成的轴增量、检查正负软限位、更新编码器/反馈命令位置、执行补偿
void emcmotTrajPlan();

#ifdef UNIT_TEST
void emcmot_test_rebuild_axis_positions_from_feedback();
#endif

#ifdef __cplusplus
}
#endif

#endif /* EMCMOT_H */
