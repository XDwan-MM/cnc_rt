/********************************************************************
 * @file        ferror_control.h
 * @brief       1、跟随误差控制功能
 * @version     V1.00
 * @date        2026/01/13
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef FERROR_CONTROL_H
#define FERROR_CONTROL_H

#include <stdbool.h>
#include "pardata.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 跟随误差控制器结构体 */
typedef struct {
  double pos_cmd;         /* 命令位置 */
  double pos_fb;          /* 反馈位置 */
  double ferror;          /* 实际跟随误差 = fabs(pos_cmd - pos_fb) */
  double vel_cmd;         /* 当前指令速度 */
  double ferror_limit;    /* 当前指令速度下的跟随误差限制值 */
  bool error_flag;        /* 误差标志（1=超出限制） */
} ferror_control_t;

// 外部声明,跟随误差控制器
extern ferror_control_t ferror_control[3];

/* 更新轴的跟随误差 */
/*
* axis_para:传入的误差值单位mm
*/
extern void updata_axis_ferror(ferror_control_t *axis_ferrorCtrl,
                        const ParamUnit* axis_para,
                        double pos_cmd,
                        double pos_fb,
                        double vel_cmd);

#ifdef __cplusplus
}
#endif

#endif /* FOLLOWING_ERROR_H */

