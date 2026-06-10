/********************************************************************
 * @file        ferror_control.h
 * @brief       1、跟随误差控制功能（FANUC PRM 1828/1829 双阈值模型）
 * @version     V2.00
 * @date        2026/06/10
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef FERROR_CONTROL_H
#define FERROR_CONTROL_H

#include <stdbool.h>
#include "pardata.h"
#include "gncdatadef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 跟随误差控制器结构体 */
typedef struct {
  double ferror;          /* 实际跟随误差 (mm)，供 HMI 显示 */
  double ferror_limit;    /* 当前生效的跟随误差限值 (mm)，供 HMI 显示 */
  bool error_flag;        /* 跟随误差超限标志 */
  bool is_moving;         /* 轴是否在运动中（区分报警类型：移动时/停止时）*/
} ferror_control_t;

// 外部声明,跟随误差控制器
extern ferror_control_t ferror_control[SYS_AXES_NUM];

/* 全轴跟随误差故障检查（在轨迹规划周期中调用）*/
extern void check_ferror_fault(void);

/* 复位清除跟随误差标志 */
extern void reset_ferror_control(void);

#ifdef __cplusplus
}
#endif

#endif /* FERROR_CONTROL_H */
