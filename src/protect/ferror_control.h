/**
 * @file        ferror_control.h
 * @brief       跟随误差检测模块
 * @date        2026/09/01
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef FERROR_CONTROL_H
#define FERROR_CONTROL_H

#include "gncdatadef.h"
#include "pardata.h"
#include <stdbool.h>

#define FERROR_SPEED_THRESHOLD (1.0e-5) /* 速度判断阈值 */

#ifdef __cplusplus
extern "C"
{
#endif

/* 跟随误差控制器结构体 */
typedef struct
{
    double ferror;       /* 实际跟随误差 (mm)，供 HMI 显示 */
    double ferror_limit; /* 当前生效的跟随误差限值 (mm)，供 HMI 显示 */
    bool error_flag;     /* 跟随误差超限标志，置位后由 reset_ferror_control() 清除 */
    bool is_moving;      /* 轴是否在运动中（区分报警类型：移动时/停止时）*/
} ferror_control_t;

// 外部声明,跟随误差控制器
extern ferror_control_t ferror_control[SYS_AXES_NUM];

/* 全轴跟随误差故障检查（在轨迹规划周期中调用，重复报警只记录一次日志） */
extern void check_ferror_fault(void);

/* 清除跟随误差超限标志及轴状态中的报警标志 */
extern void reset_ferror_control(void);

#ifdef __cplusplus
}
#endif

#endif /* FERROR_CONTROL_H */
