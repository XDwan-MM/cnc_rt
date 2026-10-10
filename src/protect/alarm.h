#ifndef __ALARM_H__
#define __ALARM_H__

/******************************************************************** 
 * @file        alarm.h
 * @brief       报警模式
 * @version     V1.00
 * @date        2025/12/18
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include "global.h"

#ifdef __cplusplus
extern "C" {
#endif

void handle_estop_cmd();
void handle_alarm_cmd();

/* 每拍检查 HAL 侧状态：周期循环是否挂了、故障复位有没有收口。
 * 由运动线程调用——它每拍都跑，且只读适配层快照、不碰 HAL context。 */
void alarm_poll_hal();

/* 每拍把轴使能意图同步给 HAL（基线里由 master.c 的 Master_Run() 隐含完成）。
 * 未调用它则所有轴永远停在 SwitchOnDisabled，不会使能。 */
void hal_axis_intent_poll();

/* 把一段 HAL 错误码（正数）报成 RT 报警。
 * 由 **PDO 线程**在周期循环退出后调用：循环一停，运动线程就卡在信号量上、
 * alarm_poll_hal 再也不跑，只有发现错误的人自己能把它报出去。 */
void alarm_report_hal_error(int32_t rc);

#ifdef __cplusplus
}
#endif

#endif

