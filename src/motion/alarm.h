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

void handle_estop_press();
void handle_estop_cmd();
void handle_alarm_cmd();

#ifdef __cplusplus
}
#endif

#endif

