/******************************************************************** 
 * @file        task.h
 * @brief       任务控制器/事件处理
 * @version     V1.00
 * @date        2026/01/28
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __TASK_H__
#define __TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "global.h"

// 任务消息队列初始化
GNC_RES task_mq_init();

// 任务消息队列处理
GNC_RES task_mq_proc();

#ifdef __cplusplus
}
#endif
#endif

