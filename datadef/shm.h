/******************************************************************** 
 * @file        shm.h
 * @brief       共享内存全局数据
 * @version     V1.00
 * @date        2025/11/06
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __SHM_H__
#define __SHM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "./motion/homing.h"
#include "./motion/mem.h"

GNC_RES plc_init();
GNC_RES param_init();
GNC_RES hal_init();
GNC_RES sys_init();
GNC_RES sys_reset();
GNC_RES tp_init();
GNC_RES tp_reset();

#ifdef __cplusplus
}
#endif

#endif