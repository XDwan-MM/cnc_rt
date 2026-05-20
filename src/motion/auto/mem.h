
/********************************************************************
 * @file        mem.h
 * @brief       记忆模式
 * @version     V1.00
 * @date        2026/03/30
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __MEM_H__
#define __MEM_H__

#include "mem_gcode.h"
#include "mem_mst.h"

#ifdef __cplusplus
extern "C" {
#endif

void handle_run_cmd();
void auto_handle_estop();
void handle_edit_cmd();

#ifdef __cplusplus
}
#endif

#endif