
/********************************************************************
 * @file        auto_mcode.h
 * @brief       自动模式
 * @version     V1.00
 * @date        2026/07/22
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __AUTO_MCODE_H__
#define __AUTO_MCODE_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REG_8BITE 256
#define REG_16BITE 65536
#define REG_24BITE 4294967296

void auto_get_mstb_cmd();
void auto_update_mstb_cmd();
void auto_execute_mstb_cmd();
bool auto_mstb_cmd_is_finished();
void cycle_tp_mstb_cmd_data_reset();

#ifdef __cplusplus
}
#endif

#endif
