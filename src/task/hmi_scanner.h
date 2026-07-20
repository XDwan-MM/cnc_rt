/******************************************************************** 
 * @file        hmi_scanner.h
 * @brief       HMI事件处理
 * @version     V1.00
 * @date        2026/07/17
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __HMI_SCANNER_H__
#define __HMI_SCANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "operatesignalapi.h"
#include "operatesignaldata.h"

// 每次轮询最多拉取的事件数
#define DEMO_MAX_EVENTS_PER_POLL 16

// 事件来源名称
static const char* sourceName(uint8_t source)
{
    switch (source) {
        case KEY_SOURCE_KEYBOARD: return "Keyboard";
        case KEY_SOURCE_SOFTKEY:  return "SoftKey";
        default:                  return "Unknown";
    }
}

// 事件状态名称
static const char* stateName(uint8_t state)
{
    switch (state) {
        case KEY_STATE_PRESSED:  return "Pressed ";
        case KEY_STATE_RELEASED: return "Released";
        default:                 return "Unknown ";
    }
}

void hmi_events_scan(void);

#ifdef __cplusplus
}
#endif
#endif

