/******************************************************************** 
 * @file        event_queue.h
 * @brief       事件队列
 * @version     V1.00
 * @date        2026/01/28
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __EVENT_QUEUE_H__
#define __EVENT_QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <stdbool.h>
#include "pmc_io_map.h"
#include "datatype.h"

// 循环队列长度
#define EVENT_QUEUE_SIZE 1000
#define EVENT_DATA_MAX_LEN 256

typedef enum{
    EV_SRC_SYS = 0,      // 系统事件
    EV_SRC_CHAN,         // 通道事件
    EV_SRC_MDI,          // MDI事件
    EV_SRC_KBD,          // 键盘事件
    EV_SRC_AXIS,         // 轴事件
    EC_SRC_NET,          // 网络事件
}EVENT_SRC;

// 事件队列
typedef struct
{
    pthread_mutex_t mutex;
    CNC_EVENT* event_data;
    int head;
    int tail;
    int size;
}EVENT_QUEUE;

GNC_RES event_queue_init(EVENT_QUEUE* queue);

GNC_RES event_queue_reset(EVENT_QUEUE* queue);

GNC_RES event_queue_push(EVENT_QUEUE* queue, CNC_EVENT* event_data);

GNC_RES event_queue_pop(EVENT_QUEUE* queue, CNC_EVENT* event_data);

int event_queue_get_size(EVENT_QUEUE* queue);

bool event_queue_is_empty(EVENT_QUEUE* queue);    

bool event_queue_is_full(EVENT_QUEUE* queue);

GNC_RES event_queue_delete(EVENT_QUEUE* queue);

// 使用实例
// 1. 初始化队列
// EVENT_QUEUE* event_queue = (EVENT_QUEUE*)calloc(sizeof(EVENT_QUEUE), 1);
// event_queue_init(event_queue);
// 2. 写入事件
// EVENT_DATA event_data = {};
// event_data.src = EV_SRC_SYS;
// event_data.code = EV_CODE_RESET;
// event_queue_push(event_queue, &event_data);
// 3. 读取事件
// EVENT_DATA event_data_out = {};
// event_queue_pop(event_queue, &event_data_out);

#ifdef __cplusplus
}
#endif
#endif
