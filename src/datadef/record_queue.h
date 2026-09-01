/******************************************************************** 
 * @file        record_queue.h
 * @brief       断点信息队列，先进先出，只保留N个最新数据
 * @version     V1.00
 * @date        2026/08/28
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __record_queue_H__
#define __record_queue_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "plan_queue.h"
#include "datatype.h"

#define RECORD_NUM 35

// 循环队列
typedef struct
{
    PLAN_BLOCK block[RECORD_NUM];
    int length;
    int head;
    int tail;
    int size;
}RECORD_QUEUE;

GNC_RES record_queue_init(RECORD_QUEUE* queue, int length);

GNC_RES record_queue_reset(RECORD_QUEUE* queue);

GNC_RES record_queue_pop(RECORD_QUEUE* queue);

GNC_RES record_queue_soft_push(RECORD_QUEUE* queue);

GNC_RES record_queue_push(RECORD_QUEUE* queue, const PLAN_BLOCK* block);

PLAN_BLOCK* record_queue_peak(const RECORD_QUEUE* queue);

PLAN_BLOCK* record_queue_peak_tail(const RECORD_QUEUE* queue);

// 以head为起始，向后偏移为index
PLAN_BLOCK* record_queue_get_block(const RECORD_QUEUE* queue, int index);

void record_queue_copy_block(const RECORD_QUEUE* queue, int index, PLAN_BLOCK* copy_block);

int record_queue_get_size(const RECORD_QUEUE* queue);

bool record_queue_is_empty(const RECORD_QUEUE* queue);

bool record_queue_is_full(const RECORD_QUEUE* queue);

GNC_RES record_queue_delete(RECORD_QUEUE* queue);

#ifdef __cplusplus
}
#endif
#endif
