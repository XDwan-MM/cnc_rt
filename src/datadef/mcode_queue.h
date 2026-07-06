/********************************************************************
 * @file        mcode_queue.h
 * @brief       译码指令队列
 * @version     V1.00
 * @date        2025/11/19
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __MCODE_QUEUE_H__
#define __MCODE_QUEUE_H__

#include "datatype.h"
#include "ncitpdatadef.h"
#include <pthread.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M_BUF_SIZE 5        // 单个程序段多个M指令（5个）

// 解释器译码结果
typedef struct mcode_block{
    pmc_mode mode;
    NcItpState state;       // 模态
    int m_code;             // 代码信号 M00~M31
    int line_number;        // 运行行号
    pmc_spindle spindle;    // 主轴
    int tool_number;        // 刀具号
}MCODE_BLOCK;

typedef struct
{
    MCODE_BLOCK *block;
    int head;
    int tail;
    int size;
    pthread_mutex_t mutex;
}MCODE_QUEUE;

GNC_RES mcode_queue_init(MCODE_QUEUE* queue);

GNC_RES mcode_queue_reset(MCODE_QUEUE* queue);

GNC_RES mcode_queue_pop(MCODE_QUEUE* queue);

GNC_RES mcode_queue_push(MCODE_QUEUE* queue, MCODE_BLOCK* pl_block);

MCODE_BLOCK* mcode_queue_peak(const MCODE_QUEUE* queue);

// 以head为起始，向后偏移为index
MCODE_BLOCK* mcode_queue_get_block(const MCODE_QUEUE* queue, int index);

void mcode_queue_copy_block(const MCODE_QUEUE* queue, int index, MCODE_BLOCK* copy_block);

int mcode_queue_get_size(const MCODE_QUEUE* queue);

bool mcode_queue_is_empty(const MCODE_QUEUE* queue);

bool mcode_queue_is_full(const MCODE_QUEUE* queue);

GNC_RES mcode_queue_delete(MCODE_QUEUE* queue);

#ifdef __cplusplus
}
#endif
#endif
