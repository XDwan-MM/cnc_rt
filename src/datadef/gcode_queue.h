/******************************************************************** 
 * @file        gcode_queue.h
 * @brief       译码指令队列
 * @version     V1.00
 * @date        2025/11/19
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __GCODE_QUEUE_H__
#define __GCODE_QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "datatype.h"
#include "motiondata.h"
#include "ncitpdatadef.h"

#include <pthread.h>

#define DCD_BUF_SIZE 200

// 解释器译码结果
typedef struct gcode_block{
    INTERP_TYPE type;   // 插补类型
    pmc_mode mode;      // 模组
    NcItpState state;   // 模态
    int line_number;    // 运行行号
    double feed_vel;    // 进给速度
    POS pos;   
    POS center_pos; 
    PLANE plane; 
}GCODE_BLOCK;

typedef struct
{
    GCODE_BLOCK *block;
    int head;
    int tail;
    int size;
    pthread_mutex_t mutex;
}GCODE_QUEUE;

GNC_RES gcode_queue_init(GCODE_QUEUE* queue);

GNC_RES gcode_queue_reset(GCODE_QUEUE* queue);

GNC_RES gcode_queue_pop(GCODE_QUEUE* queue);

GNC_RES gcode_queue_push(GCODE_QUEUE* queue, const GCODE_BLOCK* pl_block);

GCODE_BLOCK* gcode_queue_peak(const GCODE_QUEUE* queue);

// 以head为起始，向后偏移为index
GCODE_BLOCK* gcode_queue_get_block(const GCODE_QUEUE* queue, int index);

int gcode_queue_get_size(const GCODE_QUEUE* queue);

bool gcode_queue_is_empty(const GCODE_QUEUE* queue);

bool gcode_queue_is_full(const GCODE_QUEUE* queue);

GNC_RES gcode_queue_delete(GCODE_QUEUE* queue);

#ifdef __cplusplus
}
#endif
#endif
