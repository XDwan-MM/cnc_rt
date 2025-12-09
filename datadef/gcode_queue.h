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

#include "common.h"

#define DCD_BUF_SIZE 200

typedef enum
{
    RAPID,       // 快速进给 G00
    LINE,        // 直线 G01
    CIRCLE_CCW,  // 逆时针圆弧 G02
    CIRCLE_CW,   // 顺时针圆弧 G03
    HELIX,       // 螺纹 G04
    SPLINE,      // 样条
    RIGID_TAP,   // 刚性攻丝
    PROBE,       // 探针
    UNKNOWN
}INTERP_TYPE;

// 解释器译码结果
typedef struct gcode_block{
    INTERP_TYPE type;   // 插补类型
    double feed_vel;    // 进给速度
    POS pos;    
}GCODE_BLOCK;

typedef struct 
{
    GCODE_BLOCK *block;
    int head;
    int tail;
    int size;
}GCODE_QUEUE;

GNC_RES gcode_queue_init(GCODE_QUEUE* queue);

GNC_RES gcode_queue_reset(GCODE_QUEUE* queue);

GCODE_BLOCK* gcode_queue_pop(GCODE_QUEUE* queue);

GNC_RES gcode_queue_push(GCODE_QUEUE* queue, GCODE_BLOCK* pl_block);

GCODE_BLOCK* gcode_queue_peak(GCODE_QUEUE* queue);

// 以head为起始，向后偏移为index
GCODE_BLOCK* gcode_queue_get_block(GCODE_QUEUE* queue, int index);

int gcode_queue_get_size(GCODE_QUEUE* queue);

bool gcode_queue_is_empty(GCODE_QUEUE* queue);

bool gcode_queue_is_full(GCODE_QUEUE* queue);

GNC_RES gcode_queue_delete(GCODE_QUEUE* queue);



#ifdef __cplusplus
}
#endif
#endif