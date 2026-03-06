/******************************************************************** 
 * @file        plan_queue.h
 * @brief       插补指令队列
 * @version     V1.00
 * @date        2025/08/19
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __queue_H__
#define __queue_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include "datatype.h"
#include "ncitpdatadef.h"

// 循环队列长度
#define PLAN_QUEUE_SIZE 200

typedef enum{
    CLOCK = 0,    // 顺时针
    COUNTERCLOCK, // 逆时针
}CIRCLE_DIR;

typedef struct{
    POS center_pos;     // 圆心位置
    double radius;      // 半径
    PLANE plane;        // 选择的平面
    CIRCLE_DIR dir;     // 方向（顺时针，逆时针）
    double rad;         // 弧度
    double statr_rad;   // 起始弧度
    double end_rad;     // 终点弧度
}CIRCULAR_BLOCK;

typedef struct
{
    INTERP_TYPE type;   // 插补类型
    pmc_mode mode;      // 模组
    NcItpState state;   // 模态
    int line_number;    // 运行行号
    union{              // 块信息
        CIRCULAR_BLOCK circular_block;
    };
    double length;      // 长度
    double angle_rad;   // 夹角
    double start_vel;   // 起始速度
    double end_vel;     // 终点速度
    double feed_vel;    // 设置的进给速度F，读取NC文件参数，不变
    double max_vel;     // 最大速度，随着进给倍率改变而改变
    double max_acc;
    double max_jerk;
    POS start_pos;      // 起始位置
    POS end_pos;        // 终点位置
    VECTOR start_vec;   // 起始矢量
    VECTOR end_vec;     // 终点矢量
}PLAN_BLOCK;

// 循环队列
typedef struct
{
    PLAN_BLOCK* pl_block;
    int head;
    int tail;
    int size;
    // 前一段信息
    POS pre_pos;                 // 前一段终点位置
    VECTOR pre_vec;              // 前一段终点切失
    double next_vel;             // 后一段初速度
    bool isFirstRUN;             // 是否为程序起点
    // 前首段信息
    INTERP_TYPE pre_head_type;   // 前首段插补类型
    double pre_head_vel;         // 前首段末速度
    pthread_mutex_t mutex;
}PLAN_QUEUE;

GNC_RES queue_init(PLAN_QUEUE* queue);

GNC_RES queue_reset(PLAN_QUEUE* queue);

GNC_RES queue_pop(PLAN_QUEUE* queue);

GNC_RES queue_soft_push(PLAN_QUEUE* queue);

GNC_RES queue_push(PLAN_QUEUE* queue, const PLAN_BLOCK* pl_block);

PLAN_BLOCK* queue_peak(const PLAN_QUEUE* queue);

PLAN_BLOCK* queue_peak_tail(const PLAN_QUEUE* queue);

// 以head为起始，向后偏移为index
PLAN_BLOCK* queue_get_block(const PLAN_QUEUE* queue, int index);

void queue_copy_block(const PLAN_QUEUE* queue, int index, PLAN_BLOCK* copy_block);

int queue_get_size(const PLAN_QUEUE* queue);

bool queue_is_empty(const PLAN_QUEUE* queue);

bool queue_is_full(const PLAN_QUEUE* queue);

GNC_RES queue_delete(PLAN_QUEUE* queue);

#ifdef __cplusplus
}
#endif
#endif
