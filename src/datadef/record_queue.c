#include "record_queue.h"
#include <string.h>
#include <stdlib.h>

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES record_queue_init(RECORD_QUEUE* queue, int length){
    if(queue == NULL) return GNC_DATA_NULL;
    queue->length = length;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES record_queue_reset(RECORD_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES record_queue_pop(RECORD_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    if(queue->size == 0){
        return GNC_DATA_EMPTY;
    }
    queue->head = (queue->head + 1) % queue->length;
    queue->size--;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES record_queue_push(RECORD_QUEUE* queue, const PLAN_BLOCK* block){
    if(queue == NULL || block == NULL) return GNC_DATA_NULL;
    if(queue->size == queue->length){
        memcpy(&queue->block[queue->tail], block, sizeof(PLAN_BLOCK));
        queue->head = (queue->head + 1) % queue->length;
        queue->tail = (queue->tail + 1) % queue->length;
        return GNC_OPERATOR_SUCCESS;
    }
    memcpy(&queue->block[queue->tail], block, sizeof(PLAN_BLOCK));
    queue->tail = (queue->tail + 1) % queue->length;
    queue->size++;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES record_queue_soft_push(RECORD_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    if(queue->size == queue->length) return GNC_DATA_FULL;
    queue->tail = (queue->tail + 1) % queue->length;
    queue->size++;
    return GNC_OPERATOR_SUCCESS;
}

PLAN_BLOCK* record_queue_peak(const RECORD_QUEUE* queue){
    if(queue == NULL) return NULL;
    if(queue->size == 0) return NULL;
    return &queue->block[queue->head];
}

PLAN_BLOCK* record_queue_peak_tail(const RECORD_QUEUE* queue){
    if(queue == NULL) return NULL;
    if(queue->size == queue->length) return NULL;
    return &queue->block[queue->tail];
}

PLAN_BLOCK* record_queue_get_block(const RECORD_QUEUE* queue, int index){
    if(queue == NULL ) return NULL;
    if(index < 0 || index >= queue->size) return NULL;
    return &(queue->block[(queue->head + index) % queue->length]);
}

void record_queue_copy_block(const RECORD_QUEUE* queue, int index, PLAN_BLOCK* copy_block){
    if(queue == NULL) return;
    if(index < 0 || index >= queue->size) return;
    if(copy_block == NULL) return;
    memcpy(copy_block, &(queue->block[(queue->head + index) % queue->length]), sizeof(PLAN_BLOCK));
    return;
}

int record_queue_get_size(const RECORD_QUEUE* queue){
    if(queue == NULL) return -1;
    int size = queue->size;
    return size;
}

bool record_queue_is_empty(const RECORD_QUEUE* queue){
    if(queue == NULL) return true;
    int size = queue->size;
    return size == 0 ? true : false;
}

bool record_queue_is_full(const RECORD_QUEUE* queue){
    if(queue == NULL) return false;
    int size = queue->size;
    return size == queue->length ? true : false;
}

GNC_RES record_queue_delete(RECORD_QUEUE* queue){
    if (queue != NULL) {
        free(queue); 
        queue = NULL;
    }
    return GNC_OPERATOR_SUCCESS;
}
