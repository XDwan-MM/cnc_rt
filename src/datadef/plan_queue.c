#include "plan_queue.h"
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES queue_init(PLAN_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(PLAN_QUEUE_SIZE * sizeof(PLAN_BLOCK));
    if(pl_block == NULL) return GNC_DATA_NULL;
    queue->pl_block = pl_block;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    queue->isFirstRUN = true;
    queue->pre_head_vel = 0;
    queue->pre_head_type = RAPID;
    queue->next_vel = 0;
    queue->pre_pos = (POS){.x=0, .y=0, .z=0};
    queue->pre_vec = (VECTOR){.x=0, .y=0, .z=0};
    pthread_mutex_init(&queue->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_reset(PLAN_QUEUE* queue){
    if(queue == NULL || queue->pl_block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    queue->isFirstRUN = true;
    queue->pre_head_vel = 0;
    queue->pre_head_type = RAPID;
    queue->next_vel = 0;
    queue->pre_pos = (POS){.x=0, .y=0, .z=0};
    queue->pre_vec = (VECTOR){.x=0, .y=0, .z=0};
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_pop(PLAN_QUEUE* queue){
    if(queue == NULL || queue->pl_block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size == 0){
        pthread_mutex_unlock(&queue->mutex);
        return GNC_DATA_EMPTY;
    }
    queue->head = (queue->head + 1) % PLAN_QUEUE_SIZE;
    queue->size--;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_push(PLAN_QUEUE* queue, const PLAN_BLOCK* pl_block){
    if(queue == NULL || pl_block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size == PLAN_QUEUE_SIZE){
        pthread_mutex_unlock(&queue->mutex);
        return GNC_DATA_FULL;
    }
    memcpy(&queue->pl_block[queue->tail], pl_block, sizeof(PLAN_BLOCK));
    queue->tail = (queue->tail + 1) % PLAN_QUEUE_SIZE;
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_soft_push(PLAN_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    if(queue->size == PLAN_QUEUE_SIZE) return GNC_DATA_FULL;
    queue->tail = (queue->tail + 1) % PLAN_QUEUE_SIZE;
    pthread_mutex_lock(&queue->mutex);
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

PLAN_BLOCK* queue_peak(const PLAN_QUEUE* queue){
    if(queue == NULL || queue->pl_block == NULL) return NULL;
    return &queue->pl_block[queue->head];
}

PLAN_BLOCK* queue_peak_tail(const PLAN_QUEUE* queue){
    if(queue == NULL || queue->pl_block == NULL) return NULL;
    return &queue->pl_block[queue->tail];
}

PLAN_BLOCK* queue_get_block(const PLAN_QUEUE* queue, int index){
    if(queue == NULL || queue->pl_block == NULL) return NULL;
    if(index < 0 || index >= queue->size) return NULL;
    return &(queue->pl_block[(queue->head + index) % PLAN_QUEUE_SIZE]);
}

PLAN_BLOCK* queue_copy_block(const PLAN_QUEUE* queue, int index){
    if(queue == NULL || queue->pl_block == NULL) return NULL;
    if(index < 0 || index >= queue->size) return NULL;
    PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    if(pl_block == NULL) return NULL;
    memcpy(pl_block, &(queue->pl_block[(queue->head + index) % PLAN_QUEUE_SIZE]), sizeof(PLAN_BLOCK));
    return pl_block;
}

int queue_get_size(const PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->pl_block == NULL) return -1;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}

bool queue_is_empty(const PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->pl_block == NULL) return true;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size == 0 ? true : false;
}

bool queue_is_full(const PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->pl_block == NULL) return false;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return  size == PLAN_QUEUE_SIZE ? true : false;
}

GNC_RES queue_delete(PLAN_QUEUE* queue){
    if (queue != NULL && queue->pl_block != NULL) {
        pthread_mutex_destroy(&queue->mutex);
        free(queue->pl_block); 
        queue->pl_block = NULL; 
    }
    return GNC_OPERATOR_SUCCESS;
}
