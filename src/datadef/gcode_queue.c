#include "gcode_queue.h"

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES gcode_queue_init(GCODE_QUEUE* queue){
    if(queue == NULL) return GNC_OPERATOR_ERROR;
    GCODE_BLOCK* block = (GCODE_BLOCK*)malloc(DCD_BUF_SIZE * sizeof(GCODE_BLOCK));
    if(block == NULL) return GNC_OPERATOR_ERROR;
    queue->block = block;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_init(&queue->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES gcode_queue_reset(GCODE_QUEUE* queue){
    if(queue == NULL || queue->block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES gcode_queue_pop(GCODE_QUEUE* queue){
    if(queue == NULL || queue->block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return GNC_DATA_EMPTY;
    }
    queue->head = (queue->head + 1) % DCD_BUF_SIZE;
    queue->size--;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES gcode_queue_push(GCODE_QUEUE* queue, const GCODE_BLOCK* block){
    if(queue == NULL || block == NULL) return GNC_OPERATOR_ERROR;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size == DCD_BUF_SIZE) {
        pthread_mutex_unlock(&queue->mutex);
        return GNC_OPERATOR_ERROR;
    }
    memcpy(&queue->block[queue->tail], block, sizeof(GCODE_BLOCK));
    queue->tail = (queue->tail + 1) % DCD_BUF_SIZE;
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GCODE_BLOCK* gcode_queue_peak(const GCODE_QUEUE* queue){
    if(queue == NULL || queue->block == NULL) return NULL;
    if(queue->size == 0) return NULL;
    return &(queue->block[queue->head % DCD_BUF_SIZE]);
}


GCODE_BLOCK* gcode_queue_get_block(const GCODE_QUEUE* queue, int index){
    if(queue == NULL || queue->block == NULL) return NULL;
    if(index < 0 || index >= queue->size) return NULL;
    return &(queue->block[(queue->head + index) % DCD_BUF_SIZE]);
}


int gcode_queue_get_size(const GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return -1;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}

bool gcode_queue_is_empty(const GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return true;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size == 0 ? true : false;
}

bool gcode_queue_is_full(const GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return false;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return  size == DCD_BUF_SIZE ? true : false;
}

GNC_RES gcode_queue_delete(GCODE_QUEUE* queue){
    if (queue != NULL && queue->block != NULL) {
        pthread_mutex_destroy(&queue->mutex);
        free(queue->block);
        queue->block = NULL;
    }
    return GNC_OPERATOR_SUCCESS;
}
