#include "mcode_queue.h"

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES mcode_queue_init(MCODE_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    MCODE_BLOCK* block = (MCODE_BLOCK*)malloc(DCD_BUF_SIZE * sizeof(MCODE_BLOCK));
    if(block == NULL) return GNC_OPERATOR_ERROR;
    queue->block = block;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_init(&queue->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES mcode_queue_reset(MCODE_QUEUE* queue){
    if(queue == NULL || queue->block == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES mcode_queue_pop(MCODE_QUEUE* queue){
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

GNC_RES mcode_queue_push(MCODE_QUEUE* queue, MCODE_BLOCK* block){
    if(queue == NULL || block == NULL) return GNC_OPERATOR_ERROR;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size == DCD_BUF_SIZE) {
        pthread_mutex_unlock(&queue->mutex);
        return GNC_OPERATOR_ERROR;
    }
    memcpy(&queue->block[queue->tail], block, sizeof(MCODE_BLOCK));
    queue->tail = (queue->tail + 1) % DCD_BUF_SIZE;
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

MCODE_BLOCK* mcode_queue_peak(const MCODE_QUEUE* queue){
    if(queue == NULL || queue->block == NULL) return NULL;
    // MCODE_BLOCK* block = (MCODE_BLOCK*)malloc(sizeof(MCODE_BLOCK));
    // memcpy(block, &(queue->block[queue->head % DCD_BUF_SIZE]), sizeof(MCODE_BLOCK));
    // return block;
    return &(queue->block[queue->head % DCD_BUF_SIZE]);
}


MCODE_BLOCK* mcode_queue_get_block(const MCODE_QUEUE* queue, int index){
    if(queue == NULL || queue->block == NULL) return NULL;
    if(index < 0 || index >= queue->size) return NULL;
    return &(queue->block[(queue->head + index) % DCD_BUF_SIZE]);
}


int mcode_queue_get_size(const MCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return -1;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}

bool mcode_queue_is_empty(const MCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return true;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size == 0 ? true : false;
}

bool mcode_queue_is_full(const MCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->block == NULL) return false;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return  size == DCD_BUF_SIZE ? true : false;
}

GNC_RES mcode_queue_delete(MCODE_QUEUE* queue){
    if (queue != NULL && queue->block != NULL) {
        pthread_mutex_destroy(&queue->mutex);
        free(queue->block);
        queue->block = NULL;
    }
    return GNC_OPERATOR_SUCCESS;
}
