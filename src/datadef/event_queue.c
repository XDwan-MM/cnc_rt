#include "event_queue.h"
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES event_queue_init(EVENT_QUEUE* queue){
    if(queue == NULL) return GNC_DATA_NULL;
    EVENT_DATA* event_data = (EVENT_DATA*)malloc(EVENT_QUEUE_SIZE * sizeof(EVENT_DATA));
    if(event_data == NULL) return GNC_DATA_NULL;
    queue->event_data = event_data;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_init(&queue->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES event_queue_reset(EVENT_QUEUE* queue){
    if(queue == NULL || queue->event_data == NULL) return GNC_DATA_NULL;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES event_queue_push(EVENT_QUEUE* queue, EVENT_DATA* event_data){
    if(queue == NULL || queue->event_data == NULL || event_data == NULL) return GNC_DATA_NULL;
    if(queue->size >= EVENT_QUEUE_SIZE) return GNC_DATA_FULL;
    memcpy(&(queue->event_data[queue->tail]), event_data, sizeof(EVENT_DATA));
    queue->tail = (queue->tail + 1) % EVENT_QUEUE_SIZE;
    pthread_mutex_lock(&queue->mutex);
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES event_queue_pop(EVENT_QUEUE* queue, EVENT_DATA* event_data){
    if(queue == NULL || queue->event_data == NULL || event_data == NULL) return GNC_DATA_FULL;
    if(queue->size <= 0) return GNC_DATA_NULL;
    memcpy(event_data, &(queue->event_data[queue->head]), sizeof(EVENT_DATA));
    queue->head = (queue->head + 1) % EVENT_QUEUE_SIZE;
    pthread_mutex_lock(&queue->mutex);
    queue->size--;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

int event_queue_get_size(EVENT_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->event_data == NULL) return 0;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}

bool event_queue_is_empty(EVENT_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->event_data == NULL) return true;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size == 0 ? true : false;
}

bool event_queue_is_full(EVENT_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    if(queue == NULL || queue->event_data == NULL) return false;
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return  size == EVENT_QUEUE_SIZE ? true : false;
}

GNC_RES event_queue_delete(EVENT_QUEUE* queue){
    if (queue != NULL && queue->event_data != NULL) {
        pthread_mutex_destroy(&queue->mutex);
        free(queue->event_data); 
        queue->event_data = NULL; 
        return GNC_OPERATOR_SUCCESS;
    }
    return GNC_DATA_NULL;
}