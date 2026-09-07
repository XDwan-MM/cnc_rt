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
    PLC_EVENT* event_data = (PLC_EVENT*)malloc(EVENT_QUEUE_SIZE * sizeof(PLC_EVENT));
    if(event_data == NULL) return GNC_DATA_NULL;
    queue->event_data = event_data;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_init(&queue->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES event_queue_reset(EVENT_QUEUE* queue)
{
    if(queue == NULL || queue->event_data == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES event_queue_push(EVENT_QUEUE* queue, PLC_EVENT* event_data)
  {
    if(queue == NULL || queue->event_data == NULL || event_data == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size >= EVENT_QUEUE_SIZE) {
        pthread_mutex_unlock(&queue->mutex);
        return GNC_DATA_FULL;
    }
    memcpy(&queue->event_data[queue->tail], event_data, sizeof(PLC_EVENT));
    queue->tail = (queue->tail + 1) % EVENT_QUEUE_SIZE;
    queue->size++;
    pthread_mutex_unlock(&queue->mutex);
    return GNC_OPERATOR_SUCCESS;
  }

  GNC_RES event_queue_pop(EVENT_QUEUE* queue, PLC_EVENT* event_data)
  {
    if(queue == NULL || queue->event_data == NULL || event_data == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&queue->mutex);
    if(queue->size <= 0) {
        pthread_mutex_unlock(&queue->mutex);
        return GNC_DATA_EMPTY;
    }
    memcpy(event_data, &queue->event_data[queue->head], sizeof(PLC_EVENT));
    queue->head = (queue->head + 1) % EVENT_QUEUE_SIZE;
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
