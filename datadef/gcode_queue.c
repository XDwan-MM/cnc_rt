#include "gcode_queue.h"

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES gcode_queue_init(GCODE_QUEUE* queue){
    GCODE_BLOCK* block = (GCODE_BLOCK*)malloc(DCD_BUF_SIZE * sizeof(GCODE_BLOCK));
    queue->block = block;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES gcode_queue_reset(GCODE_QUEUE* queue){
    if(queue->block == NULL) return GNC_DATA_NULL;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    return GNC_OPERATOR_SUCCESS;
}

GCODE_BLOCK* gcode_queue_pop(GCODE_QUEUE* queue){
    if(queue == NULL) return NULL;
    if(queue->size == 0) return NULL;
    
    GCODE_BLOCK* block = (GCODE_BLOCK*)malloc(sizeof(GCODE_BLOCK));
    memcpy(block, &(queue->block[queue->head]), sizeof(GCODE_BLOCK));
    queue->head = (queue->head + 1) % DCD_BUF_SIZE;
    queue->size--;
    return block;
}

GNC_RES gcode_queue_push(GCODE_QUEUE* queue, GCODE_BLOCK* block){
    if(queue == NULL) return GNC_OPERATOR_ERROR;
    if(queue->size == DCD_BUF_SIZE) return GNC_OPERATOR_ERROR;
    
    memcpy(&queue->block[queue->tail], block, sizeof(GCODE_BLOCK));
    queue->tail = (queue->tail + 1) % DCD_BUF_SIZE;
    queue->size++;
    return GNC_OPERATOR_SUCCESS;
}

GCODE_BLOCK* gcode_queue_peak(GCODE_QUEUE* queue){
    if(queue == NULL) return NULL;
    // GCODE_BLOCK* block = (GCODE_BLOCK*)malloc(sizeof(GCODE_BLOCK));
    // memcpy(block, &(queue->block[queue->head % DCD_BUF_SIZE]), sizeof(GCODE_BLOCK));
    // return block;
    return &(queue->block[queue->head % DCD_BUF_SIZE]);
}


GCODE_BLOCK* gcode_queue_get_block(GCODE_QUEUE* queue, int index){
    if(queue == NULL) return NULL;
    if(index < 0 || index > queue->size) return NULL;
    return &(queue->block[(queue->head + index) % DCD_BUF_SIZE]);
}


int gcode_queue_get_size(GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return queue->size;
}

bool gcode_queue_is_empty(GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return queue->size == 0 ? true : false;
}

bool gcode_queue_is_full(GCODE_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return  queue->size == DCD_BUF_SIZE ? true : false;
}

GNC_RES gcode_queue_delete(GCODE_QUEUE* queue){
    if (queue != NULL) {
        free(queue->block); 
        queue->block = NULL; 
    }
    return GNC_OPERATOR_SUCCESS;
}


// GNC_RES gcode_queue_pop(GCODE_QUEUE* queue, GCODE_BLOCK* block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(queue->size == 0) return GNC_OPERATOR_ERROR;
    
//     memcpy(block, &(queue->block[queue->head]), sizeof(GCODE_BLOCK));
//     queue->head = (queue->head + 1) % DCD_BUF_SIZE;
//     queue->size--;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES gcode_queue_peak(GCODE_QUEUE* queue, GCODE_BLOCK* block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(queue->size == 0) return GNC_OPERATOR_ERROR;
    
//     memcpy(block, &(queue->block[queue->head]), sizeof(GCODE_BLOCK));
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES gcode_queue_get_block(GCODE_QUEUE* queue, int index, GCODE_BLOCK** block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(index < 0 || index >= queue->size) return GNC_OPERATOR_ERROR;
//     *block = &(queue->block[(queue->head + index) % DCD_BUF_SIZE]);
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES gcode_queue_get_size(GCODE_QUEUE* queue, int* size){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;

//     *size = queue->size;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES gcode_queue_is_empty(GCODE_QUEUE* queue, bool* isEmpty){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     *isEmpty = queue->size == 0 ? true : false;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES gcode_queue_is_full(GCODE_QUEUE* queue, bool* isFull){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     *isFull = queue->size == DCD_BUF_SIZE ? true : false;
//     return GNC_OPERATOR_SUCCESS;
// }
