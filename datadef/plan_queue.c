#include "plan_queue.h"

/// @brief 初始化循环队列
/// @param queue 队列
/// @return 
GNC_RES queue_init(PLAN_QUEUE* queue){
    PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(PLAN_QUEUE_SIZE * sizeof(PLAN_BLOCK));
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
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_reset(PLAN_QUEUE* queue){
    if(queue->pl_block == NULL) return GNC_DATA_NULL;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
    queue->isFirstRUN = true;
    queue->pre_head_vel = 0;
    queue->pre_head_type = RAPID;
    queue->next_vel = 0;
    queue->pre_pos = (POS){.x=0, .y=0, .z=0};
    queue->pre_vec = (VECTOR){.x=0, .y=0, .z=0};
    return GNC_OPERATOR_SUCCESS;
}

PLAN_BLOCK* queue_pop(PLAN_QUEUE* queue){
    if(queue == NULL) return NULL;
    if(queue->size == 0) return NULL;
    
    PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    memcpy(pl_block, &(queue->pl_block[queue->head]), sizeof(PLAN_BLOCK));
    queue->head = (queue->head + 1) % PLAN_QUEUE_SIZE;
    queue->size--;
    return pl_block;
}

GNC_RES queue_push(PLAN_QUEUE* queue, PLAN_BLOCK* pl_block){
    if(queue == NULL) return GNC_OPERATOR_ERROR;
    if(queue->size == PLAN_QUEUE_SIZE) return GNC_OPERATOR_ERROR;
    memcpy(&queue->pl_block[queue->tail], pl_block, sizeof(PLAN_BLOCK));
    queue->tail = (queue->tail + 1) % PLAN_QUEUE_SIZE;
    queue->size++;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES queue_soft_push(PLAN_QUEUE* queue){
    if(queue == NULL) return GNC_OPERATOR_ERROR;
    if(queue->size == PLAN_QUEUE_SIZE) return GNC_OPERATOR_ERROR;
    queue->tail = (queue->tail + 1) % PLAN_QUEUE_SIZE;
    queue->size++;
    return GNC_OPERATOR_SUCCESS;
}

PLAN_BLOCK* queue_peak(PLAN_QUEUE* queue){
    if(queue == NULL) return NULL;
    // PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    // memcpy(pl_block, &(queue->pl_block[queue->head]), sizeof(PLAN_BLOCK));
    // return pl_block;
    return &queue->pl_block[queue->head];
}

PLAN_BLOCK* queue_peak_tail(PLAN_QUEUE* queue){
    if(queue == NULL) return NULL;
    // PLAN_BLOCK* pl_block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    // memcpy(pl_block, &(queue->pl_block[queue->head]), sizeof(PLAN_BLOCK));
    // return pl_block;
    return &queue->pl_block[queue->tail];
}

PLAN_BLOCK* queue_get_block(PLAN_QUEUE* queue, int index){
    if(queue == NULL) return NULL;
    if(index < 0 || index > queue->size) return NULL;
    return &(queue->pl_block[(queue->head + index) % PLAN_QUEUE_SIZE]);
}


int queue_get_size(PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return queue->size;
}

bool queue_is_empty(PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return queue->size == 0 ? true : false;
}

bool queue_is_full(PLAN_QUEUE* queue){
    // VALIDATE_ARGUMENT(queue != NULL, "queue","queue is null");
    return  queue->size == PLAN_QUEUE_SIZE ? true : false;
}

GNC_RES queue_delete(PLAN_QUEUE* queue){
    if (queue != NULL) {
        free(queue->pl_block); 
        queue->pl_block = NULL; 
    }
    return GNC_OPERATOR_SUCCESS;
}


// GNC_RES queue_pop(PLAN_QUEUE* queue, PLAN_BLOCK* pl_block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(queue->size == 0) return GNC_OPERATOR_ERROR;
    
//     memcpy(pl_block, &(queue->pl_block[queue->head]), sizeof(PLAN_BLOCK));
//     queue->head = (queue->head + 1) % PLAN_QUEUE_SIZE;
//     queue->size--;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES queue_peak(PLAN_QUEUE* queue, PLAN_BLOCK* pl_block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(queue->size == 0) return GNC_OPERATOR_ERROR;
    
//     memcpy(pl_block, &(queue->pl_block[queue->head]), sizeof(PLAN_BLOCK));
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES queue_get_block(PLAN_QUEUE* queue, int index, PLAN_BLOCK** pl_block){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     if(index < 0 || index >= queue->size) return GNC_OPERATOR_ERROR;
//     *pl_block = &(queue->pl_block[(queue->head + index) % PLAN_QUEUE_SIZE]);
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES queue_get_size(PLAN_QUEUE* queue, int* size){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;

//     *size = queue->size;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES queue_is_empty(PLAN_QUEUE* queue, bool* isEmpty){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     *isEmpty = queue->size == 0 ? true : false;
//     return GNC_OPERATOR_SUCCESS;
// }

// GNC_RES queue_is_full(PLAN_QUEUE* queue, bool* isFull){
//     if(queue == NULL) return GNC_OPERATOR_ERROR;
//     *isFull = queue->size == PLAN_QUEUE_SIZE ? true : false;
//     return GNC_OPERATOR_SUCCESS;
// }
