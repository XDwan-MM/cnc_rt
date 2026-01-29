#include "task.h"
#include "eventdef.h"
#include "global.h"
#include <stdlib.h>

EVENT_DATA event_data;

static GNC_RES task_handle_sys_event(EVENT_DATA event_data);

// 任务消息队列初始化
GNC_RES task_mq_init(){
    event_queue = (EVENT_QUEUE*)calloc(sizeof(EVENT_QUEUE), 1);
    return event_queue_init(event_queue);
}

// 消息队列处理（放在任务线程中调用循环）
GNC_RES task_mq_proc(){
    if(event_queue_is_empty(event_queue)) return GNC_DATA_EMPTY;
    if(event_queue_is_full(event_queue)) return GNC_OPERATOR_ERROR; // error!
    if(sys->nc_event != SYS_EVENT_NONE) return GNC_OPERATOR_WARNING;
    event_queue_pop(event_queue, &event_data);
    switch(event_data.src){
        case EV_SRC_SYS:
            task_handle_sys_event(event_data);
            break;
        default:
            break;
    }
}

static GNC_RES task_handle_sys_event(EVENT_DATA event_data){
    switch(event_data.code){
        case ncEvtRstPress:
            sys->nc_event = SYS_EVENT_RESET;
            break;
        case ncEvtCycPress:
            sys->nc_event = SYS_EVENT_RERUN;
            break;
        case ncEvtHoldPress:
            sys->nc_event = SYS_EVENT_PAUSE;
            break;
        case ncEvtXPosPress:
            sys->nc_event = SYS_EVENT_X_POS_PRESS;
            break;
        case ncEvtXNegPress:
            sys->nc_event = SYS_EVENT_X_NEG_PRESS;
            break;
        case ncEvtYPosPress:
            sys->nc_event = SYS_EVENT_Y_POS_PRESS;
            break;
        case ncEvtYNegPress:
            sys->nc_event = SYS_EVENT_Y_NEG_PRESS;
            break;
        case ncEvtZPosPress:
            sys->nc_event = SYS_EVENT_Z_POS_PRESS;
            break;
        case ncEvtZNegPress:
            sys->nc_event = SYS_EVENT_Z_NEG_PRESS;
            break;
        case ncEvtXPosRelease:
            sys->nc_event = SYS_EVENT_X_POS_RELEASE;
            break;
        case ncEvtXNegRelease:
            sys->nc_event = SYS_EVENT_X_NEG_RELEASE;
            break;
        case ncEvtYPosRelease:
            sys->nc_event = SYS_EVENT_Y_POS_RELEASE;
            break;
        case ncEvtYNegRelease:
            sys->nc_event = SYS_EVENT_Y_NEG_RELEASE;
            break;
        case ncEvtZPosRelease:
            sys->nc_event = SYS_EVENT_Z_POS_RELEASE;
            break;
        case ncEvtZNegRelease:
            sys->nc_event = SYS_EVENT_Z_NEG_RELEASE;
            break;
        // 控制模式
        case ncEvtJogPress:
            sys->target_mode = MODE_JOG;
            break;
        case ncEvtRapidPress:
            sys->target_mode = MODE_RAPID_JOG;
            break;
        case ncEvtMemPress:
            sys->target_mode = MODE_MEM;
            break;
        case ncEvtEStopPress:
            sys->target_mode = MODE_ALARM;
            break;
        default:
            break;
    }
    return GNC_OPERATOR_SUCCESS;
}
