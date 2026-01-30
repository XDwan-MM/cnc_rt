#include "task.h"
#include "eventdef.h"
#include <stdlib.h>

static PMC_EVENT event_data;
static GNC_RES task_handle_pmc_event(PMC_EVENT* event_data);

// 任务消息队列初始化
GNC_RES task_mq_init(){
    event_queue = (EVENT_QUEUE*)calloc(sizeof(EVENT_QUEUE), 1);
    return event_queue_init(event_queue);
}

// 消息队列处理（放在任务线程中调用循环）
GNC_RES task_mq_proc()
{
    if(event_queue_is_empty(event_queue)) return GNC_DATA_EMPTY;
    if(event_queue_is_full(event_queue)) return GNC_OPERATOR_ERROR; // error!
    if(sys->nc_event != SYS_EVENT_NONE) return GNC_OPERATOR_WARNING;
    event_queue_pop(event_queue, &event_data);
    task_handle_pmc_event(&event_data);
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES task_handle_pmc_event(PMC_EVENT* event_data){
    switch(event_data->id){
        case EV_MSG_NONE:
            sys->nc_event = SYS_EVENT_NONE;
            break;
        case EV_MSG_MODE: /**< 模式切换 (参数1: 模式索引) */
        {
            if(event_data->mode_payload.mode == 0)
            {
                sys->target_mode = MODE_EDIT;
            }
            if(event_data->mode_payload.mode == 1)
            {
                sys->target_mode = MODE_MEM;
            }
            if(event_data->mode_payload.mode == 2)
            {
                sys->target_mode = MODE_MDI;
            }
            if(event_data->mode_payload.mode == 3)
            {
                sys->target_mode = MODE_JOG;
            }
            if(event_data->mode_payload.mode == 4)
            {
                sys->target_mode = MODE_MPG;
            }
            if(event_data->mode_payload.mode == 5)
            {
                sys->target_mode = MODE_HOMING;
            }
            if(event_data->mode_payload.mode == 6)
            {
                sys->target_mode = MODE_DNC;
            }
            if(event_data->mode_payload.mode == 7)
            {
                // 下面我不确定
                sys->target_mode = MODE_IDLE;
            }
            break;
        }
        case EV_MSG_JOG: /**< 手动点动 (参数1: 轴号, 参数2: 方向) */
        {
            // 1.X轴点动事件
            if(event_data->jog_payload.axis_num == AXIS_X)
            {
                if(event_data->jog_payload.direction == 1)// 正向运动
                {
                    sys->nc_event = SYS_EVENT_X_POS_PRESS;
                }
                else if(event_data->jog_payload.direction == -1)// 负向运动
                {
                    sys->nc_event = SYS_EVENT_X_NEG_PRESS;
                }
                else if(event_data->jog_payload.direction == 0) //停止运动
                {
                    sys->nc_event = SYS_EVENT_X_POS_RELEASE;
                    sys->nc_event = SYS_EVENT_X_NEG_RELEASE;
                }
            }
            // 2.Y轴点动事件
            if(event_data->jog_payload.axis_num == AXIS_Y)
            {
                if(event_data->jog_payload.direction == 1)// 正向运动
                {
                    sys->nc_event = SYS_EVENT_Y_POS_PRESS;
                }
                else if(event_data->jog_payload.direction == -1)// 负向运动
                {
                    sys->nc_event = SYS_EVENT_Y_NEG_PRESS;
                }
                else if(event_data->jog_payload.direction == 0) //停止运动
                {
                    sys->nc_event = SYS_EVENT_Y_POS_RELEASE;
                    sys->nc_event = SYS_EVENT_Y_NEG_RELEASE;
                }
            }
            // 3.Z轴点动事件
            if(event_data->jog_payload.axis_num == AXIS_Z)
            {
                if(event_data->jog_payload.direction == 1)// 正向运动
                {
                    sys->nc_event = SYS_EVENT_Z_POS_PRESS;
                }
                else if(event_data->jog_payload.direction == -1)// 负向运动
                {
                    sys->nc_event = SYS_EVENT_Z_NEG_PRESS;
                }
                else if(event_data->jog_payload.direction == 0) //停止运动
                {
                    sys->nc_event = SYS_EVENT_Z_POS_RELEASE;
                    sys->nc_event = SYS_EVENT_Z_NEG_RELEASE;
                }
            }
            break;
        }
        case EV_MSG_SPINDLE:
        {
            if(event_data->spindle_payload.direction == 1)
            {
                // 主轴正转
            }
            if(event_data->spindle_payload.direction == -1)
            {
                // 主轴反转
            }
            if(event_data->spindle_payload.direction == 0)
            {
                // 主轴停止
            }
            break;
        }
        case EV_MSG_OVERRIDE:
        {
            // 1.进给倍率
            if(event_data->override_payload.override_type == 0)
            {

            }
            // 2.快速进给倍率
            if(event_data->override_payload.override_type == 1)
            {

            }
            // 3.主轴倍率
            if(event_data->override_payload.override_type == 2)
            {

            }
            break;
        }
        case EV_MSG_AUTO_RUN:
        {
            if(event_data->run_payload.run_state == 1)
            {
                // 循环启动
                sys->nc_event = SYS_EVENT_RERUN;
            }
            if(event_data->run_payload.run_state == 0)
            {
                // 进给保持
                sys->nc_event = SYS_EVENT_PAUSE;
            }
            break;
        }
        case EV_MSG_RESET:
        {
            if(event_data->run_payload.run_state == 1)
            {
                sys->nc_event = SYS_EVENT_RESET;
            }
            break;
        }
        default:
            break;
    }
    return GNC_OPERATOR_SUCCESS;
}
