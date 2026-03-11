#include "task.h"
#include "eventdef.h"
#include "spindle.h"
#include "logapi.h"
#include "log.h"
#include <alarmapi.h>
#include <stdlib.h>
#include "regapi.h"   /* 引入寄存器访问 API */
#include "global.h"


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
    if(event_queue_is_full(event_queue)){
        // log_system("[task] 事件队列已满");
        // LOG_OUTPUT(ERROR, "事件队列已满");
        // setAlarm(ALARM_SY, ALARM_ERROR, 0);
        // return GNC_OPERATOR_ERROR;
    }
    // if(rt_data->nc_event != SYS_EVENT_NONE) return GNC_OPERATOR_WARNING;
    event_queue_pop(event_queue, &event_data);
    task_handle_pmc_event(&event_data);
    return GNC_OPERATOR_SUCCESS;
}

const char* mode[] = {"编辑", "记忆", "MDI", "JOG", "手轮", "回零", "DNC"};

static GNC_RES task_handle_pmc_event(PMC_EVENT* event_data){
    if (!event_data || !rt_data) return GNC_DATA_NULL;

    switch(event_data->id){
        case EV_MSG_NONE:
            rt_data->nc_event = SYS_EVENT_NONE;
            break;
        /* 1. 模式切换：更新目标模式 */
        case EV_MSG_MODE:
        {
            switch (event_data->mode_payload.mode) {
            case 0: rt_data->target_mode = MODE_EDIT;   break; // EDIT
            case 1: rt_data->target_mode = MODE_MEM;    break; // MEM
            case 2: rt_data->target_mode = MODE_MDI;    break; // MDI
            case 3: rt_data->target_mode = MODE_JOG;    break; // JOG
            case 4: rt_data->target_mode = MODE_MPG;    break; // 手轮
            case 5: rt_data->target_mode = MODE_HOMING; break; // 回零 (ZRN)
            case 6: rt_data->target_mode = MODE_DNC;    break; // DNC (DNCI)
            default: rt_data->target_mode = MODE_IDLE;  break; // wcf: 我不太确定
            }
            // log_system("[task] 处理模式切换事件(%d)", rt_data->target_mode);
            LOG_OUTPUT(INFO, "处理模式切换事件 %d", rt_data->target_mode);
            break;
        }
        /* 2. 进给轴按键处理：根据当前模式分发内部系统事件 */
        case EV_MSG_FEEDAXIS_KEY:
        {
            int axis = event_data->feedaxis_key_payload.axis_num;   // 0:X, 1:Y, 2:Z
            int direction = event_data->feedaxis_key_payload.direction; // 1:正按, -1:负按, 0:松开

            // 基于当前系统模式进行逻辑分发
            if (rt_data->current_mode == MODE_JOG || rt_data->current_mode == MODE_HOMING)
            {
                // axis为轴号
                if (axis == AXIS_X) {
                    if (direction == 1)
                    {
                        rt_data->nc_event = SYS_EVENT_X_POS_PRESS;
                        LOG_OUTPUT(INFO, "处理X+按下事件");
                    }
                    else if (direction == -1)
                    {
                        rt_data->nc_event = SYS_EVENT_X_NEG_PRESS;
                        LOG_OUTPUT(INFO, "处理X-按下事件");
                    }
                    else {
                        // 松开时，为了保险，可以根据 record_nc_event 或直接触发 Release
                        rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_X_POS_PRESS) ?
                                            SYS_EVENT_X_POS_RELEASE : SYS_EVENT_X_NEG_RELEASE;
                        LOG_OUTPUT(INFO, "处理X释放事件");
                    }
                }
                else if (axis == AXIS_Y) {
                    if (direction == 1)
                    {
                        rt_data->nc_event = SYS_EVENT_Y_POS_PRESS;
                        LOG_OUTPUT(INFO, "处理Y+按下事件");
                    }
                    else if (direction == -1)
                    {
                        rt_data->nc_event = SYS_EVENT_Y_NEG_PRESS;
                        LOG_OUTPUT(INFO, "处理Y-按下事件");
                    }
                    else {
                        rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_Y_POS_PRESS) ?
                                            SYS_EVENT_Y_POS_RELEASE : SYS_EVENT_Y_NEG_RELEASE;
                        LOG_OUTPUT(INFO, "处理Y释放事件");
                    }
                }
                else if (axis == AXIS_Z) {
                    if (direction == 1)
                    {
                        rt_data->nc_event = SYS_EVENT_Z_POS_PRESS;
                        LOG_OUTPUT(INFO, "处理Z+按下事件");
                    }
                    else if (direction == -1)
                    {
                        rt_data->nc_event = SYS_EVENT_Z_NEG_PRESS;
                        LOG_OUTPUT(INFO, "处理Z-按下事件");
                    }
                    else {
                        rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_Z_POS_PRESS) ?
                                            SYS_EVENT_Z_POS_RELEASE : SYS_EVENT_Z_NEG_RELEASE;
                        LOG_OUTPUT(INFO, "处理Z释放事件");
                    }
                }
                // log_system("[task] 处理进给轴按键事件 Axis %d action %d under mode %d", axis, direction, rt_data->current_mode);
                LOG_OUTPUT(INFO, "处理进给轴按键事件 Axis %d action %d under mode %d", axis, direction, rt_data->current_mode);
            }
            break;
        }
        case EV_MSG_SPINDLE:
        {
            if(event_data->spindle_payload.direction == 1)
            {
                spindle.state = DRV_STATE_OPERATION_ENABLED;
                spindle.mode = DRV_MODE_CSV;
                sp_sign = 1;
                // log_system("[task] 处理主轴正转事件");
                LOG_OUTPUT(INFO, "处理主轴正转事件");
            }
            if(event_data->spindle_payload.direction == -1)
            {
                spindle.state = DRV_STATE_OPERATION_ENABLED;
                spindle.mode = DRV_MODE_CSV;
                sp_sign = 2;
                // log_system("[task] 处理主轴反转事件");
                LOG_OUTPUT(INFO, "处理主轴反转事件");
            }
            if(event_data->spindle_payload.direction == 0)
            {
                sp_sign = 0;
                // log_system("[task] 处理主轴停止事件");
                LOG_OUTPUT(INFO, "处理主轴停止事件");
            }
            break;
        }
        case EV_MSG_OVERRIDE:
        {
            // 1.进给倍率
            if(event_data->override_payload.override_type == 0)
            {
                setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, (double)event_data->override_payload.val / 100, 0);
                rt_data->nc_event = SYS_EVENT_FEED_OVERRIDE;
                // log_system("[task] 处理进给倍率事件");
                LOG_OUTPUT(INFO, "处理进给倍率事件");
            }
            // 2.快速进给倍率
            if(event_data->override_payload.override_type == 1)
            {
                setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, (double)event_data->override_payload.val / 100, 0);
                rt_data->nc_event = SYS_EVENT_RAPID_OVERRIDE;
                // log_system("[task] 处理快速进给倍率事件");
                LOG_OUTPUT(INFO, "处理快速进给倍率事件");
            }
            // 3.主轴倍率
            if(event_data->override_payload.override_type == 2)
            {
                setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, (double)event_data->override_payload.val / 100, 0);
                rt_data->nc_event = SYS_EVENT_SPDL_OVERRIDE;
                // log_system("[task] 处理主轴倍率事件");
                LOG_OUTPUT(INFO, "处理主轴倍率事件");
            }
            break;
        }
        case EV_MSG_AUTO_RUN:
        {
            if(event_data->run_payload.run_state == 1)
            {
                if(rt_data->current_mode == MODE_MEM){
                    // 循环启动
                    rt_data->nc_event = SYS_EVENT_RERUN;
                    // log_system("[task] 处理循环启动事件");
                    LOG_OUTPUT(INFO, "处理循环启动事件");
                }
            }
            if(event_data->run_payload.run_state == 0)
            {
                if(rt_data->current_mode == MODE_MEM){
                    // 进给保持
                    rt_data->nc_event = SYS_EVENT_PAUSE;
                    // log_system("[task] 处理进给保持事件");
                    LOG_OUTPUT(INFO, "处理进给保持事件");
                }
            }
            break;
        }
        case EV_MSG_RESET:
        {
            if(event_data->reset_payload.reset_state == 1)
            {
                rt_data->nc_event = SYS_EVENT_RESET;
                // log_system("[task] 处理复位事件");
                LOG_OUTPUT(INFO, "处理复位事件");
                setRegBit(REG_F, 1, 1);
            }
            break;
        }
        case EV_MSG_ESP:
        {
            if (event_data->esp_payload.esp_state == 1)
            {
                // var_chan[0].is_estop = 1;
                LOG_OUTPUT(INFO, "处理急停拍下事件");
            } 
            if (event_data->esp_payload.esp_state == 0)
            {
                // var_chan[0].is_estop = 1;
                LOG_OUTPUT(INFO, "处理急停拉起事件");
            } 
        }
        default:
            break;
    }
    return GNC_OPERATOR_SUCCESS;
}
