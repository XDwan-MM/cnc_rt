#include "task.h"
#include "eventdef.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <stdlib.h>
#include "regapi.h"   /* 引入寄存器访问 API */
#include "global.h"
#include "spl_control.h"

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
        // LOG(LOG_LEVEL_ERR, "事件队列已满");
        // setAlarm(ALARM_SY, ALARM_ERROR, 0);
        // return GNC_OPERATOR_ERROR;
    }
    event_queue_pop(event_queue, &event_data);
    task_handle_pmc_event(&event_data);
    return GNC_OPERATOR_SUCCESS;
}

const char* mode[] = {"编辑", "记忆", "MDI", "JOG", "手轮", "回零", "DNC", "未知"};

static GNC_RES task_handle_pmc_event(PMC_EVENT* event_data)
{
    if (!event_data || !rt_data)
        return GNC_DATA_NULL;

    switch (event_data->id)
    {
    case EV_MSG_NONE:
        rt_data->nc_event = SYS_EVENT_NONE;
        break;
    /* 1. 模式切换：更新目标模式 */
    case EV_MSG_MODE:
    {
        switch (event_data->mode_payload.mode)
        {
        case CNC_MODE_EDIT:
            rt_data->target_mode = MODE_EDIT;
            break; // EDIT
        case CNC_MODE_MEM:
            rt_data->target_mode = MODE_MEM;
            break; // MEM
        case CNC_MODE_MDI:
            rt_data->target_mode = MODE_MDI;
            break; // MDI
        case CNC_MODE_JOG:
            rt_data->target_mode = MODE_JOG;
            break; // JOG
        case CNC_MODE_MPG:
            rt_data->target_mode = MODE_MPG;
            break; // 手轮
        case CNC_MODE_ZRN:
            rt_data->target_mode = MODE_HOMING;
            break; // 回零 (ZRN)
        case CNC_MODE_DNC:
            rt_data->target_mode = MODE_DNC;
            break; // DNC
        case CNC_MODE_RAPID:
            rt_data->target_mode = MODE_RAPID_JOG;
            break; // RAPID
        default:
            rt_data->target_mode = MODE_IDLE;
            break; // wcf: 我不太确定
        }
        LOG(LOG_LEVEL_INFO, "处理模式切换事件 %d", rt_data->target_mode);
        break;
    }
    /* 2. 进给轴按键处理：根据当前模式分发内部系统事件 */
    case EV_MSG_FEEDAXIS_KEY:
    {
        int axis = event_data->feedaxis_key_payload.axis_num;       // 0:X, 1:Y, 2:Z
        int direction = event_data->feedaxis_key_payload.direction; // 1:正按, -1:负按, 0:松开

        // 基于当前系统模式进行逻辑分发
        if (rt_data->current_mode == MODE_JOG || rt_data->current_mode == MODE_HOMING)
        {
            // axis为轴号
            if (axis == AXIS_X)
            {
                if (direction == 1)
                {
                    rt_data->nc_event = SYS_EVENT_X_POS_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理X+按下事件");
                }
                else if (direction == -1)
                {
                    rt_data->nc_event = SYS_EVENT_X_NEG_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理X-按下事件");
                }
                else
                {
                    // 松开时，为了保险，可以根据 record_nc_event 或直接触发 Release
                    rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_X_POS_PRESS) ? SYS_EVENT_X_POS_RELEASE : SYS_EVENT_X_NEG_RELEASE;
                    LOG(LOG_LEVEL_INFO, "处理X释放事件");
                }
            }
            else if (axis == AXIS_Y)
            {
                if (direction == 1)
                {
                    rt_data->nc_event = SYS_EVENT_Y_POS_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理Y+按下事件");
                }
                else if (direction == -1)
                {
                    rt_data->nc_event = SYS_EVENT_Y_NEG_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理Y-按下事件");
                }
                else
                {
                    rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_Y_POS_PRESS) ? SYS_EVENT_Y_POS_RELEASE : SYS_EVENT_Y_NEG_RELEASE;
                    LOG(LOG_LEVEL_INFO, "处理Y释放事件");
                }
            }
            else if (axis == AXIS_Z)
            {
                if (direction == 1)
                {
                    rt_data->nc_event = SYS_EVENT_Z_POS_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理Z+按下事件");
                }
                else if (direction == -1)
                {
                    rt_data->nc_event = SYS_EVENT_Z_NEG_PRESS;
                    LOG(LOG_LEVEL_INFO, "处理Z-按下事件");
                }
                else
                {
                    rt_data->nc_event = (rt_data->nc_event == SYS_EVENT_Z_POS_PRESS) ? SYS_EVENT_Z_POS_RELEASE : SYS_EVENT_Z_NEG_RELEASE;
                    LOG(LOG_LEVEL_INFO, "处理Z释放事件");
                }
            }
            LOG(LOG_LEVEL_INFO, "处理进给轴按键事件 Axis %d action %d under mode %d", axis, direction, rt_data->current_mode);
        }
        break;
    }
    case EV_MSG_SPINDLE:
    {
        if(event_data->spindle_payload.direction == 1)
        {
            spl_data.spl_event = SPINDLE_EVENT_CW;
            spl_data.emot_speed = spl_data.mcp_target_speed;
            LOG(LOG_LEVEL_INFO, "处理主轴正转事件");
        }
        if(event_data->spindle_payload.direction == -1)
        {
            spl_data.spl_event = SPINDLE_EVENT_CCW;
            spl_data.emot_speed = -1.0 * spl_data.mcp_target_speed;
            LOG(LOG_LEVEL_INFO, "处理主轴反转事件");
        }
        if(event_data->spindle_payload.direction == 0)
        {
            spl_data.spl_event = SPINDLE_EVENT_STOP;
            spl_data.emot_speed = 0;
            LOG(LOG_LEVEL_INFO, "处理主轴停止事件");
        }
        break;
    }
    case EV_MSG_OVERRIDE:
    {
        // 1.进给倍率
        if (event_data->override_payload.override_type == 0)
        {
            setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, (double)event_data->override_payload.val, 0);
            rt_data->nc_event = SYS_EVENT_FEED_OVERRIDE;
            LOG(LOG_LEVEL_INFO, "处理进给倍率事件 %d", event_data->override_payload.val);
        }
        // 2.快速进给倍率
        if (event_data->override_payload.override_type == 1)
        {
            setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, (double)event_data->override_payload.val, 0);
            rt_data->nc_event = SYS_EVENT_RAPID_OVERRIDE;
            LOG(LOG_LEVEL_INFO, "处理快速进给倍率事件 %d", event_data->override_payload.val);
        }
        // 3.主轴倍率
        if (event_data->override_payload.override_type == 2)
        {
            setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, (double)event_data->override_payload.val, 0);
            rt_data->nc_event = SYS_EVENT_SPDL_OVERRIDE;
            LOG(LOG_LEVEL_INFO, "处理主轴倍率事件 %d", event_data->override_payload.val);
        }
        break;
    }
    case EV_MSG_AUTO_RUN:
    {
        if (event_data->run_payload.run_state == 1)
        {
            if (rt_data->current_mode == MODE_MEM)
            {
                // 循环启动
                rt_data->nc_event = SYS_EVENT_RERUN;
                LOG(LOG_LEVEL_INFO, "处理循环启动事件");
            }
        }
        if (event_data->run_payload.run_state == 0)
        {
            if (rt_data->current_mode == MODE_MEM)
            {
                // 进给保持
                rt_data->nc_event = SYS_EVENT_PAUSE;
                LOG(LOG_LEVEL_INFO, "处理进给保持事件");
            }
        }
        break;
    }
    case EV_MSG_RESET:
    {
        if (event_data->reset_payload.reset_state == 1)
        {
            rt_data->nc_event = SYS_EVENT_RESET;
            LOG(LOG_LEVEL_INFO, "处理复位事件");
            setRegBit(REG_F, 1, 1);
        }
        break;
    }
    case EV_MSG_ENANLE:
        switch(event_data->enable_payload.enable_key){
            case BLOCK_SKIP:
                rt_data->sys_state[SYS_IS_BLOCK_SKIP] = event_data->enable_payload.enable_state;
                break;
            case SINGLE_BLOCK:
                rt_data->sys_state[SYS_IS_BLOCK_SKIP] = event_data->enable_payload.enable_state;
                break;
            case DRY_RUN:
                rt_data->sys_state[SYS_IS_SINGLE_BLOCK] = event_data->enable_payload.enable_state;
                break;
            case MAC_LOCK:
                rt_data->sys_state[SYS_IS_MAC_LOCK] = event_data->enable_payload.enable_state;
                break;
            case MSBT_LOCK:
                rt_data->sys_state[SYS_IS_MSBT_LOCK] = event_data->enable_payload.enable_state;
                break;   
            default:
                break; 
        }
        break;
    default:
        break;
    }
    return GNC_OPERATOR_SUCCESS;
}
