#include "task.h"
#include "eventdef.h"
#include "global.h"
#include "hmi_scanner.h"
#include "regapi.h"
#include "spl_control.h"
#include "servosampleevents.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <stdlib.h>
#include "reset.h"

static PMC_EVENT event_data;
static GNC_RES task_handle_pmc_event(PMC_EVENT *event_data);
static void task_handle_mode_event(PMC_EVENT *event_data);
static void task_handle_feed_event(PMC_EVENT *event_data);
static void task_handle_spl_event(PMC_EVENT *event_data);
static void task_handle_overrate_event(PMC_EVENT *event_data);
static void task_handle_auto_event(PMC_EVENT *event_data);
static void task_handle_enable_event(PMC_EVENT *event_data);
static void task_handle_reset_event(PMC_EVENT *event_data);
static void task_handle_hmi_event(PMC_EVENT *event_data);

// 任务消息队列初始化
GNC_RES task_mq_init()
{
    event_queue = (EVENT_QUEUE *)calloc(sizeof(EVENT_QUEUE), 1);
    return event_queue_init(event_queue);
}

// 消息队列处理
GNC_RES task_mq_proc()
{
    if (event_queue_is_full(event_queue))
    {
        LOG(LOG_LEVEL_ERR, "事件队列已满");
        // setAlarm(ALARM_SY, ALARM_ERROR, 0);
        // return GNC_OPERATOR_ERROR;
    }
    // 弹出事件
    int operator_res = event_queue_pop(event_queue, &event_data);
    if (operator_res != GNC_OPERATOR_SUCCESS)
    {
        return GNC_DATA_EMPTY;
    }
    // 处理事件
    task_handle_pmc_event(&event_data);
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES task_handle_pmc_event(PMC_EVENT *event_data)
{
    if (!event_data || !rt_data)
        return GNC_DATA_NULL;

    switch (event_data->id)
    {
    case EV_MSG_NONE:
    {
        rt_data->nc_event = SYS_EVENT_NONE;
        break;
    }
    case EV_MSG_MODE:
    {
        task_handle_mode_event(event_data);
        break;
    }
    case EV_MSG_FEEDAXIS_KEY:
    {
        task_handle_feed_event(event_data);
        break;
    }
    case EV_MSG_SPINDLE:
    {
        task_handle_spl_event(event_data);
        break;
    }
    case EV_MSG_OVERRIDE:
    {
        task_handle_overrate_event(event_data);
        break;
    }
    case EV_MSG_AUTO_RUN:
    {
        task_handle_auto_event(event_data);
        break;
    }
    case EV_MSG_ENANLE:
    {
        task_handle_enable_event(event_data);
        break;
    }
    case EV_MSG_HMI:
    {
        task_handle_hmi_event(event_data);
        break;
    }
    case EV_MSG_RESET:
    {
        task_handle_reset_event(event_data);
        break;
    }
    case EV_MSG_SPD_ORI:
    {
        LOG(LOG_LEVEL_INFO, "处理主轴定向事件");
        break;
    }
    default:
        break;
    }
    return GNC_OPERATOR_SUCCESS;
}

static void task_handle_mode_event(PMC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO, "处理模式切换事件 %d", rt_data->target_mode);
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
    default:
        rt_data->target_mode = MODE_IDLE;
        break; // wcf: 我不太确定
    }
}

static void task_handle_feed_event(PMC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO,
        "处理进给轴按键事件 轴选键%d 动作%d 当前模式%d",
        event_data->feedaxis_key_payload.axis_num,
        event_data->feedaxis_key_payload.direction,
        rt_data->current_mode);
    int axis = event_data->feedaxis_key_payload.axis_num;       // 0:X, 1:Y, 2:Z
    int direction = event_data->feedaxis_key_payload.direction; // 1:正按, -1:正释放, 2:负按，-2:负释放

    // 基于当前系统模式进行逻辑分发
    if (rt_data->current_mode == MODE_JOG || rt_data->current_mode == MODE_HOMING)
    {
        // axis为轴号
        if (axis == AXIS_X)
        {
            switch (direction)
            {
            case 1:
                rt_data->nc_event = SYS_EVENT_X_POS_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理X+按下事件");
                break;
            case -1:
                rt_data->nc_event = SYS_EVENT_X_POS_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理X+释放事件");
                break;
            case 2:
                rt_data->nc_event = SYS_EVENT_X_NEG_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理X-按下事件");
                break;
            case -2:
                rt_data->nc_event = SYS_EVENT_X_NEG_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理X-释放事件");
                break;
            default:
                rt_data->nc_event = SYS_EVENT_NONE;
                break;
            }
        }
        else if (axis == AXIS_Y)
        {
            switch (direction)
            {
            case 1:
                rt_data->nc_event = SYS_EVENT_Y_POS_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理Y+按下事件");
                break;
            case -1:
                rt_data->nc_event = SYS_EVENT_Y_POS_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理Y+释放事件");
                break;
            case 2:
                rt_data->nc_event = SYS_EVENT_Y_NEG_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理Y-按下事件");
                break;
            case -2:
                rt_data->nc_event = SYS_EVENT_Y_NEG_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理Y-释放事件");
                break;
            default:
                rt_data->nc_event = SYS_EVENT_NONE;
                break;
            }
        }
        else if (axis == AXIS_Z)
        {
            switch (direction)
            {
            case 1:
                rt_data->nc_event = SYS_EVENT_Z_POS_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理Z+按下事件");
                break;
            case -1:
                rt_data->nc_event = SYS_EVENT_Z_POS_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理Z+释放事件");
                break;
            case 2:
                rt_data->nc_event = SYS_EVENT_Z_NEG_PRESS;
                // LOG(LOG_LEVEL_INFO, "处理Z-按下事件");
                break;
            case -2:
                rt_data->nc_event = SYS_EVENT_Z_NEG_RELEASE;
                // LOG(LOG_LEVEL_INFO, "处理Z-释放事件");
                break;
            default:
                rt_data->nc_event = SYS_EVENT_NONE;
                break;
            }
        }
    }
}

static void task_handle_spl_event(PMC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO, "处理主轴事件 动作%d", event_data->spindle_payload.direction);
    if (event_data->spindle_payload.direction == 1)
    {
        // LOG(LOG_LEVEL_INFO, "处理主轴正转事件");
        spl_data.spl_event = SPINDLE_EVENT_CW;
        spl_data.emot_speed = spl_data.mcp_target_speed;
    }
    if (event_data->spindle_payload.direction == -1)
    {
        // LOG(LOG_LEVEL_INFO, "处理主轴反转事件");
        spl_data.spl_event = SPINDLE_EVENT_CCW;
        spl_data.emot_speed = -1.0 * spl_data.mcp_target_speed;
    }
    if (event_data->spindle_payload.direction == 0)
    {
        // LOG(LOG_LEVEL_INFO, "处理主轴停止事件");
        spl_data.spl_event = SPINDLE_EVENT_STOP;
        spl_data.emot_speed = 0;
    }
}

static void task_handle_overrate_event(PMC_EVENT *event_data)
{
    // 1.进给倍率
    if (event_data->override_payload.override_type == 0)
    {
        LOG(LOG_LEVEL_INFO, "处理进给倍率事件 倍率值%d", event_data->override_payload.val);
        setVarChanDouble(0, VAR_CHAN_FEED_OVERRIDE, (double)event_data->override_payload.val, 0);
        rt_data->nc_event = SYS_EVENT_FEED_OVERRIDE;
        if (IsEqualTo(var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f, 0))
        {
            setAlarm(ALARM_SY, ALARM_WARNING, 3);
            LOG(LOG_LEVEL_WARN, "进给倍率为0");
        }
        else
        {
            delAlarm(ALARM_SY, ALARM_WARNING, 3);
        }
    }
    // 2.快速进给倍率
    if (event_data->override_payload.override_type == 1)
    {
        LOG(LOG_LEVEL_INFO, "处理快速进给倍率事件 倍率值%d", event_data->override_payload.val);
        setVarChanDouble(0, VAR_CHAN_RAPID_OVERRIDE, (double)event_data->override_payload.val, 0);
        rt_data->nc_event = SYS_EVENT_RAPID_OVERRIDE;
        if (IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0))
        {
            setAlarm(ALARM_SY, ALARM_WARNING, 2);
            LOG(LOG_LEVEL_WARN, "快速倍率为0");
        }
        else
        {
            delAlarm(ALARM_SY, ALARM_WARNING, 2);
        }
    }
    // 3.主轴倍率
    if (event_data->override_payload.override_type == 2)
    {
        LOG(LOG_LEVEL_INFO, "处理主轴倍率事件 倍率值%d", event_data->override_payload.val);
        setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, (double)event_data->override_payload.val, 0);
        rt_data->nc_event = SYS_EVENT_SPDL_OVERRIDE;
    }
}

static void task_handle_auto_event(PMC_EVENT *event_data)
{
    // check：复位在F1.1置高，实时不响应启动和暂停事件
    if (nc_reset->is_resetting)
    {
        if (event_data->run_payload.run_state == 1)
            LOG(LOG_LEVEL_WARN, "复位中...循环启动无效");
        if (event_data->run_payload.run_state == 0)
            LOG(LOG_LEVEL_WARN, "复位中...进给保持无效");
        return;
    }
    if (event_data->run_payload.run_state == 1)
    {
        if (rt_data->current_mode == MODE_MEM || rt_data->current_mode == MODE_MDI)
        {
            // 循环启动
            cncServoSampleNotifyCycleStart();
            rt_data->nc_event = SYS_EVENT_RERUN;
            LOG(LOG_LEVEL_INFO, "处理循环启动事件");
        }
    }
    if (event_data->run_payload.run_state == 0)
    {
        if (rt_data->current_mode == MODE_MEM || rt_data->current_mode == MODE_MDI)
        {
            // 进给保持
            rt_data->nc_event = SYS_EVENT_PAUSE;
            LOG(LOG_LEVEL_INFO, "处理暂停事件");
        }
    }
}

static void task_handle_enable_event(PMC_EVENT *event_data)
{
    switch (event_data->enable_payload.enable_key)
    {
    case BLOCK_SKIP:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_BLOCK_SKIP_ON;
            LOG(LOG_LEVEL_INFO, "处理跳段开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_BLOCK_SKIP_OFF;
            LOG(LOG_LEVEL_INFO, "处理跳段关闭事件");
        }
        break;
    case DRY_RUN:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_DRY_RUN_ON;
            LOG(LOG_LEVEL_INFO, "处理空运行开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_DRY_RUN_OFF;
            LOG(LOG_LEVEL_INFO, "处理空运行关闭事件");
        }
        break;
    case MAC_LOCK:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_MAC_LOCK_ON;
            LOG(LOG_LEVEL_INFO, "处理机床锁住开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_MAC_LOCK_OFF;
            LOG(LOG_LEVEL_INFO, "处理机床锁住关闭事件");
        }
        break;
    case ALL_AXIS_LK:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_ALL_AXIS_LOCK_ON;
            LOG(LOG_LEVEL_INFO, "处理所有轴互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_ALL_AXIS_LOCK_OFF;
            LOG(LOG_LEVEL_INFO, "处理所有轴互锁关闭事件");
        }
        break;
    case MSBT_LOCK:
        rt_data->sys_state[SYS_IS_MSBT_LOCK] = event_data->enable_payload.enable_state;
        LOG(LOG_LEVEL_INFO, "处理辅助功能锁住事件");
        break;
    case SINGLE_BLOCK:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_SINGLE_BLOCK_ON;
            LOG(LOG_LEVEL_INFO, "处理单段开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_SINGLE_BLOCK_OFF;
            LOG(LOG_LEVEL_INFO, "处理单段关闭事件");
        }
        break;
    default:
        break;
    }
}

static void task_handle_reset_event(PMC_EVENT *event_data)
{
    if (event_data->reset_payload.reset_state == 1)
    {
        if (event_data->reset_payload.reset_type == 0)
        {
            LOG(LOG_LEVEL_INFO, "处理急停复位事件");
            rt_data->nc_event = SYS_EVENT_HMI_RESET;
        }
        if (event_data->reset_payload.reset_type == 1)
        {
            LOG(LOG_LEVEL_INFO, "处理m30复位事件");
            rt_data->nc_event = SYS_EVENT_OUTSIDE_RESET;
        }
    }
}

static void task_handle_hmi_event(PMC_EVENT *event_data)
{
    switch (event_data->hmi_payload.press_key)
    {
    case KEY_KP_0:
        rt_data->nc_event = SYS_EVENT_HMI_RESET;
        cncServoSampleNotifyReset();
        LOG(LOG_LEVEL_INFO, "处理HMI复位事件");
        break;
    case CMD_RESET_PIECE_COUNT:
        setVarChanInt(0, VAR_CHAN_PART_CNTR, 0, 0);
        LOG(LOG_LEVEL_INFO, "处理HMI重置加工计数");
        break;
    default:
        break;
    }
}
