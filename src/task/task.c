#include "task.h"
#include "eventdef.h"
#include "global.h"
#include "hmi_scanner.h"
#include "regapi.h"
#include "servosampleevents.h"
#include "spl_control.h"
#include "spl_orient.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <stdlib.h>
#include "reset.h"

static PLC_EVENT event_data;

/* 内部函数声明 */
static GNC_RES task_handle_plc_event(PLC_EVENT *event_data);
static void task_handle_mode_event(PLC_EVENT *event_data);
static void task_handle_feed_event(PLC_EVENT *event_data);
static void task_handle_spl_event(PLC_EVENT *event_data);
static void task_handle_overrate_event(PLC_EVENT *event_data);
static void task_handle_auto_event(PLC_EVENT *event_data);
static void task_handle_enable_event(PLC_EVENT *event_data);
static void task_handle_reset_event(PLC_EVENT *event_data);
static void task_handle_hmi_event(PLC_EVENT *event_data);
static void task_handle_external_decel_event(PLC_EVENT *event_data);
static bool task_external_decel_enabled(int function);
static void task_handle_spd_ori_event(PLC_EVENT *event_data);

/* 外部函数定义 */

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
    task_handle_plc_event(&event_data);
    return GNC_OPERATOR_SUCCESS;
}

/* 内部函数定义 */

static GNC_RES task_handle_plc_event(PLC_EVENT *event_data)
{
    if (!event_data || !rt_data)
        return GNC_DATA_NULL;

    switch (event_data->id)
    {
    case PLC_EV_MSG_NONE:
    {
        rt_data->nc_event = SYS_EVENT_NONE;
        break;
    }
    case PLC_EV_MSG_MODE:
    {
        task_handle_mode_event(event_data);
        break;
    }
    case PLC_EV_MSG_FEEDAXIS_KEY:
    {
        task_handle_feed_event(event_data);
        break;
    }
    case PLC_EV_MSG_SPINDLE:
    {
        task_handle_spl_event(event_data);
        break;
    }
    case PLC_EV_MSG_OVERRIDE:
    {
        task_handle_overrate_event(event_data);
        break;
    }
    case PLC_EV_MSG_AUTO_RUN:
    {
        task_handle_auto_event(event_data);
        break;
    }
    case PLC_EV_MSG_ENANLE:
    {
        task_handle_enable_event(event_data);
        break;
    }
    case PLC_EV_MSG_HMI:
    {
        task_handle_hmi_event(event_data);
        break;
    }
    case PLC_EV_MSG_RESET:
    {
        task_handle_reset_event(event_data);
        break;
    }
    case PLC_EV_MSG_SPD_ORI:
    {
        task_handle_spd_ori_event(event_data);
        // spl_orient_request();
        break;
    }
    case PLC_EV_MSG_EXTERNAL_DECEL:
    {
        if (task_external_decel_enabled(event_data->external_decel_payload.function))
        {
            LOG(LOG_LEVEL_INFO,
                "处理PLC外部减速事件: 功能号=%d, 轴号=%d, 方向=%s(%d), 状态=%s(%d)",
                event_data->external_decel_payload.function,
                event_data->external_decel_payload.axis_num,
                event_data->external_decel_payload.direction > 0 ? "正向" :
                    (event_data->external_decel_payload.direction < 0 ? "负向" : "未知"),
                event_data->external_decel_payload.direction,
                event_data->external_decel_payload.active != 0 ? "生效" : "解除",
                event_data->external_decel_payload.active);
        }
        task_handle_external_decel_event(event_data);
        break;
    }
    default:
        break;
    }
    return GNC_OPERATOR_SUCCESS;
}

static void task_handle_mode_event(PLC_EVENT *event_data)
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

static void task_handle_feed_event(PLC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO, "处理进给轴按键事件 轴选键%d 动作%d 当前模式%d", event_data->feedaxis_key_payload.axis_num, event_data->feedaxis_key_payload.direction, rt_data->current_mode);
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

static void task_handle_spl_event(PLC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO, "处理主轴事件 动作%d", event_data->spindle_payload.direction);
    if (event_data->spindle_payload.direction == 1)
    {
        LOG(LOG_LEVEL_INFO, "处理主轴正转事件");
        if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
            spl_context.orient_request_pending ||
            spl_context.orient_release_pending)
        {
            spl_context.deferred_start_pending = true;
            spl_context.deferred_start_state = SPINDLE_CW;
            spl_context.deferred_start_speed = spl_context.mcp_target_speed;
            return;
        }
        spl_context.spl_event = SPINDLE_EVENT_CW;
        spl_context.command_speed = spl_context.mcp_target_speed;
        // spl_context.mcp_target_speed = cycle_tp->plan_block->mst_data.spindle.speed;
        // spl_context.mcode_feedback = false;
        // spl_context.m19_active = false;
        // spl_context.orient_hold_stop = false;
        spl_context.command_state = SPINDLE_CW;
        spl_context.start_request_pending = true;
    }
    if (event_data->spindle_payload.direction == -1)
    {
        LOG(LOG_LEVEL_INFO, "处理主轴反转事件");
        if (spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
            spl_context.orient_request_pending ||
            spl_context.orient_release_pending)
        {
            spl_context.deferred_start_pending = true;
            spl_context.deferred_start_state = SPINDLE_CCW;
            spl_context.deferred_start_speed = spl_context.mcp_target_speed;
            return;
        }
        spl_context.spl_event = SPINDLE_EVENT_CCW;
        spl_context.command_speed = spl_context.mcp_target_speed;
        // spl_context.mcp_target_speed = cycle_tp->plan_block->mst_data.spindle.speed;
        // spl_context.mcode_feedback = false;
        // spl_context.m19_active = false;
        // spl_context.orient_hold_stop = false;
        spl_context.command_state = SPINDLE_CCW;
        spl_context.start_request_pending = true;
    }
    if (event_data->spindle_payload.direction == 0)
    {
        LOG(LOG_LEVEL_INFO, "处理主轴停止事件");

        /*
         * M19 通常会同时改变 G29.6 和 G70.6。定向上升沿可能先到达，
         * 随后的 G29.6 下降沿只是定向前的停转请求，不能覆盖尚未处理
         * 的 SPINDLE_EVENT_ORIENT；定向流程本身已经会停止主轴。
         */
        if (spl_context.spl_event == SPINDLE_EVENT_ORIENT ||
            spl_context.orient_phase != SPINDLE_ORIENT_IDLE ||
            spl_context.orient_request_pending)
        {
            return;
        }

        spl_context.spl_event = SPINDLE_EVENT_STOP;
        spl_context.command_speed = 0.0;
        // spl_context.mcode_feedback = false;
        // spl_context.m19_active = false;
        // spl_context.orient_hold_stop = false;
        spl_context.command_state = SPINDLE_STOP;
        spl_context.start_request_pending = false;
    }
}

static void task_handle_spd_ori_event(PLC_EVENT *event_data)
{
    LOG(LOG_LEVEL_INFO, "处理主轴定向事件 动作%d", event_data->spd_ori_payload.p1);
    // 推送主轴定向事件
    if (event_data->spd_ori_payload.p1 == 1)
    {
        spl_context.spl_event = SPINDLE_EVENT_ORIENT;
    }
    // 推送主轴定向释放事件
    if (event_data->spd_ori_payload.p1 == -1)
    {
        spl_context.spl_event = SPINDLE_EVENT_RELEASE_ORIENT;
    }
}

static void task_handle_overrate_event(PLC_EVENT *event_data)
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

static void task_handle_auto_event(PLC_EVENT *event_data)
{
    // check：复位在F1.1置高，实时不响应启动事件
    if (nc_reset->is_resetting && event_data->run_payload.run_state == 1)
    {
        LOG(LOG_LEVEL_WARN, "复位中，循环启动无效");
        return;
    }
    // 如果程序再启动时，实时不响应启动事件
    if(rt_data->sys_state[SYS_IS_PROG_RESTAR] && event_data->run_payload.run_state == 1) {
        LOG(LOG_LEVEL_INFO, "程序再启动中，循环启动无效");
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

static void task_handle_enable_event(PLC_EVENT *event_data)
{
    switch (event_data->enable_payload.enable_key)
    {
    case PROG_RESTAR:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PROGRAM_RESTART_ON;
            LOG(LOG_LEVEL_INFO, "处理程序再启动开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PROGRAM_RESTART_OFF;
            LOG(LOG_LEVEL_INFO, "处理程序再启动关闭事件");
        }
        break;
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
    case IT:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT_ON;
            LOG(LOG_LEVEL_INFO, "处理所有轴互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT_OFF;
            LOG(LOG_LEVEL_INFO, "处理所有轴互锁关闭事件");
        }
        break;
    case STLK:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_STLK_ON;
            LOG(LOG_LEVEL_INFO, "处理起动锁停开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_STLK_OFF;
            LOG(LOG_LEVEL_INFO, "处理起动锁停关闭事件");
        }
        break;
    case BSL:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_BSL_ON;
            LOG(LOG_LEVEL_INFO, "处理程序段开始互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_BSL_OFF;
            LOG(LOG_LEVEL_INFO, "处理程序段开始互锁关闭事件");
        }
        break;
    case CSL:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_CSL_ON;
            LOG(LOG_LEVEL_INFO, "处理切削程序段开始互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_CSL_OFF;
            LOG(LOG_LEVEL_INFO, "处理切削程序段开始互锁关闭事件");
        }
        break;
    case IT1:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT1_ON;
            LOG(LOG_LEVEL_INFO, "处理轴1互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT1_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴1互锁关闭事件");
        }
        break;
    case IT2:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT2_ON;
            LOG(LOG_LEVEL_INFO, "处理轴2互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT2_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴2互锁关闭事件");
        }
        break;
    case IT3:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT3_ON;
            LOG(LOG_LEVEL_INFO, "处理轴3互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT3_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴3互锁关闭事件");
        }
        break;
    case IT4:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT4_ON;
            LOG(LOG_LEVEL_INFO, "处理轴4互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT4_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴4互锁关闭事件");
        }
        break;
    case IT5:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_IT5_ON;
            LOG(LOG_LEVEL_INFO, "处理轴5互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_IT5_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴5互锁关闭事件");
        }
        break;
    case PMIT1:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PMIT1_ON;
            LOG(LOG_LEVEL_INFO, "处理轴1正向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PMIT1_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴1正向互锁关闭事件");
        }
        break;
    case PMIT2:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PMIT2_ON;
            LOG(LOG_LEVEL_INFO, "处理轴2正向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PMIT2_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴2正向互锁关闭事件");
        }
        break;
    case PMIT3:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PMIT3_ON;
            LOG(LOG_LEVEL_INFO, "处理轴3正向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PMIT1_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴3正向互锁关闭事件");
        }
        break;
    case PMIT4:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PMIT4_ON;
            LOG(LOG_LEVEL_INFO, "处理轴4正向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PMIT4_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴4正向互锁关闭事件");
        }
        break;
    case PMIT5:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_PMIT5_ON;
            LOG(LOG_LEVEL_INFO, "处理轴5正向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_PMIT5_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴5正向互锁关闭事件");
        }
        break;
    case NMIT1:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_NMIT1_ON;
            LOG(LOG_LEVEL_INFO, "处理轴1负向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_NMIT1_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴1负向互锁关闭事件");
        }
        break;
    case NMIT2:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_NMIT2_ON;
            LOG(LOG_LEVEL_INFO, "处理轴2负向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_NMIT2_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴2负向互锁关闭事件");
        }
        break;
    case NMIT3:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_NMIT3_ON;
            LOG(LOG_LEVEL_INFO, "处理轴3负向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_NMIT3_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴3负向互锁关闭事件");
        }
        break;
    case NMIT4:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_NMIT4_ON;
            LOG(LOG_LEVEL_INFO, "处理轴4负向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_NMIT4_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴4负向互锁关闭事件");
        }
        break;
    case NMIT5:
        if (event_data->enable_payload.enable_state)
        {
            rt_data->nc_event = SYS_EVENT_NMIT5_ON;
            LOG(LOG_LEVEL_INFO, "处理轴5负向互锁开启事件");
        }
        else
        {
            rt_data->nc_event = SYS_EVENT_NMIT5_OFF;
            LOG(LOG_LEVEL_INFO, "处理轴5负向互锁关闭事件");
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

static void task_handle_reset_event(PLC_EVENT *event_data)
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

static void task_handle_hmi_event(PLC_EVENT *event_data)
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

static void task_handle_external_decel_event(PLC_EVENT *event_data)
{
    int axis;
    int function;
    int direction;
    int direction_index;
    bool active;
    AXIS_STATUS *status;

    if (event_data == NULL || rt_data == NULL)
        return;

    axis = event_data->external_decel_payload.axis_num;
    function = event_data->external_decel_payload.function;
    direction = event_data->external_decel_payload.direction;
    active = (event_data->external_decel_payload.active != 0);

    if (axis < 0 || axis >= SYS_AXES_NUM)
        return;
    if (function < 1 || function > SYS_EXTERNAL_DECEL_FUNCTION_COUNT)
        return;
    if (direction != 1 && direction != -1)
        return;

    direction_index = (direction > 0) ? 0 : 1;
    status = &rt_data->axis_data[axis].status;
    status->external_decel_active[function - 1][direction_index] = active;

    if (direction > 0)
    {
        rt_data->nc_event = active ? SYS_EVENT_EXTERNAL_DECEL_POS_ON : SYS_EVENT_EXTERNAL_DECEL_POS_OFF;
    }
    else
    {
        rt_data->nc_event = active ? SYS_EVENT_EXTERNAL_DECEL_NEG_ON : SYS_EVENT_EXTERNAL_DECEL_NEG_OFF;
    }
}

static bool task_external_decel_enabled(int function)
{
    if (rt_data == NULL || !rt_data->chan_data[0].ex_dec_enable)
        return false;

    /* 功能 1 使用总开关，功能 2~5 还需检查各自的独立开关。 */
    switch (function)
    {
    case PLC_EXTERNAL_DECEL_FUNCTION_1:
        return true;
    case PLC_EXTERNAL_DECEL_FUNCTION_2:
        return rt_data->chan_data[0].ex_dec2_enable;
    case PLC_EXTERNAL_DECEL_FUNCTION_3:
        return rt_data->chan_data[0].ex_dec3_enable;
    case PLC_EXTERNAL_DECEL_FUNCTION_4:
        return rt_data->chan_data[0].ex_dec4_enable;
    case PLC_EXTERNAL_DECEL_FUNCTION_5:
        return rt_data->chan_data[0].ex_dec5_enable;
    default:
        return false;
    }
}
