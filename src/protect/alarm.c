#include "alarm.h"
#include "master.h"
#include "rt_data.h"
#include "reset.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include "handwheel.h"

// 急停处理逻辑
void handle_estop_cmd()
{
    // 按下急停时执行一次
    if (!rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
    {
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = true;
        // 系统事件置为NONE
        rt_data->nc_event = SYS_EVENT_NONE;
        // 人机交互状态栏显示急停报警
        setVarChanInt(0, VAR_CHAN_IS_ESTOP, 1, 0);
        // 1.急停信号ESP
        nc_set_resetting(ESTOP_RESET);
        // 清除回参考点信号
        reset_ref_state();
        // 重置各个模式
        auto_handle_estop();
        ref_handle_estop();
        jog_handle_estop();
        handwheel_handle_estop();
        LOG(LOG_LEVEL_INFO, "处理按下急停");
    }
    // 急停下不生效的事件
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_RERUN:
        rt_data->nc_event = SYS_EVENT_NONE;
        LOG(LOG_LEVEL_ERR, "急停下操作 %d 按键无效", rt_data->nc_event);
        break;
    default:
        break;
    }
}

// 报警状态逻辑
void handle_alarm_cmd()
{
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_NONE:
        break;
    case SYS_EVENT_HMI_RESET:
        // 系统急停输入信号为真时，不能响应复位事件
        if (rt_data->sys_state[SYS_ESTOP_INPUT])
            break;
        // 清除报警，不清除PLC和用户PLC报警
        cleanAllAlarm(ALARM_SY, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_CH, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_AX, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_SV, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_DV, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_PS, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_HM, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_ETHCAT_SV, ALARM_LEVEL_ALL);
        rt_data->sys_state[SYS_IS_ALARMING] = false;
        // 清除内部急停标志和人机急停显示标志
        rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] = false;
        setVarChanInt(0, VAR_CHAN_IS_ESTOP, 0, 0);
        // 状态复位、数据重置
        tp_reset();
        rt_data_reset();
        // 重置事件
        event_queue_reset(event_queue);
        // 更新cmd_pos位置
        update_pos_flag = 0;
        // 清除驱动器报警
#ifndef DEBUG_MODE_ENABLED
        Master_Reset();
#endif
        // 复位生效参数
        Axis_Limit_Manager_Init();
        // 复位清除跟随误差报警标志
        reset_ferror_control();
        // 清除译码器状态
        rt_set_parser_state(-1, 0);
        LOG(LOG_LEVEL_INFO, "报警已清除");
        break;
    case SYS_EVENT_RERUN:
    case SYS_EVENT_PAUSE:
    case SYS_EVENT_X_POS_PRESS:
    case SYS_EVENT_X_POS_RELEASE:
    case SYS_EVENT_X_NEG_PRESS:
    case SYS_EVENT_X_NEG_RELEASE:
    case SYS_EVENT_Y_POS_PRESS:
    case SYS_EVENT_Y_POS_RELEASE:
    case SYS_EVENT_Y_NEG_PRESS:
    case SYS_EVENT_Y_NEG_RELEASE:
    case SYS_EVENT_Z_POS_PRESS:
    case SYS_EVENT_Z_POS_RELEASE:
    case SYS_EVENT_Z_NEG_PRESS:
    case SYS_EVENT_Z_NEG_RELEASE:
    default:
        // 报警时由pmc控制面板按键是否可执行，nc侧不约束
        LOG(LOG_LEVEL_ERR, "报警下操作 %d 按键", rt_data->nc_event);
        break;
    }
}
