#include "alarm.h"
#include <hmisysapi.h>
#include "rt_data.h"
#include <alarmapi.h>

// 急停处理逻辑
void handle_estop_cmd(){
    // 复位状态：1.急停信号ESP
    setVarChanInt(0, VAR_CHAN_IS_RESETTING, 1, 0);
    // 重置各个模式
    mem_handle_estop();
    homing_handle_estop();
    jog_handle_estop();
    if(rt_data->nc_event != SYS_EVENT_NONE || rt_data->target_mode != rt_data->current_mode){
        //　不允许操作
        rt_data->nc_event = SYS_EVENT_NONE;
        // 不允许切换模式
        rt_data->target_mode = rt_data->current_mode;
        LOG(LOG_LEVEL_ERR, "急停报警下不允许操作，请检查问题！");
    }
}

// 报警状态逻辑
void handle_alarm_cmd(){
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;
        case SYS_EVENT_RESET:
            // LOG(LOG_LEVEL_INFO, "报警处理复位");        
            // 清除报警
            cleanAllAlarm(ALARM_TYPE_ALL, ALARM_LEVEL_ALL);
            rt_data->sys_state[SYS_IS_ALARMING] = false;
            // 状态复位、数据重置
            tp_reset();
            // 重置事件
            event_queue_reset(event_queue);
            // 更新cmd_pos位置
            update_pos_flag = 1;
            // 清除驱动器报警
            #ifndef DEBUG_MODE_ENABLED
            Master_Reset();
            #endif
            // 复位生效参数
            Axis_Limit_Manager_Init();
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
            LOG(LOG_LEVEL_ERR, "报警下不支持操作，请按复位清除报警");
            break;
    }
    rt_data->nc_event = SYS_EVENT_NONE;
}
