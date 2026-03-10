#include "alarm.h"
#include "logapi.h"
#include "log.h"
#include "rt_data.h"
#include <alarmapi.h>

// 报警状态逻辑
void handle_alarm_cmd(){
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;
        case SYS_EVENT_RESET:
            // 清除报警
            cleanAllAlarm(ALARM_TYPE_ALL, ALARM_LEVEL_ALL);
            // 状态复位、数据重置
            tp_reset();
            // 切换到IDLE模式
            rt_data_reset();
            // 译码器复位
            rt_data->sys_state[SYS_IS_ESTOP_ON] = false;
            event_queue_reset(event_queue);
            LOG_OUTPUT(INFO, "报警已清除");
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
            // log_system("[alarm] 报警下不支持操作，请按复位清除报警");
            LOG_OUTPUT(ERROR, "报警下不支持操作，请按复位清除报警");
            break;
    }
    rt_data->nc_event = SYS_EVENT_NONE;
}
