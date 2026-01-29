#include "alarm.h"

// 报警状态逻辑
void handle_alarm_cmd(){
    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // 如果急停未重置
            if(var_chan[0].is_estop){
                // LOG_OUTPUT(INFO, "please reset estop at first.");
                break;
            }
            // 状态复位、数据重置
            tp_reset();
            // 切换到IDLE模式
            sys_reset();
            // 译码器复位
            var_chan[0].is_estop = false;
            // LOG_OUTPUT(INFO, "change to reset mode");
            break;
        
        case SYS_EVENT_RERUN:
        case SYS_EVENT_PAUSE:
        case SYS_EVENT_X_NEG_PRESS:
        case SYS_EVENT_Y_NEG_PRESS:
        case SYS_EVENT_Z_NEG_PRESS:
        case SYS_EVENT_X_POS_PRESS:
        case SYS_EVENT_Y_POS_PRESS:
        case SYS_EVENT_Z_POS_PRESS:
        case SYS_EVENT_Z_POS_RELEASE:
            // 报警状态不支持其他操作
            // LOG_OUTPUT(INFO, "alarm mode is not support this operation.");
            break;

        default:
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}
