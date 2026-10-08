#include "reset.h"
#include "rerun.h"
#include "rt_data.h"
#include "global.h"
#include <hmisysapi.h>

const char* reset_trigger_name[] = {"紧急停止信号ESP", "外部复位信号ERS", "复位&倒带信号RRW", "按下MDI的RESET键"};

GNC_RES reset_data_init(){
    nc_reset = (RESET_DATA*)calloc(sizeof(RESET_DATA), 1);
    if(nc_reset == NULL) return GNC_DATA_NULL;
    nc_reset->is_mdi_reset = 0;
    nc_reset->is_resetting = 0;
    nc_reset->is_resetted = 0;
    nc_reset->is_reset_delaying = 0;
    nc_reset->reset_tic = 0;
    nc_reset->mdi_reset_tic = 0;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES reset_data_reset(){
    if(nc_reset == NULL) return GNC_DATA_NULL;
    nc_reset->is_mdi_reset = 0;
    nc_reset->is_resetting = 0;
    nc_reset->is_resetted = 0;
    nc_reset->is_reset_delaying = 0;
    nc_reset->reset_tic = 0;
    nc_reset->mdi_reset_tic = 0;
    return GNC_OPERATOR_SUCCESS;
}

/// @brief 触发F1.1的信号来源
/// @param reset_trigger 
void nc_set_resetting(RESET_TRIGGER reset_trigger){
    nc_reset->reset_trigger = reset_trigger;
    nc_reset->is_resetting = 1;
    LOG(LOG_LEVEL_DEBUG, "复位中F1.1信号来源：%s", reset_trigger_name[reset_trigger]);
    switch (nc_reset->reset_trigger)
    {
    // 基于MDI的复位确认信号
    case HMI_RESET:
        nc_reset->is_mdi_reset = 1;
        nc_reset->mdi_reset_tic = 0;
        break;
    default:
        break;
    }
}

/// @brief 完成F1.1的判断
void nc_set_resetted(){
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE]) return;
    switch (rt_data->current_mode) {
        case MODE_MDI: // MDI
        case MODE_MEM: // 记忆
        case MODE_DNC:
            if(nc_reset->is_resetting && cycle_tp->exec_state == EXEC_IDLE && IsEqualTo(spindle_driver.actual_vel, 0)){
                nc_reset->is_resetted = 1;
                rt_data_reset();
                rerun_data_reset();
            }
            break;
        case MODE_HOMING: // 回零
            if(nc_reset->is_resetting && !var_chan[0][VAR_CHAN_IS_HOMING].value.i && IsEqualTo(spindle_driver.actual_vel, 0)){
                nc_reset->is_resetted = 1;
                rt_data_reset();
                rerun_data_reset();
            }
            break;
        case MODE_JOG: // 点动
        case MODE_MPG: // 手轮
        case MODE_IDLE: // 空闲状态
        case MODE_EDIT: // 编辑
        default: // error
            if(nc_reset->is_resetting && IsEqualTo(spindle_driver.actual_vel, 0)){
                nc_reset->is_resetted = 1;
                rt_data_reset();
                rerun_data_reset();
            }
            break;
    }
}
