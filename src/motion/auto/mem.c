#include "mem.h"
#include "mem_mst.h"
#include "mem_gcode.h"
#include "global.h"
#include <hmisysapi.h> 
#include <alarmapi.h>

static void auto_set_state();
static void auto_handle_alarm();
static bool auto_check_running();
static void auto_check_prog_end();
static inline bool is_tap_mode(pmc_mode mode);
static inline bool is_pause_mode(pmc_mode mode);
static bool reset_trigger = false;

void handle_auto_cmd(){
    // 设置自动加工状态
    auto_set_state();
    // 执行M代码
    auto_execute_mcode();
    // 执行G代码
    auto_execute_gcode();
    // 检查程序是否结束
    auto_check_prog_end();
}

static void auto_handle_alarm(){
    mcode_data->mcode_state = MCODE_EXECUTE_RESET;
    if(cycle_tp->mem_exec_state == EXEC_RUNNING){
        cycle_tp->mem_exec_state = EXEC_PAUSING;
        cycle_tp->is_event_trigger = true;
        cycle_tp->is_resetting = 1;
    }else if(cycle_tp->mem_exec_state == EXEC_PAUSING){
        cycle_tp->is_resetting = 1;
    }
    // 通知译码复位
    rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
}

static bool auto_check_running(){
    if(rt_get_chan_state(0, CHAN_IS_PARSERING)) return true;
    if(!queue_is_empty(plan_queue) || !queue_is_empty(pretreat_queue) || !gcode_queue_is_empty(gcode_queue) || mcode_data->mcode_size != 0) return true;
    return false;
}

static void auto_check_prog_end(){
    if(auto_check_running()) return;
    if(cycle_tp->mem_exec_state == EXEC_RUNNING){
        int error_no = rt_get_parser_error_no();
        int error_line = rt_get_parser_error_line();
        if(error_no >= 0){
            setVarChanInt(0, VAR_CHAN_RUN_ROW, error_line, 0);
            int res = setAlarm(ALARM_PS, ALARM_ERROR, error_no);
            if(res == -8) return;
            LOG(LOG_LEVEL_INFO, "error line: %d", error_line);
            LOG(LOG_LEVEL_ERR, "译码错误 %d", error_no);
        }else{
            gcode_handle_reset();
            mcode_handle_reset();
            LOG(LOG_LEVEL_DEBUG, "程序结束");
        } 
    }
}

static void auto_set_state(){
    // 自动模式下，报警或切换模式，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        if(!reset_trigger){
            reset_trigger = true;
            auto_handle_alarm();
        }
        return;
    }else reset_trigger = false;
    switch(rt_data->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            mcode_data->mcode_state = MCODE_EXECUTE_RESET;
            LOG(LOG_LEVEL_INFO, "自动模式处理复位");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                cycle_tp->mem_exec_state = EXEC_PAUSING;
                cycle_tp->is_event_trigger = true; 
                cycle_tp->is_resetting = 1;
            }else if(cycle_tp->mem_exec_state == EXEC_PAUSING){
                cycle_tp->is_resetting = 1;
            }
            // 通知译码复位
            rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
            break;
        
        case SYS_EVENT_RERUN:
            LOG(LOG_LEVEL_INFO, "自动模式处理自动运行");
            if(IsEqualTo(var_chan[0][VAR_CHAN_FEED_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 3);
                LOG(LOG_LEVEL_WARN, "进给倍率为0");
            }else{
                delAlarm(ALARM_SY, ALARM_WARNING, 3);
            }
            if(IsEqualTo(var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f, 0)){
                setAlarm(ALARM_SY, ALARM_WARNING, 2);
                LOG(LOG_LEVEL_WARN, "快速倍率为0");
            }else{
                delAlarm(ALARM_SY, ALARM_WARNING, 2);
            }
            if(cycle_tp->mem_exec_state == EXEC_IDLE){
                cycle_tp->mem_exec_state = EXEC_RUNNING;
                cycle_tp->is_event_trigger = true;
                setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
                setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
                // 重置
                queue_reset(plan_queue);
                queue_reset(pretreat_queue);
                gcode_queue_reset(gcode_queue);
                mcode_handle_reset();
                // 通知译码运行，初始化译码器
                rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, true);
                // 译码正在运行，译码结束译码器将其置false
                rt_set_chan_state(0, CHAN_IS_PARSERING, true);
                setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                LOG(LOG_LEVEL_DEBUG, "开始加工，速度规划模式 %d", par_chan[0][PAR_CH_VPLAN_MODE].value.val_int);
            }
            else if(cycle_tp->mem_exec_state == EXEC_PAUSING && !cycle_tp->is_resetting){
                cycle_tp->mem_exec_state = EXEC_RUNNING;
                cycle_tp->is_event_trigger = true;
                setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
                setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
            }
            mcode_data->mcode_state = MCODE_EXECUTE_RUNNING;
            break;
        
        case SYS_EVENT_PAUSE:
            LOG(LOG_LEVEL_INFO, "自动模式处理暂停");
            mcode_data->mcode_state = MCODE_EXECUTE_PAUSING;
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                // 攻丝方式下进给暂停无效
                if(!queue_is_empty(plan_queue)){
                    if(is_tap_mode(cycle_tp->plan_block->mode)) {
                        // ｍ代码状态也不能进入暂停
                        mcode_data->mcode_state = MCODE_EXECUTE_RUNNING;
                        break;
                    }
                }
                cycle_tp->mem_exec_state = EXEC_PAUSING;
                cycle_tp->is_event_trigger = true;
            }
            break;

        case SYS_EVENT_FEED_OVERRIDE:
            LOG(LOG_LEVEL_INFO, "自动模式处理进给倍率切换");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                if(!queue_is_empty(plan_queue)){
                    if(is_pause_mode(cycle_tp->plan_block->mode)) break;
                    if(!is_tap_mode(cycle_tp->plan_block->mode) && cycle_tp->plan_block->type != RAPID){
                        cycle_tp->is_event_trigger = true;
                    }
                }
            }
            break;

        case SYS_EVENT_RAPID_OVERRIDE:
            LOG(LOG_LEVEL_INFO, "自动模式处理快速倍率切换 %f", var_chan[0][VAR_CHAN_RAPID_OVERRIDE].value.f);
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                if(!queue_is_empty(plan_queue)){
                    if(is_pause_mode(cycle_tp->plan_block->mode)) break;
                    if(!is_tap_mode(cycle_tp->plan_block->mode) && cycle_tp->plan_block->type == RAPID){
                        cycle_tp->is_event_trigger = true;
                    };
                }
            }
            break;

        case SYS_EVENT_DRY_RUN_ON:
            LOG(LOG_LEVEL_INFO, "自动模式处理开启空运行");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                if(!queue_is_empty(plan_queue)){
                    if(is_pause_mode(cycle_tp->plan_block->mode)) break;
                    cycle_tp->is_event_trigger = true;
                }
            }
            break;
        
        case SYS_EVENT_DRY_RUN_OFF:
            LOG(LOG_LEVEL_INFO, "自动模式处理关闭空运行");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                if(!queue_is_empty(plan_queue)){
                    if(is_pause_mode(cycle_tp->plan_block->mode)) break;
                    cycle_tp->is_event_trigger = true;
                }
            }
            break;
        
        case SYS_EVENT_SINGLE_BLOCK_ON:
        case SYS_EVENT_SINGLE_BLOCK_OFF:
            LOG(LOG_LEVEL_INFO, "自动模式处理单段");
            if(cycle_tp->mem_exec_state == EXEC_RUNNING){
                if(!queue_is_empty(plan_queue)){
                    if(is_pause_mode(cycle_tp->plan_block->mode)) break;
                    cycle_tp->is_event_trigger = true;
                }
            }
            break;

        case SYS_EVENT_BLOCK_SKIP_ON:
            if(cycle_tp->mem_exec_state == EXEC_IDLE) {
                rt_set_sys_state(SYS_IS_BLOCK_SKIP, true);
                LOG(LOG_LEVEL_INFO, "自动模式开启跳段");
                break;
            }
            LOG(LOG_LEVEL_WARN, "自动模式下不允许加工中开启跳段");
            break;

        case SYS_EVENT_BLOCK_SKIP_OFF:
            if(cycle_tp->mem_exec_state == EXEC_IDLE) {
                rt_set_sys_state(SYS_IS_BLOCK_SKIP, false);
                LOG(LOG_LEVEL_INFO, "自动模式关闭机跳段");
                break;
            }
            LOG(LOG_LEVEL_WARN, "自动模式下不允许加工中关闭跳段");
            break;

        case SYS_EVENT_MAC_LOCK_ON:
            if(cycle_tp->mem_exec_state == EXEC_IDLE) {
                rt_data->sys_state[SYS_IS_MAC_LOCK] = true;
                LOG(LOG_LEVEL_INFO, "自动模式开启机械锁住");
                break;
            }
            LOG(LOG_LEVEL_WARN, "自动模式下不允许加工中开启机械锁住");
            break;

        case SYS_EVENT_MAC_LOCK_OFF:
            if(cycle_tp->mem_exec_state == EXEC_IDLE) {
                rt_data->sys_state[SYS_IS_MAC_LOCK] = false;
                LOG(LOG_LEVEL_INFO, "自动模式关闭机械锁住");
                break;
            }
            LOG(LOG_LEVEL_WARN, "自动模式下不允许加工中关闭机械锁住");
            break;

        case SYS_EVENT_X_POS_PRESS:
        case SYS_EVENT_Y_POS_PRESS:
        case SYS_EVENT_Z_POS_PRESS:
        case SYS_EVENT_X_NEG_PRESS:
        case SYS_EVENT_Y_NEG_PRESS:
        case SYS_EVENT_Z_NEG_PRESS:
            LOG(LOG_LEVEL_WARN, "自动模式不支持该操作");
            break;
        
        default:
            break;
    }
}

void auto_handle_estop(){
    mcode_handle_estop();
    gcode_handle_estop();
}

static inline bool is_tap_mode(pmc_mode mode){
    return cycle_tp->plan_block->mode == PMC_MODE_TAP || cycle_tp->plan_block->mode == PMC_MODE_TAP_RETURE;
}

static inline bool is_pause_mode(pmc_mode mode){
    return cycle_tp->plan_block->mode == PMC_MODE_PAUSE;
}

// 编辑模式下复位，光标回到首行
void handle_edit_cmd(){
    switch(rt_data->nc_event){
        case SYS_EVENT_RESET:
            setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
            break;
        default:
            break;
    }
}