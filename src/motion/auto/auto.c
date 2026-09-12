#include "auto.h"
#include "auto_mst.h"
#include "auto_gcode.h"
#include "global.h"
#include "rt_data.h"
#include "rerun.h"
#include <hmisysapi.h> 
#include <alarmapi.h>
#include "stored_stroke_check.h"
#include "trace_publisher.h"
#include "ncitpmacrodef.h"
#include "macrovarapi.h"

static void auto_check_error_line();
static void auto_update_state();
static void auto_check_event();
static void auto_update_stage();
static void auto_update_cmd();
static void auto_execute_cmd();
static void auto_start_run();
static void auto_reset_run();
static void auto_pause_run();
static void auto_restart_run();
static void auto_update_cur_cmd();
static void auto_get_next_cmd();
static void auto_handle_alarm();
static void auto_update_var();
static bool auto_is_program_end();
static bool auto_check_program_end();
static void auto_pop_current_block();

void handle_auto_cmd(){
    // 更新状态
    auto_update_state();
    // 处理中断事件
    auto_check_event();
    // 检查运行阶段
    auto_update_cmd();
    // 执行指令
    auto_execute_cmd();
    trace_publisher_publish_current(0);
    // 更新阶段
    auto_update_stage();
    // 检查程序结束
    auto_check_program_end();
}

static void auto_update_state(){
    // 报警或切换模式下，减速停止
    static bool reset_trigger = false;
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        if(!reset_trigger){
            reset_trigger = true;
            auto_handle_alarm();
        }
        return;
    }else reset_trigger = false;
    // 正常模式下
    switch (cycle_tp->exec_state)
    {
    case EXEC_IDLE:
        switch (rt_data->nc_event)
        {
        case SYS_EVENT_RERUN:
            LOG(LOG_LEVEL_INFO, "自动模式处理循环启动");
            // 判断是否从首行开始
            if(check_is_program_rerun()) break;
            auto_start_run();
            cycle_tp->exec_state = EXEC_RUNNING;
            cycle_tp->running_stage = PROGRAM_START;
            break;
        default:
            break;
        }
        break;

    case EXEC_RUNNING:
        switch (rt_data->nc_event)
        {
        case SYS_EVENT_PAUSE:
            LOG(LOG_LEVEL_INFO, "自动模式处理进给保持");
            switch(cycle_tp->exec_cmd){
            case DO_MOVE_CMD:
            case DO_CHOP_CMD:
            case DO_DWELL_CMD:
            case DO_HOME_CMD:
            case DO_SKIP_CMD:
            case DO_MSTB_CMD:
            case DO_NULL_CMD:
                cycle_tp->exec_state = EXEC_PAUSING;
                switch (cycle_tp->running_stage)
                {
                case PROGRAM_RUNNING:
                case BLOCK_PAUSED:
                case BLOCK_BREAKPOINT:
                    cycle_tp->running_stage = BLOCK_BREAKPOINT;
                    break;
                default:
                    break; 
                }
                break; 
            case DO_TAP_CMD: // 攻丝时不能暂停
            default:
                break;
            }
            break;
        case SYS_EVENT_HMI_RESET:
            LOG(LOG_LEVEL_INFO, "自动模式处理MDI复位");
            cycle_tp->exec_state = EXEC_RESET;
            switch (cycle_tp->running_stage)
            {
            case PROGRAM_RUNNING:
            case BLOCK_PAUSED:
            case BLOCK_BREAKPOINT:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            default:
                break; 
            }
            break;
        case SYS_EVENT_OUTSIDE_RESET:
            LOG(LOG_LEVEL_INFO, "自动模式处理外部复位");
            cycle_tp->exec_state = EXEC_RESET;
            break;
        default:
            break;
        }
        break;

    case EXEC_PAUSING:
        switch (rt_data->nc_event)
        {
        case SYS_EVENT_HMI_RESET:
            LOG(LOG_LEVEL_INFO, "自动模式处理复位");
            switch(cycle_tp->exec_cmd){
            case DO_MOVE_CMD:
            case DO_CHOP_CMD:
            case DO_HOME_CMD:
            case DO_TAP_CMD:
            case DO_SKIP_CMD:
                cycle_tp->exec_state = EXEC_RESET;
                break;
            case DO_DWELL_CMD:
            case DO_MSTB_CMD:
            case DO_NULL_CMD:
                cycle_tp->exec_state = EXEC_RESET;
                switch (cycle_tp->running_stage)
                {
                case PROGRAM_RUNNING:
                case BLOCK_PAUSED:
                case BLOCK_BREAKPOINT:
                    cycle_tp->running_stage = BLOCK_BREAKPOINT;
                    break;
                default:
                    break; 
                }
                break;
            default:
                break;
            } 
            break;
        case SYS_EVENT_RERUN:
            LOG(LOG_LEVEL_INFO, "自动模式处理循环启动");
            cycle_tp->exec_state = EXEC_RUNNING;
            auto_restart_run();
            switch (cycle_tp->running_stage)
            {
            case PROGRAM_RUNNING:
            case BLOCK_PAUSED:
            case BLOCK_BREAKPOINT:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            default:
                break; 
            }
            break;
        default:
            break;
        }
        break;

    case EXEC_RESET:
    default:
        break;
    }
}

static void auto_check_event(){
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_SINGLE_BLOCK_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启单程序");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = SINGLE_BLOCK_ENABLE;
            rt_data->sys_state[SYS_IS_SINGLE_BLOCK] = true;
            break;
        }
        break;
    case SYS_EVENT_SINGLE_BLOCK_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭单程序");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = NONE_EVENT;
            rt_data->sys_state[SYS_IS_SINGLE_BLOCK] = false;
            break;
        }
        break;
    case SYS_EVENT_DRY_RUN_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启空运行");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            switch(cycle_tp->exec_cmd){
            case DO_CHOP_CMD:
            case DO_HOME_CMD:
            case DO_TAP_CMD:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            case DO_SKIP_CMD:   // 跳转不受空运行影响
            case DO_DWELL_CMD:
            case DO_MSTB_CMD:
            default:
                break;
            }
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = DRY_RUN_ENABLE;
            rt_data->sys_state[SYS_IS_DRY_RUN] = true;
            break;
        }
        break;
    case SYS_EVENT_DRY_RUN_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭空运行");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            switch(cycle_tp->exec_cmd){
            case DO_CHOP_CMD:
            case DO_HOME_CMD:
            case DO_TAP_CMD:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            case DO_SKIP_CMD:
            case DO_DWELL_CMD:
            case DO_MSTB_CMD:
            default:
                break;
            }
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = NONE_EVENT;
            rt_data->sys_state[SYS_IS_DRY_RUN] = false;
            break;
        }
        break;
    case SYS_EVENT_MAC_LOCK_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启机械锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = MAC_LOCK_ENABLE;
            rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = true;
            break;
        }
        break;
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭机械锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            cycle_tp->trigger_event = NONE_EVENT;
            rt_data->sys_state[SYS_IS_MAC_LOCK_ON] = false;
            break;
        }
        break;    
    case SYS_EVENT_STLK_ON:
    case SYS_EVENT_IT_ON:
    case SYS_EVENT_IT1_ON:
    case SYS_EVENT_IT2_ON:
    case SYS_EVENT_IT3_ON:
    case SYS_EVENT_IT4_ON:
    case SYS_EVENT_IT5_ON:
    case SYS_EVENT_PMIT2_ON:
    case SYS_EVENT_PMIT3_ON:
    case SYS_EVENT_PMIT4_ON:
    case SYS_EVENT_PMIT5_ON:
    case SYS_EVENT_NMIT2_ON:
    case SYS_EVENT_NMIT3_ON:
    case SYS_EVENT_NMIT4_ON:
    case SYS_EVENT_NMIT5_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启所有轴互锁");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_IT_LOCK, true);
            break;
        }
        break;
    case SYS_EVENT_STLK_OFF:
    case SYS_EVENT_IT_OFF:
    case SYS_EVENT_IT1_OFF:
    case SYS_EVENT_IT2_OFF:
    case SYS_EVENT_IT3_OFF:
    case SYS_EVENT_IT4_OFF:
    case SYS_EVENT_IT5_OFF:
    case SYS_EVENT_PMIT2_OFF:
    case SYS_EVENT_PMIT3_OFF:
    case SYS_EVENT_PMIT4_OFF:
    case SYS_EVENT_PMIT5_OFF:
    case SYS_EVENT_NMIT2_OFF:
    case SYS_EVENT_NMIT3_OFF:
    case SYS_EVENT_NMIT4_OFF:
    case SYS_EVENT_NMIT5_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭所有轴互锁");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_IT_LOCK, false);
            break;
        }
        break;   
    case SYS_EVENT_BSL_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启程序段开始锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_BSL_LOCK, true);
            break;
        }
        break;
    case SYS_EVENT_BSL_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭程序段开始锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_BSL_LOCK, false);
            break;
        }
        break;
    case SYS_EVENT_CSL_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启切削程序段开始锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_CSL_LOCK, true);
            break;
        }
        break;
    case SYS_EVENT_CSL_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭切削程序段开始锁住");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = BLOCK_BREAKPOINT;
        default:
            rt_set_sys_state(SYS_IS_CSL_LOCK, false);
            break;
        }
        break;
    case SYS_EVENT_BLOCK_SKIP_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理开启跳段");
        cycle_tp->trigger_event = BLOCK_SKIP_ENABLE;
        rt_set_sys_state(SYS_IS_BLOCK_SKIP, true);
        break;    
    case SYS_EVENT_BLOCK_SKIP_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理关闭跳段");
        cycle_tp->trigger_event = NONE_EVENT;
        rt_set_sys_state(SYS_IS_BLOCK_SKIP, false);
        break;
    case SYS_EVENT_STOP_SKIP_ON:
        LOG(LOG_LEVEL_INFO, "自动模式处理跳转信号");
        rt_set_sys_state(SYS_IS_STOP_SKIP, true);
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            switch(cycle_tp->exec_cmd){
            case DO_SKIP_CMD:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            default:
                break;
            }
            break;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            break;
        }
        break;
    case SYS_EVENT_STOP_SKIP_OFF:
        LOG(LOG_LEVEL_INFO, "自动模式处理跳转信号");
        rt_set_sys_state(SYS_IS_STOP_SKIP, false);
        break;
    case SYS_EVENT_FEED_OVERRIDE:
        LOG(LOG_LEVEL_INFO, "自动模式处理进给倍率切换");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            switch(cycle_tp->exec_cmd){
            case DO_CHOP_CMD:
            case DO_HOME_CMD:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            case DO_TAP_CMD:  // 攻丝模式不受倍率影响
            case DO_SKIP_CMD: // 跳转不受倍率影响
            case DO_DWELL_CMD:
            case DO_MSTB_CMD:
            default:
                break;
            }
            break;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            break;
        }
        break;
    case SYS_EVENT_RAPID_OVERRIDE:
        LOG(LOG_LEVEL_INFO, "自动模式处理快速进给倍率切换");
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            switch(cycle_tp->exec_cmd){
            case DO_MOVE_CMD:
                cycle_tp->running_stage = BLOCK_BREAKPOINT;
                break;
            default:
                break;
            }
            break;
        case EXEC_PAUSING:
        case EXEC_RESET:
        case EXEC_IDLE:
            break;
        }
        break;
    default:
        break;
    }
}

static void auto_update_cmd(){
    switch (cycle_tp->running_stage)
    {
    case PROGRAM_END:
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
        case EXEC_PAUSING:
        case EXEC_RESET:
            auto_reset_run();
            auto_check_soft_limit_del_done();
            LOG(LOG_LEVEL_INFO, "程序运行结束");
            break;
        case EXEC_IDLE:
        default:
            cycle_tp->exec_cmd = DO_NULL_CMD;
            break;
        }
        break;
    case PROGRAM_START:
    case PROGRAM_WAIT:
    case BLOCK_STOP_2_END:
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = PROGRAM_RUNNING;
            auto_get_next_cmd();
            break;
        case EXEC_PAUSING:
            auto_pause_run();
            break;
        case EXEC_RESET:
            auto_reset_run();
            auto_check_soft_limit_del_done();
            break;
        case EXEC_IDLE:
        default:
            cycle_tp->exec_cmd = DO_NULL_CMD;
            break;
        }
        break;
    case BLOCK_PAUSED:
        switch (cycle_tp->exec_state)
        {
        case EXEC_RUNNING:
            cycle_tp->running_stage = PROGRAM_RUNNING;
            auto_update_cur_cmd();
            break;
        case EXEC_PAUSING:
            auto_pause_run();
            break;
        case EXEC_RESET:
            auto_reset_run();
            auto_check_soft_limit_del_done();
            break;
        case EXEC_IDLE:
        default:
            cycle_tp->exec_cmd = DO_NULL_CMD;
            break;
        }
        break;
    case BLOCK_ARRIVE_2_END:
        cycle_tp->running_stage = PROGRAM_RUNNING;
        auto_get_next_cmd();
        break;
    case BLOCK_BREAKPOINT:
        cycle_tp->running_stage = PROGRAM_RUNNING;
        auto_update_cur_cmd();
        break;
    case PROGRAM_RUNNING:
    default:
        break;
    }            
}

static void auto_execute_cmd(){
    switch (cycle_tp->exec_cmd)
    {
    case DO_MOVE_CMD:
    case DO_CHOP_CMD:
    case DO_TAP_CMD:
    case DO_SKIP_CMD:
        auto_execute_pos_cmd();
        break;
    case DO_DWELL_CMD:
        auto_execute_dwell_cmd();
        break;
    case DO_MSTB_CMD:
        auto_execute_mstb_cmd();
        break;
    case DO_NULL_CMD:
        auto_execute_null_cmd();
    default:
        break;
    }
}

static void auto_update_stage(){
    switch (cycle_tp->exec_cmd)
    {
    case DO_MOVE_CMD:
    case DO_CHOP_CMD:
    case DO_TAP_CMD:
        if(auto_pos_cmd_is_finished()) {
            cycle_tp->running_stage = cycle_tp->target_stage;
            switch (cycle_tp->target_stage)
            {
            case BLOCK_ARRIVE_2_END:
                auto_pop_current_block();
                break;
            case BLOCK_STOP_2_END:
                auto_pop_current_block();
                cycle_tp->exec_cmd = DO_NULL_CMD;
                cycle_tp->target_stage = PROGRAM_WAIT;
                if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK]) {
                    cycle_tp->exec_state = EXEC_PAUSING;
                }
                if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
                else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
                break;
            case BLOCK_PAUSED:
            case PROGRAM_WAIT:
                if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
                else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
                break;
            default:
                break;
            }
        }
        break;
    case DO_SKIP_CMD:
        if(auto_pos_cmd_is_finished()) {
            cycle_tp->running_stage = cycle_tp->target_stage;
            switch (cycle_tp->target_stage)
            {
            case BLOCK_STOP_2_END:
                queue_pop(plan_queue);
                rt_set_sys_state(SYS_IS_STOP_SKIP, false);
                // 更新宏变量轴地址
                ItpSetOriginMacPos(rt_data->axis_data[AXIS_X].motor_cmd_pos,
                                rt_data->axis_data[AXIS_Y].motor_cmd_pos,
                                rt_data->axis_data[AXIS_Z].motor_cmd_pos,
                                0);
                rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, true);
                LOG(LOG_LEVEL_DEBUG, "探针到位 %f %f %f", rt_data->axis_data[AXIS_X].motor_cmd_pos, rt_data->axis_data[AXIS_Y].motor_cmd_pos, rt_data->axis_data[AXIS_Z].motor_cmd_pos);
                cycle_tp->exec_cmd = DO_NULL_CMD;
                cycle_tp->target_stage = PROGRAM_WAIT;
                if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK]) {
                    cycle_tp->exec_state = EXEC_PAUSING;
                }
                if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
                else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
                break;
            case BLOCK_PAUSED:
            case PROGRAM_WAIT:
                if(rt_data->sys_state[SYS_IS_STOP_SKIP]){
                    queue_pop(plan_queue);
                    rt_set_sys_state(SYS_IS_STOP_SKIP, false);
                    // 更新宏变量轴地址
                    ItpSetOriginMacPos(rt_data->axis_data[AXIS_X].motor_cmd_pos,
                                rt_data->axis_data[AXIS_Y].motor_cmd_pos,
                                rt_data->axis_data[AXIS_Z].motor_cmd_pos,
                                0);
                    rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, true);
                    LOG(LOG_LEVEL_DEBUG, "获取到外部跳转信号后减速停止 %f %f %f", rt_data->axis_data[AXIS_X].motor_cmd_pos, rt_data->axis_data[AXIS_Y].motor_cmd_pos, rt_data->axis_data[AXIS_Z].motor_cmd_pos);
                }
                if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
                else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
                break;
            default:
                break;
            }
        }
        break;
    case DO_DWELL_CMD:
        if(auto_dwell_cmd_is_finished()) {
            cycle_tp->running_stage = cycle_tp->target_stage;
            if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
            else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
            switch (cycle_tp->target_stage)
            {
            case BLOCK_ARRIVE_2_END:
                auto_pop_current_block();
                break;
            case BLOCK_STOP_2_END:
                auto_pop_current_block();
                cycle_tp->exec_cmd = DO_NULL_CMD;
                cycle_tp->target_stage = PROGRAM_WAIT;
                if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK]) {
                    cycle_tp->exec_state = EXEC_PAUSING;
                }
                break;
            default:
                break;
            }
        }
        break;
    case DO_MSTB_CMD:
        if(auto_mstb_cmd_is_finished()){
            cycle_tp->running_stage = cycle_tp->target_stage;
            if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
            else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
            switch (cycle_tp->target_stage)
            {
            case BLOCK_ARRIVE_2_END:
                auto_pop_current_block();
                break;
            case BLOCK_STOP_2_END:
                auto_pop_current_block();
                cycle_tp->exec_cmd = DO_NULL_CMD;
                cycle_tp->target_stage = PROGRAM_WAIT;
                if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK]) {
                    cycle_tp->exec_state = EXEC_PAUSING;
                }
                break;
            default:
                break;
            }
        }
        break;
    case DO_NULL_CMD:
    default:
        cycle_tp->running_stage = cycle_tp->target_stage;
        if (rt_data->sys_state[SYS_IS_MAC_LOCK_ON]) rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = true;
        else rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = false;
        switch (cycle_tp->target_stage)
        {
        case BLOCK_STOP_2_END:
            auto_pop_current_block();
            cycle_tp->exec_cmd = DO_NULL_CMD;
            cycle_tp->target_stage = PROGRAM_WAIT;
            if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK]) {
                cycle_tp->exec_state = EXEC_PAUSING;
            }
            break;
        case PROGRAM_WAIT:
        default:
            break;
        }
        break;
    }
}

// 检查程序是否结束
static bool auto_check_program_end(){
    if(auto_is_program_end()) {
        // 检查程序是否结束
        cycle_tp->running_stage = PROGRAM_END;
        // 检查译码错误
        auto_check_error_line();
        return true;
    }
    return false;
}

static bool auto_is_program_end(){
    return !rt_get_chan_state(0, CHAN_IS_PARSERING) && queue_is_empty(plan_queue) && queue_is_empty(pretreat_queue) && queue_is_empty(parser_queue);
}

static void auto_check_error_line(){
    int error_no = rt_get_parser_error_no();
    int error_line = rt_get_parser_error_line();
    if(error_no >= 0){
        setVarChanInt(0, VAR_CHAN_RUN_ROW, error_line, 0);
        int res = setAlarm(ALARM_PS, ALARM_ERROR, error_no);
        if(res == -8) return;
        LOG(LOG_LEVEL_INFO, "error line: %d", error_line);
        LOG(LOG_LEVEL_ERR, "译码错误 %d", error_no);
    }
}

static void auto_get_next_cmd(){
    // 更新指令队列
    cycle_tp_add_block();
    // 如果队列为空
    if(queue_is_empty(plan_queue)){ 
        cycle_tp->exec_cmd = DO_NULL_CMD;
        cycle_tp->target_stage = PROGRAM_WAIT;
        trace_publisher_reset(0);
        return;
    };
    cycle_tp->plan_block = queue_peak(plan_queue);
    switch (cycle_tp->plan_block->mode)
    {
    case PMC_MODE_CHOP:
    case PMC_MODE_EXACT_STOP:
    case PMC_MODE_MEASURE_TOOL:
    case PMC_MODE_ARRIVE_CHECK:
    case PMC_MODE_TAP:
    case PMC_MODE_TAP_RETURE:
    case PMC_MODE_STOPSKIP:
        auto_get_pos_cmd();
        break;
    case PMC_MODE_PAUSE:
        auto_get_dwell_cmd();
        break;
    case PMC_MODE_M_CMD:
        auto_get_mstb_cmd();
        break;
    case PMC_MODE_NULL:
        auto_get_null_cmd();
        break;
    default:
        cycle_tp->exec_cmd = DO_NULL_CMD;
    }
    trace_publisher_activate(0,
                             cycle_tp->plan_block,
                             cycle_tp->exec_cmd == DO_MOVE_CMD ||
                             cycle_tp->exec_cmd == DO_CHOP_CMD ||
                             cycle_tp->exec_cmd == DO_TAP_CMD);
    auto_update_var();
    rerun_data_store(cycle_tp->plan_block);
}

static void auto_update_cur_cmd(){
    // 更新指令队列
    cycle_tp_add_block();
    // 如果队列为空
    if(queue_is_empty(plan_queue)){
        cycle_tp->exec_cmd = DO_NULL_CMD;
        cycle_tp->target_stage = PROGRAM_WAIT;
        return;
    };
    cycle_tp->plan_block = queue_peak(plan_queue);
    switch (cycle_tp->plan_block->mode)
    {
    case PMC_MODE_CHOP:
    case PMC_MODE_EXACT_STOP:
    case PMC_MODE_MEASURE_TOOL:
    case PMC_MODE_ARRIVE_CHECK:
    case PMC_MODE_TAP:
    case PMC_MODE_TAP_RETURE:
    case PMC_MODE_STOPSKIP:
        auto_update_pos_cmd();
        break;
    case PMC_MODE_PAUSE:
        auto_update_dwell_cmd();
        break;
    case PMC_MODE_M_CMD:
        auto_update_mstb_cmd();
        break;
    case PMC_MODE_NULL:
        auto_update_null_cmd();
        break;
    default:
        cycle_tp->exec_cmd = DO_NULL_CMD;
        break;
    }
    auto_update_var();
}

// 自动模式初始化
static void auto_start_run(){
    // 重置队列
    queue_reset(plan_queue);
    queue_reset(pretreat_queue);
    queue_reset(parser_queue);
    cycle_tp_reset();
    rt_set_zero_vel_params();
    trace_publisher_begin_program(0);
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 0, 0);
    setVarChanInt(0, VAR_CHAN_CYCLE, 1, 0);
    setVarChanInt(0, VAR_CHAN_HOLD, 0, 0);
    // 通知译码运行，初始化译码器
    rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, true);
    // 译码正在运行，译码结束译码器将其置false
    rt_set_chan_state(0, CHAN_IS_PARSERING, true);
    // 设置运行行号
    setVarChanInt(0, VAR_CHAN_RUN_ROW, rerun_data->rerun_row, 0);
    LOG(LOG_LEVEL_DEBUG, "开始加工，速度规划模式 %d, 开启行号 %d", rt_data->chan_data[0].vplan_mode, rerun_data->rerun_row); 
}

static void auto_pause_run(){
    cycle_tp->exec_cmd = DO_NULL_CMD;
    // setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
    setVarChanInt(0, VAR_CHAN_CYCLE, 0, 0);
    setVarChanInt(0, VAR_CHAN_HOLD, 1, 0);
    rt_set_zero_vel_params();
    rerun_store_breakpoint(cycle_tp->plan_block->is_breakpoint);
}

static void auto_restart_run(){
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
    setVarChanInt(0, VAR_CHAN_CYCLE, 1, 0);
    setVarChanInt(0, VAR_CHAN_HOLD, 0, 0);
}

static void auto_reset_run(){
    queue_reset(plan_queue);
    queue_reset(pretreat_queue);
    queue_reset(parser_queue);
    cycle_tp_reset();
    trace_publisher_reset(0);
    rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
    rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
    rt_set_chan_state(0, CHAN_IS_PARSERING, false);
    rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, false);
    setVarChanInt(0, VAR_CHAN_IS_RUNNING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 0, 0);
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);
    setVarChanInt(0, VAR_CHAN_CYCLE, 0, 0);
    setVarChanInt(0, VAR_CHAN_HOLD, 0, 0);
    rt_set_zero_vel_params();
    rerun_store_breakpoint(cycle_tp->plan_block->is_breakpoint);
}

static void auto_update_var(){
    // 运行模态
    VarUnion set_modal;
    set_modal.type = 0;
    for (int i = 0; i < MODAL_GROUP; ++i)
    {
        set_modal.value.i = cycle_tp->plan_block->state.modal[i];
        setVarChanValue(0, VAR_CHAN_MODAL, i, &set_modal, 0);
    }
    // 运行行号
    setVarChanInt(0, VAR_CHAN_RUN_ROW, cycle_tp->plan_block->line_number, 0);
    // LOG(LOG_LEVEL_DEBUG, "line number:%d", cycle_tp->plan_block->line_number);
    // 速度
    switch (cycle_tp->plan_block->mode)
    {
    case PMC_MODE_PAUSE:
    case PMC_MODE_M_CMD:
    case PMC_MODE_STOPSKIP:
    case PMC_MODE_NULL:
        rt_set_zero_vel_params();
    default:
        break;
    }
}

static void auto_pop_current_block()
{
    trace_publisher_complete(0);
    queue_pop(plan_queue);
}

static GNC_RES cycle_tp_pos_cmd_data_init(POS_CMD_DATA* pos_cmd_data){
    if(pos_cmd_data == NULL) return GNC_DATA_NULL;
    cycle_tp->pos_cmd_data.vel_profile = (VEL_PROFILE*)calloc(sizeof(VEL_PROFILE), 1);
    cycle_tp->pos_cmd_data.traj_data = (TRAJ_PARAMS*)calloc(sizeof(TRAJ_PARAMS), 1);
    if (cycle_tp->pos_cmd_data.vel_profile == NULL || cycle_tp->pos_cmd_data.traj_data == NULL) return GNC_DATA_NULL;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES cycle_tp_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    cycle_tp_delete();
    cycle_tp = (COORD_TP*)calloc(sizeof(COORD_TP), 1);
    if(cycle_tp == NULL) return GNC_DATA_NULL;
    cycle_tp->plan_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
    res = cycle_tp_pos_cmd_data_init(&cycle_tp->pos_cmd_data);
    if(cycle_tp->plan_block == NULL || res != GNC_OPERATOR_SUCCESS){
        cycle_tp_delete();
        return GNC_DATA_NULL;
    }
    return res;
}

static void auto_handle_alarm(){
    if(cycle_tp->exec_state == EXEC_IDLE) return;
    cycle_tp->exec_state = EXEC_RESET;
    switch (cycle_tp->running_stage)
    {
    case PROGRAM_START:
    case PROGRAM_END:
    case PROGRAM_WAIT:
    case BLOCK_PAUSED:
    case BLOCK_STOP_2_END:
        cycle_tp->running_stage = PROGRAM_END;
        LOG(LOG_LEVEL_DEBUG, "报警/切换模式下立刻停止");
        break;
    default:
        cycle_tp->running_stage = BLOCK_BREAKPOINT;
        LOG(LOG_LEVEL_DEBUG, "报警/切换模式下减速停止");
        break;
    }   
}

void auto_handle_estop(){
    auto_reset_run();
}

void cycle_tp_delete(){
    if(cycle_tp == NULL) return;
    free(cycle_tp->plan_block);
    cycle_tp->plan_block = NULL;
    free(cycle_tp->pos_cmd_data.vel_profile);
    cycle_tp->pos_cmd_data.vel_profile = NULL;
    free(cycle_tp->pos_cmd_data.traj_data);
    cycle_tp->pos_cmd_data.traj_data = NULL;
    free(cycle_tp);
    cycle_tp = NULL;
}

void cycle_tp_reset(){
    if(cycle_tp == NULL || cycle_tp->plan_block == NULL) return;
    cycle_tp->exec_state = EXEC_IDLE;
    cycle_tp->running_stage = PROGRAM_END;
    cycle_tp->target_stage = PROGRAM_END;
    cycle_tp->exec_cmd = DO_NULL_CMD;
    cycle_tp->plan_block->type = UNKNOWN;
    cycle_tp->ncu_cycle = rt_data->ncu_data.cycle;
    cycle_tp->velplan_mode = rt_data->chan_data->vplan_mode;
    cycle_tp->max_rapid_vel = MIN3(MIN(rt_data->axis_data[0].max_rapid_vel, rt_data->axis_data[0].jog_lowspd), MIN(rt_data->axis_data[1].max_rapid_vel, rt_data->axis_data[1].jog_lowspd), MIN(rt_data->axis_data[2].max_rapid_vel, rt_data->axis_data[2].jog_lowspd));
    cycle_tp->max_rapid_acc = MIN3(rt_data->axis_data[0].max_rapid_acc, rt_data->axis_data[1].max_rapid_acc, rt_data->axis_data[2].max_rapid_acc);
    cycle_tp->max_rapid_jerk = MIN3(rt_data->axis_data[0].max_rapid_jk, rt_data->axis_data[1].max_rapid_jk, rt_data->axis_data[2].max_rapid_jk);
    cycle_tp->max_feed_vel = MIN3(rt_data->axis_data[0].max_feed_vel, rt_data->axis_data[1].max_feed_vel, rt_data->axis_data[2].max_feed_vel);
    cycle_tp->max_feed_acc = MIN3(rt_data->axis_data[0].max_feed_acc, rt_data->axis_data[1].max_feed_acc, rt_data->axis_data[2].max_feed_acc);
    cycle_tp->max_feed_jerk = MIN3(rt_data->axis_data[0].max_feed_jk, rt_data->axis_data[1].max_feed_jk, rt_data->axis_data[2].max_feed_jk);
    cycle_tp_reset_pos_cmd_data();
    cycle_tp_reset_dwell_cmd_data();
    cycle_tp_mstb_cmd_data_reset();
}

// 编辑模式下复位，光标回到首行
void handle_edit_cmd(){
    switch(rt_data->nc_event){
        case SYS_EVENT_HMI_RESET:
            setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
            break;
        default:
            break;
    }
}
