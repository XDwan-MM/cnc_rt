#include "mem.h"
#include "mem_mst.h"
#include "mem_gcode.h"
#include "global.h"
#include <hmisysapi.h> 
#include <alarmapi.h>

static void auto_check_error();
static int error_no = -1;

void handle_run_cmd(){
    // 自动模式下，报警或切换模式，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        rt_data->nc_event = SYS_EVENT_RESET;
    }
    execute_mcode();
    execute_gcode();
    // auto_check_error();
}

void auto_handle_estop(){
    mcode_handle_estop();
    gcode_handle_estop();
}

static void auto_check_error(){
    if(cycle_tp->mem_exec_state == EXEC_IDLE){
        error_no = rt_get_parser_state();
        if(error_no >= 0){
            int res = setAlarm(ALARM_PS, ALARM_ERROR, error_no);
            if(res == -8) return;
            LOG(LOG_LEVEL_ERR, "译码错误 %d", error_no);
        }
    }
}