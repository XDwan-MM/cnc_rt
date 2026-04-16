#include "mem.h"
#include "mem_mst.h"
#include "mem_gcode.h"
#include "global.h"

void handle_run_cmd(){
    // 自动模式下，报警或切换模式，减速停止
    if(rt_data->sys_state[SYS_IS_ALARMING] || rt_data->sys_state[SYS_IS_SWITCHING_MODE]){
        rt_data->nc_event = SYS_EVENT_RESET;
    }
    execute_mcode();
    execute_gcode();
}