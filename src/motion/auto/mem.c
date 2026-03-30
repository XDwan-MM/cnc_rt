#include "mem.h"
#include "mem_mst.h"
#include "mem_gcode.h"

void handle_run_cmd(){
    execute_mcode();
    execute_gcode();
}