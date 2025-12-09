#include "shm.h"

PARAMS *params;     // 机床参数
HAL_DATA *hal;      // 硬件抽象层数据
PLC_REG *plc;       // plc数据
SYS_DATA *sys;      // 系统数据

GCODE_QUEUE *gcode_queue;    // 译码队列
PLAN_QUEUE *pretreat_queue;  // 预处理队列
PLAN_QUEUE *plan_queue;      // 插补队列

MEM_EXEC_STATE mem_exec_state;
MEM_TP *pausing_tp;
MEM_TP *cycle_tp;
bool is_paused_in_block;
VELPLAN_TYPE velplan_type; 

HOMING_TP *x_homing_tp;
HOMING_TP *y_homing_tp;
HOMING_TP *z_homing_tp;

char* log_file = "./files/log.json";


GNC_RES plc_init(){
    plc = (PLC_REG*)malloc(sizeof(PLC_REG));
    plc->nc_mode = MODE_EDIT;
    plc->pause_signal = false;
    plc->reset_signal = false;
    plc->estop_signal = false;
    plc->cycle_start_signal = false;
    plc->enable_signal = true;
}

// 更新：【读取参数】实时组
// 调试：模拟初始化时读取机床参数
GNC_RES param_init(){
    params = (PARAMS*)malloc(sizeof(PARAMS));
    params->cycle_time = 0.001;
    params->rapid_vel = 100;
    params->rapid_acc = 300;
    params->rapid_jerk = 1000;
    params->feed_vel = 100;
    params->feed_acc = 300;
    params->feed_jerk = 1000;
    params->homing_vel = 100;
    params->homing_acc = 300;
    params->homing_jerk = 1000;
    params->chan_num = 1;
    params->homing_type = ABSOLUTE_AXIS_HOMING;
    params->velplan_type = T_SHAPE;
    params->home_pos.x = 0;
    params->home_pos.y = 0;
    params->home_pos.z = 0;
    params->is_lookahead = true;
}

// 更新：【硬件抽象层】初始化 张哲
GNC_RES hal_init(){
    hal = (HAL_DATA*)malloc(sizeof(HAL_DATA));
    memset(hal, 0, sizeof(HAL_DATA));
    memset(&hal->current_pos, 0, sizeof(POS));     
    memset(&hal->target_pos, 0, sizeof(POS));
    memset(&hal->feedback_pos, 0, sizeof(POS));
    // 调试：设置开机初始位置
    hal->current_pos.x = -100;
    hal->current_pos.y = 0;
    hal->current_pos.z = 0;
    hal->target_pos.x = -100;
    hal->target_pos.y = 0;
    hal->target_pos.z = 0;
}

// 更新：【系统参数】实时组
GNC_RES sys_init(){
    sys = (SYS_DATA*)malloc(sizeof(SYS_DATA));
    sys->target_mode = MODE_EDIT;
    sys->current_mode = MODE_EDIT;
    sys->nc_event = SYS_EVENT_NONE;
    sys->is_estop_actived = false;
    sys->is_switching_mode = false;
}

GNC_RES sys_reset(){
    sys->target_mode = MODE_IDLE;
    sys->current_mode = MODE_IDLE;
    sys->nc_event = SYS_EVENT_NONE;
    sys->is_estop_actived = false;
    sys->is_switching_mode = false;
}

GNC_RES tp_init(){
    x_homing_tp = (HOMING_TP*)malloc(sizeof(HOMING_TP));
    y_homing_tp = (HOMING_TP*)malloc(sizeof(HOMING_TP));
    z_homing_tp = (HOMING_TP*)malloc(sizeof(HOMING_TP));
    homing_tp_init(x_homing_tp);
    homing_tp_init(y_homing_tp);
    homing_tp_init(z_homing_tp);
    plan_tp_init();
    // 调试：解释器译码队列
    gcode_queue = (GCODE_QUEUE*)malloc(sizeof(GCODE_QUEUE));
    gcode_queue_init(gcode_queue);
    pretreat_queue = (PLAN_QUEUE*)malloc(sizeof(PLAN_QUEUE));
    queue_init(pretreat_queue);  
    plan_queue = (PLAN_QUEUE*)malloc(sizeof(PLAN_QUEUE));  
    queue_init(plan_queue);  
}

GNC_RES tp_reset(){
    homing_tp_reset(x_homing_tp);
    homing_tp_reset(y_homing_tp);
    homing_tp_reset(z_homing_tp);
    plan_tp_reset();
    gcode_queue_reset(gcode_queue);
    queue_reset(pretreat_queue);
    queue_reset(plan_queue);
}
