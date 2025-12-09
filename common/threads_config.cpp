#include "threads_config.h"

thread_config_t plc_config = {
    .core_id = 3,
    .policy = SCHED_RR,
    .priority = 30,
    .thread_name = "软PLC"
};

thread_config_t task_config = {
    .core_id = 3,
    .policy = SCHED_RR,
    .priority = 30,
    .thread_name = "任务控制器"
};

thread_config_t motion_config = {
    .core_id = 2,
    .policy = SCHED_RR,
    .priority = 48,
    .thread_name = "运动控制器"
};

thread_config_t parser_config = {
    .core_id = 2,
    .policy = SCHED_RR,
    .priority = 20,
    .thread_name = "解释器线程"
};

// 函数指针初始化
void (*plc_func_ptr)() = PLC_run;
void (*task_func_ptr)() = task_run;
void (*motion_func_ptr)() = motion_run;
void (*parser_func_ptr)() = parser_run;

void PLC_run()
{

}

void task_run()
{

}

void motion_run()
{

}

void parser_run()
{

}
