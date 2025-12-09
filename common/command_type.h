#ifndef COMMAND_TYPE_H
#define COMMAND_TYPE_H

#define NMLTYPE int

#define EMC_NULL_TYPE                                ((NMLTYPE) 21)   // 空消息，通常用于心跳或测试

// 用于 EMC_TASK (任务) 组件的 NML 消息类型

//#define EMC_TASK_INIT_TYPE                           ((NMLTYPE) 501)  // 初始化任务控制器
//#define EMC_TASK_HALT_TYPE                           ((NMLTYPE) 502)  // 停止任务控制器
//#define EMC_TASK_ABORT_TYPE                          ((NMLTYPE) 503)  // 中止任务控制器
#define EMC_TASK_SET_MODE_TYPE                       ((NMLTYPE) 504)  // 设置任务模式 (MEM, MDI,...)
#define EMC_TASK_SET_STATE_TYPE                      ((NMLTYPE) 505)  // 设置任务状态 (ON, OFF, ESTOP, ESTOP_RESET)
//#define EMC_TASK_PLAN_OPEN_TYPE                      ((NMLTYPE) 506)  // 打开 G 代码文件
//#define EMC_TASK_PLAN_RUN_TYPE                       ((NMLTYPE) 507)  // 运行 G 代码
//#define EMC_TASK_PLAN_READ_TYPE                      ((NMLTYPE) 508)  // 预读 G 代码
//#define EMC_TASK_PLAN_EXECUTE_TYPE                   ((NMLTYPE) 509)  // 执行单条 G 代码（通常用于 MDI）
//#define EMC_TASK_PLAN_PAUSE_TYPE                     ((NMLTYPE) 510)  // 暂停 G 代码执行
//#define EMC_TASK_PLAN_STEP_TYPE                      ((NMLTYPE) 511)  // 单步运行 G 代码
//#define EMC_TASK_PLAN_RESUME_TYPE                    ((NMLTYPE) 512)  // 恢复 G 代码执行
//#define EMC_TASK_PLAN_END_TYPE                       ((NMLTYPE) 513)  // 结束 G 代码执行
//#define EMC_TASK_PLAN_CLOSE_TYPE                     ((NMLTYPE) 514)  // 关闭 G 代码文件
//#define EMC_TASK_PLAN_INIT_TYPE                      ((NMLTYPE) 515)  // 初始化 G 代码解释器
//#define EMC_TASK_PLAN_SYNCH_TYPE                     ((NMLTYPE) 516)  // 同步 G 代码解释器
//#define EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE         ((NMLTYPE) 517)  // 设置选择性暂停开关 (M1)
//#define EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE          ((NMLTYPE) 518)  // 设置程序段跳过开关 (/)
//#define EMC_TASK_PLAN_OPTIONAL_STOP_TYPE             ((NMLTYPE) 519)  // （内部使用）执行选择性暂停
//#define EMC_TASK_PLAN_REVERSE_TYPE                   ((NMLTYPE) 520)  // 反向运行
//#define EMC_TASK_PLAN_FORWARD_TYPE                   ((NMLTYPE) 521)  // 正向运行



#endif // COMMAND_TYPE_H
