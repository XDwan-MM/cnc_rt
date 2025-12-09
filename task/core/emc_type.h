#ifndef EMC_TYPES_H
#define EMC_TYPES_H

#include "common.h"

#define  EMCMOT_MAX_SPINDLES 4



/* 定义了一些数组的大小 */
#define ACTIVE_G_CODES 17      // 活动 G 代码数组大小
#define ACTIVE_M_CODES 10      // 活动 M 代码数组大小
#define ACTIVE_SETTINGS 5      // 活动设置数组大小

#define NMLTYPE int

// ---------------------
// EMC 类型声明
// ---------------------

// 用于基础 EMC 的 NML 消息类型

#define EMC_OPERATOR_ERROR_TYPE                      ((NMLTYPE) 11)   // 操作员错误消息
#define EMC_OPERATOR_TEXT_TYPE                       ((NMLTYPE) 12)   // 操作员文本消息
#define EMC_OPERATOR_DISPLAY_TYPE                    ((NMLTYPE) 13)   // 操作员显示消息

#define EMC_NULL_TYPE                                ((NMLTYPE) 21)   // 空消息，通常用于心跳或测试

#define EMC_SET_DEBUG_TYPE                           ((NMLTYPE) 22)   // 设置调试等级的消息

//#define EMC_SYSTEM_CMD_TYPE                          ((NMLTYPE) 30)   // 系统命令消息

// 用于 EMC_JOINT (关节/轴) 组件的 NML 消息类型
#define EMC_JOINT_INIT_TYPE                           ((NMLTYPE) 118)  // 初始化关节
#define EMC_JOINT_HALT_TYPE                           ((NMLTYPE) 119)  // 停止关节
#define EMC_JOINT_ABORT_TYPE                          ((NMLTYPE) 120)  // 中止关节运动
#define EMC_JOINT_ENABLE_TYPE                         ((NMLTYPE) 121)  // 使能关节
#define EMC_JOINT_DISABLE_TYPE                        ((NMLTYPE) 122)  // 禁能关节
#define EMC_JOINT_HOME_TYPE                           ((NMLTYPE) 123)  // 执行回零命令
#define EMC_JOG_CONT_TYPE                             ((NMLTYPE) 124)  // 连续点动命令
#define EMC_JOG_INCR_TYPE                             ((NMLTYPE) 125)  // 增量点动命令
#define EMC_JOG_ABS_TYPE                              ((NMLTYPE) 126)  // 绝对位置点动命令
#define EMC_JOINT_ACTIVATE_TYPE                       ((NMLTYPE) 127)  // 激活关节（用于运动学）
#define EMC_JOINT_DEACTIVATE_TYPE                     ((NMLTYPE) 128)  // 停用关节
#define EMC_JOINT_OVERRIDE_LIMITS_TYPE                ((NMLTYPE) 129)  // 忽略限位命令
#define EMC_JOINT_LOAD_COMP_TYPE                      ((NMLTYPE) 131)  // 加载补偿数据（如螺距误差补偿）
#define EMC_JOINT_UNHOME_TYPE                         ((NMLTYPE) 135)  // 取消回零状态
#define EMC_JOG_STOP_TYPE                             ((NMLTYPE) 136)  // 停止点动


// 用于 EMC_TRAJ (轨迹规划) 组件的 NML 消息类型

// 运动终止条件的定义
#define EMC_TRAJ_TERM_COND_STOP  0  // 终止条件：停止
#define EMC_TRAJ_TERM_COND_EXACT 1  // 终止条件：精确定位
#define EMC_TRAJ_TERM_COND_BLEND 2  // 终止条件：路径混合（拐角不减速到0）

#define EMC_TRAJ_SET_MODE_TYPE                       ((NMLTYPE) 204)  // 设置轨迹模式 (Free, Coord, Teleop)
#define EMC_TRAJ_SET_SCALE_TYPE                      ((NMLTYPE) 209)  // 设置进给倍率
#define EMC_TRAJ_SET_RAPID_SCALE_TYPE                ((NMLTYPE) 238)  // 设置快移倍率

#define EMC_TRAJ_INIT_TYPE                           ((NMLTYPE) 211)  // 初始化轨迹规划器
#define EMC_TRAJ_HALT_TYPE                           ((NMLTYPE) 212)  // 停止轨迹规划
#define EMC_TRAJ_ENABLE_TYPE                         ((NMLTYPE) 213)  // 使能轨迹规划
#define EMC_TRAJ_DISABLE_TYPE                        ((NMLTYPE) 214)  // 禁能轨迹规划
#define EMC_TRAJ_ABORT_TYPE                          ((NMLTYPE) 215)  // 中止轨迹规划
#define EMC_TRAJ_PAUSE_TYPE                          ((NMLTYPE) 216)  // 暂停轨迹规划
#define EMC_TRAJ_RESUME_TYPE                         ((NMLTYPE) 218)  // 恢复执行
#define EMC_TRAJ_DELAY_TYPE                          ((NMLTYPE) 219)  // 设置延时
#define EMC_TRAJ_LINEAR_MOVE_TYPE                    ((NMLTYPE) 220)  // 直线移动指令
#define EMC_TRAJ_CIRCULAR_MOVE_TYPE                  ((NMLTYPE) 221)  // 圆弧移动指令
#define EMC_TRAJ_SET_HOME_TYPE                       ((NMLTYPE) 225)  // 设置逻辑上的原点
#define EMC_TRAJ_SET_ROTATION_TYPE                   ((NMLTYPE) 226)  // 设置坐标系旋转
#define EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE       ((NMLTYPE) 228)  // 清除探针触发标志
#define EMC_TRAJ_PROBE_TYPE                          ((NMLTYPE) 229)  // 探针指令
#define EMC_TRAJ_SET_TELEOP_ENABLE_TYPE              ((NMLTYPE) 230)  // 设置遥操作模式使能
#define EMC_TRAJ_SET_SPINDLESYNC_TYPE                ((NMLTYPE) 232)  // 设置主轴同步（用于刚性攻丝）
#define EMC_TRAJ_SET_SPINDLE_SCALE_TYPE              ((NMLTYPE) 233)  // 设置主轴速度倍率

// EMC_MOTION 聚合类的类型声明

#define EMC_MOTION_INIT_TYPE                         ((NMLTYPE) 301)  // 初始化运动控制器
#define EMC_MOTION_HALT_TYPE                         ((NMLTYPE) 302)  // 停止运动控制器
#define EMC_MOTION_ABORT_TYPE                        ((NMLTYPE) 303)  // 中止运动控制器

// 用于 EMC_TASK (任务) 组件的 NML 消息类型

#define EMC_TASK_INIT_TYPE                           ((NMLTYPE) 501)  // 初始化任务控制器
#define EMC_TASK_HALT_TYPE                           ((NMLTYPE) 502)  // 停止任务控制器
#define EMC_TASK_ABORT_TYPE                          ((NMLTYPE) 503)  // 中止任务控制器
#define EMC_TASK_SET_MODE_TYPE                       ((NMLTYPE) 504)  // 设置任务模式 (MANUAL, AUTO, MDI):OK
#define EMC_TASK_SET_STATE_TYPE                      ((NMLTYPE) 505)  // 设置任务状态 (ON, OFF, ESTOP, ESTOP_RESET)
#define EMC_TASK_PLAN_OPEN_TYPE                      ((NMLTYPE) 506)  // 打开 G 代码文件: OK
#define EMC_TASK_PLAN_RUN_TYPE                       ((NMLTYPE) 507)  // 运行 G 代码 : OK
#define EMC_TASK_PLAN_READ_TYPE                      ((NMLTYPE) 508)  // 预读 G 代码
#define EMC_TASK_PLAN_EXECUTE_TYPE                   ((NMLTYPE) 509)  // 执行单条 G 代码（通常用于 MDI）
#define EMC_TASK_PLAN_PAUSE_TYPE                     ((NMLTYPE) 510)  // 暂停 G 代码执行
#define EMC_TASK_PLAN_STEP_TYPE                      ((NMLTYPE) 511)  // 单步运行 G 代码
#define EMC_TASK_PLAN_RESUME_TYPE                    ((NMLTYPE) 512)  // 恢复 G 代码执行
#define EMC_TASK_PLAN_END_TYPE                       ((NMLTYPE) 513)  // 结束 G 代码执行
#define EMC_TASK_PLAN_CLOSE_TYPE                     ((NMLTYPE) 514)  // 关闭 G 代码文件
#define EMC_TASK_PLAN_INIT_TYPE                      ((NMLTYPE) 515)  // 初始化 G 代码解释器
#define EMC_TASK_PLAN_SYNCH_TYPE                     ((NMLTYPE) 516)  // 同步 G 代码解释器
#define EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE         ((NMLTYPE) 517)  // 设置选择性暂停开关 (M1)
#define EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE          ((NMLTYPE) 518)  // 设置程序段跳过开关 (/)
#define EMC_TASK_PLAN_OPTIONAL_STOP_TYPE             ((NMLTYPE) 519)  // （内部使用）执行选择性暂停
#define EMC_TASK_PLAN_REVERSE_TYPE                   ((NMLTYPE) 520)  // 反向运行
#define EMC_TASK_PLAN_FORWARD_TYPE                   ((NMLTYPE) 521)  // 正向运行

// EMC_AUX (辅助功能) 组件的类型声明（1）
#define EMC_AUX_ESTOP_ON_TYPE                         ((NMLTYPE) 1206) // 触发急停
#define EMC_AUX_ESTOP_OFF_TYPE                        ((NMLTYPE) 1207) // 关闭急停（仅限内部逻辑）
#define EMC_AUX_ESTOP_RESET_TYPE                      ((NMLTYPE) 1208) // 复位急停
#define EMC_AUX_INPUT_WAIT_TYPE                       ((NMLTYPE) 1209) // 等待输入信号

// EMC_SPINDLE (主轴) 组件的类型声明
#define EMC_SPINDLE_ON_TYPE                          ((NMLTYPE) 1304) // 启动主轴（正转/反转）
#define EMC_SPINDLE_OFF_TYPE                         ((NMLTYPE) 1305) // 关闭主轴
#define EMC_SPINDLE_SPEED_TYPE                       ((NMLTYPE) 1316) // 设置主轴速度 (S 指令)

#define EMC_INIT_TYPE                                ((NMLTYPE) 1901) // 初始化整个 EMC 系统
#define EMC_HALT_TYPE                                ((NMLTYPE) 1902) // 停止整个 EMC 系统
#define EMC_ABORT_TYPE                               ((NMLTYPE) 1903) // 中止整个 EMC 系统

// EMC_TASK 状态的枚举类型
enum EMC_TASK_STATE {
    ESTOP = 1,       // 急停状态
    ESTOP_RESET = 2, // 急停复位状态
    OFF = 3,         // 控制器关闭状态
    ON = 0          // 控制器开启状态
};

// EMC_TASK 执行状态的枚举类型
enum EMC_TASK_EXEC {
    TASK_STATE_ERROR = 1,           // 错误
    TASK_STATE_DONE = 2,            // 完成
    WAITING_FOR_MOTION = 3,         // 等待运动完成
    WAITING_FOR_MOTION_QUEUE = 4,   // 等待运动队列
    WAITING_FOR_IO = 5,             // 等待 IO
    WAITING_FOR_MOTION_AND_IO = 7,  // 等待运动和 IO
    WAITING_FOR_DELAY = 8,          // 等待延时
    WAITING_FOR_SYSTEM_CMD = 9,     // 等待系统命令
    WAITING_FOR_SPINDLE_ORIENTED = 10 // 等待主轴定向完成
};

// EMC_TASK 解释器状态的枚举类型
enum EMC_TASK_INTERP {
    IDLE = 1,    // 空闲
    READING = 2, // 正在读取 G 代码
    PAUSED = 3,  // 已暂停
    WAITING = 4  // 等待（例如，等待用户输入）
};

// 运动控制模式的枚举类型
enum EMC_TRAJ_MODE {
    FREE = 1,	 // 自由模式（轴独立运动）
    COORD = 2,	 // 协调模式（多轴联动）
    TELEOP = 3	 // 遥操作模式（基于世界坐标系的速度控制）
};

// emcIoAbort() 中止原因的枚举类型
enum EMC_ABORT {
    TASK_EXEC_ERROR = 1,          // 任务执行错误
    AUX_ESTOP = 2,                // 辅助急停
    MOTION_OR_IO_RCS_ERROR = 3,   // 运动或IO的RCS通信错误
    TASK_STATE_OFF = 4,           // 任务状态为 OFF
    TASK_STATE_ESTOP_RESET = 5,   // 任务状态为 ESTOP_RESET
    TASK_STATE_ESTOP = 6,         // 任务状态为 ESTOP
    TASK_STATE_NOT_ON = 7,        // 任务状态未 ON
    TASK_ABORT = 8,               // 任务中止
    INTERPRETER_ERROR = 9,	      // G代码解释器在预读时出错
    INTERPRETER_ERROR_MDI = 10,	  // G代码解释器在执行MDI时出错
    USER = 100                    // 用户自定义的中止代码起始值
};

typedef enum
{
    RCS_UNINITIALIZED = -1,
    RCS_DONE = 1,
    RCS_EXEC = 2,
    RCS_ERROR = 3
}RCS_STATUS;

//NC_MODE {
//    MODE_JOG = 1,   // 点动模式
//    MODE_RAPID = 2, //快速模式
//    MODE_MPG = 3,   // 手轮模式
//    MODE_HOMING = 4,   // 回原点模式
//    MODE_EDIT = 5,  // 编辑模式
//    MODE_MEM = 6,   // 自动模式
//    MODE_MDI = 7,   // MDI (手动数据输入) 模式
//    MODE_DNC = 8,   // 远程连接模式
//};

#endif // EMC_TYPES_H
