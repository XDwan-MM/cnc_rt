#ifndef STATUS_MSG_H
#define STATUS_MSG_H
#include <stdbool.h>

#include "emc_type.h"
#include "greecnc.h"

/**
 * @brief 任务状态。
 */
typedef struct {
   // EMC_TASK_STAT_MSG parent; // “继承”自 EMC_TASK_STAT_MSG
    NC_MODE mode;
    enum EMC_TASK_STATE state;
    enum EMC_TASK_EXEC execState;
    enum EMC_TASK_INTERP interpState;
    int callLevel;
    int motionLine;
    int currentLine;
    int readLine;
    int optional_stop_state;
    int block_delete_state;
    char file[LINELEN];
    int task_paused;
} EMC_TASK_STAT;



typedef struct {
    // EMC_TRAJ_STAT_MSG parent; // “继承”自 EMC_TRAJ_STAT_MSG
    // double linearUnits;       // 线性单位
    // double angularUnits;      // 角度单位
    // double cycleTime;         // 周期时间（秒）
    // int joints;               // 最大关节数
    // int spindles;             // 最大主轴数
    // int axis_mask;            // 实际存在的轴的掩码
    // enum EMC_TRAJ_MODE mode; // 模式
    int enabled;              // 是否使能
    // int inpos;                // 是否到位
    int queue;                // 挂起的运动数量
    // int activeQueue;          // 正在混合的运动数量
    int queueFull;            // 队列是否已满
    int id;                   // 当前执行的运动ID
    // int paused;               // 是否已暂停
    // double scale;             // 速度比例因子
    // double rapid_scale;       // 快速移动比例因子
    // EmcPose position;         // 当前指令位置
    // EmcPose actualPosition;   // 当前实际位置
    // double velocity;          // 系统速度
    // double acceleration;      // 系统加速度
    // double maxVelocity;       // 最大系统速度
    // double maxAcceleration;   // 最大系统加速度
    // EmcPose probedPosition;   // 探针触发时的位置
    // int probe_tripped;        // 探针是否已触发
    // int probing;              // 是否正在探测
    // int probeval;             // 探针输入当前值
    // int kinematics_type;      // 运动学类型
    // int motion_type;          // 运动类型
    // double distance_to_go;    // 当前移动的剩余距离
    // EmcPose dtg;              // 各轴的剩余距离
    // double current_vel;       // 当前移动的速度
    // int feed_override_enabled; // 进给修调是否启用
    // int adaptive_feed_enabled; // 自适应进给是否启用
    // int feed_hold_enabled;     // 进给保持是否启用
    // StateTag tag;
} EMC_TRAJ_STAT;

typedef struct {
    SpindleState spindle_state;

} EMC_SPINDLE_STAT;

typedef struct {
    // 聚合与运动相关的状态类
    EMC_TRAJ_STAT traj;
    RCS_STATUS status;

    // EMC_JOINT_STAT joint[EMCMOT_MAX_JOINTS];
    // EMC_AXIS_STAT axis[EMCMOT_MAX_AXIS];
    EMC_SPINDLE_STAT spindle[EMCMOT_MAX_SPINDLES];
    // int synch_di[EMCMOT_MAX_DIO];
    // int synch_do[EMCMOT_MAX_DIO];
    // double analog_input[EMCMOT_MAX_AIO];
    // double analog_output[EMCMOT_MAX_AIO];
    // int misc_error[EMCMOT_MAX_MISC_ERROR];
    // int debug;
    // int on_soft_limit;
    // int external_offsets_applied;
    // EmcPose eoffset_pose;
    // int numExtraJoints;
    // int jogging_active;
} EMC_MOTION_STAT;

typedef struct
{
    RCS_STATUS status;
    int feed_override;
    int rapid_override;
    int spindle_override;
    bool coolant_on;
    bool cycle_start;
    bool feed_hold;
    bool reset;
    bool emg;

}EMC_IO_STATE;

typedef struct {
    int echo_serial_number;
    NMLTYPE command_type;
    int status;
    int state;
    int line;
    EMC_TASK_STAT task;  // 任务状态
    EMC_MOTION_STAT motion; // 运动状态
    EMC_IO_STATE io;
    int debug;           // 调试级别
} EMC_STAT;

#endif // STATUS_MSG_H
