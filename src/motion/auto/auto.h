
/********************************************************************
 * @file        auto.h
 * @brief       自动模式
 * @version     V1.00
 * @date        2026/07/22
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __AUTO_H__
#define __AUTO_H__

#include "auto_gcode.h"
#include "auto_mst.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUTO_END_TOL 0.001    
#define LOOK_AHEAD_SIZE 200  // 速度前瞻的最大段数

typedef enum
{
    EXEC_IDLE = 0,       // 空闲
    EXEC_RUNNING,        // 自动加工中
    EXEC_PAUSING,        // 正在暂停
    EXEC_RESET,          // 复位
}EXEC_STATE;

typedef enum{
    PROGRAM_END,            // 程序结束
    PROGRAM_START,          // 程序开始
    PROGRAM_WAIT,           // 等待程序  
    PROGRAM_RUNNING,        // 运行程序
    BLOCK_ARRIVE_2_END,     // 完成指令
    BLOCK_STOP_2_END,       // 完成指令，末速度为0
    BLOCK_PAUSED,           // 暂停指令
    BLOCK_BREAKPOINT,       // 中断指令
}RUNNING_STAGE;

typedef enum{
    DO_NULL_CMD = 0,
    DO_MOVE_CMD,       // 定位指令
    DO_CHOP_CMD,       // 插补指令
    DO_DWELL_CMD,      // 等待指令 G04
    DO_MSTB_CMD,       // MST指令
    DO_HOME_CMD,       // 回参考点 G27 G28 G29 G30
    DO_TAP_CMD,        // 攻丝指令 G73 G84
    DO_SKIP_CMD,       // 跳转指令 G31
}CMD_TYPE;

typedef enum{
    NONE_EVENT = 0,
    SINGLE_BLOCK_ENABLE,
    MAC_LOCK_ENABLE,
    ALL_AXIS_LOCK_ENABLE,
    DRY_RUN_ENABLE,
    BLOCK_SKIP_ENABLE,
    FEED_OVERRIDE_CHANGE,
    RAPID_OVERRIDE_CHANGE,
}TRIGGER_EVENT;

typedef struct{
    VEL_PROFILE* vel_profile;      
    TRAJ_PARAMS* traj_data;
    POS current_pos; 
    double current_vel; 
    double target_vel;
    int interp_cycle;               // 累计的插补周期
    double current_distance;        // 单个插补周期当前距离
    double target_distance;         // 单个插补周期目标距离 target_distance = accum_distance - current_distance
    double accum_distance;          // 速度曲线累计插补距离
    double remain_distance;         // 速度曲线剩余插补距离
} POS_CMD_DATA;

typedef struct{
    int dwell_cycle;                // 暂停时间
    int interp_cycle;               // 累计暂停时间
} DWELL_CMD_DATA;

typedef struct{
    bool sub_program;       // 子程序或宏程序标志
    int mcode_size;         // 多个辅助功能个数
    bool signal_mf;         // 辅助功能选通脉冲信号
    bool signal_sf;         // 主轴速度选通信号
    bool signal_tf;         // 刀具功能选通信号
    bool signal_den;        // 分配结束信号
    bool signal_fin;        // 结束信号
    bool m29_fin_low_seen;   /* M29：当前指令内见过 FIN 低电平 */
    bool m29_fin_latched;    /* M29：当前指令的 FIN 已锁存（指令内独占） */
    int  m29_wait_cycles;    /* M29：等握手已过的拍数（只诊断/超时，不参与判定） */
    bool m29_wait_alarmed;   /* M29：等待超时是否已报警（每条指令只报一次） */
    bool pmc_end;           // pmc侧动作完成
    int tfin_count;         // 完成信号计数
    int tmf_count;          // 选通脉冲延时计数
    bool finished;
}MSTB_CMD_DATA;

typedef struct{
    // 加工参数
    VELPLAN_TYPE velplan_mode;
    double max_feed_vel;
    double max_rapid_vel;
    double max_feed_acc;
    double max_rapid_acc;
    double max_feed_jerk;
    double max_rapid_jerk;
    double ncu_cycle;
    double tfin_cycle;
    double tmf_cycle;
    // 运动状态
    PLAN_BLOCK *plan_block;         
    EXEC_STATE exec_state;
    CMD_TYPE exec_cmd;
    RUNNING_STAGE running_stage;
    RUNNING_STAGE target_stage;
    TRIGGER_EVENT trigger_event;          
    // 执行指令
    POS_CMD_DATA pos_cmd_data;
    DWELL_CMD_DATA dwell_cmd_data;
    MSTB_CMD_DATA mstb_cmd_data;
}COORD_TP;

GNC_RES cycle_tp_init();
void cycle_tp_delete();
void cycle_tp_reset();
void handle_auto_cmd();
void handle_edit_cmd();
void auto_handle_estop();

#ifdef __cplusplus
}
#endif

#endif