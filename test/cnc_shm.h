#ifndef CNC_STATE_H
#define CNC_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "emc_type.h"


// --- 核心：CNC状态结构体 ---
typedef struct {
    // 系统状态变量
    NC_MODE current_mode; //当前模式
    SpindleState spindle_state; //主轴状态
    int feed_override; //进给速度倍率
    int rapid_override; //快速速度倍率
    int spindle_override; //主轴速度倍率
    bool coolant_on;
    bool cycle_start; //循环启动
    bool feed_hold; //进给保持
    bool reset;
    bool emg; //急停

    // 轴向点动状态 (JOG模式下使用)
    bool jog_x_pos;   // +X 正向点动
    bool jog_x_neg;   // -X 负向点动
    bool jog_y_pos;   // +Y 正向点动
    bool jog_y_neg;   // -Y 负向点动
    bool jog_z_pos;   // +Z 正向点动
    bool jog_z_neg;   // -Z 负向点动

    NC_MODE pre_current_mode;
    SpindleState pre_spindle_state;
    int pre_feed_override;
    int pre_rapid_override;
    int pre_spindle_override;
    bool pre_coolant_on;
    bool pre_feed_hold;
    bool pre_reset;
    bool pre_emg;

    // 轴向点动状态 (JOG模式下使用)
    bool pre_jog_x_pos;   // +X 正向点动
    bool pre_jog_x_neg;   // -X 负向点动
    bool pre_jog_y_pos;   // +Y 正向点动
    bool pre_jog_y_neg;   // -Y 负向点动
    bool pre_jog_z_pos;   // +Z 正向点动
    bool pre_jog_z_neg;   // -Z 负向点动

    // 事件描述 (用于传递刚才发生了什么)
    char last_event_msg[128];

    // 运行状态标志
    bool is_running; // 0:停止, 1:运行
} CncState;

// 初始化CNC状态
void cnc_state_init(CncState *state);

#endif // CNC_STATE_H
