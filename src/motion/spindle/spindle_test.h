#ifndef SPINDLE_TEST_H
#define SPINDLE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include <stdint.h>
#include <stdbool.h>
#include "hal_driver.h"
#include "global.h"

typedef enum{
    SPINDLE_EVENT_NONE = 0,
    SPINDLE_EVENT_STOP,     // 主轴停止
    SPINDLE_EVENT_CW,       // 主轴正转
    SPINDLE_EVENT_CCW,      // 主轴反转
    SPINDLE_EVENT_ORIENT
}SpindleEvent;

typedef struct{
    SpindleState spl_state;     // 主轴状态
    SpindleEvent spl_event;     // 主轴事件
    int32_t current_speed;       // 主轴速度
    double emot_speed;        // 目标速度
    double mcp_target_speed;    // 面板按键目标速度
    bool mcode_feedback;        // 是否完成M指令反馈
}SPINDLE_DATA;

extern SPINDLE_DATA spl_data;
void spindle_set_state();
void spindle_cal_speed();
bool spindle_is_stopped();
bool spindle_is_rotating();

#ifdef __cplusplus
}
#endif


#endif // SPINDLE_H
