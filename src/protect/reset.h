/********************************************************************
 * @file        reset.h
 * @brief       复位
 * @version     V1.00
 * @date        2026/08/27
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 * @details     
 ********************************************************************/

#ifndef RESET_H
#define RESET_H

#include "datatype.h"
#include "plan_queue.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    ESTOP_RESET = 0,    // 1.紧急停止信号ESP
    OUTSIDE_RESET,      // 2.外部复位信号ERS
    BACKWARD_RESET,     // 3.复位&倒带信号RRW
    HMI_RESET,          // 4.按下MDI的RESET键
} RESET_TRIGGER;

typedef struct
{
    RESET_TRIGGER reset_trigger;
    bool is_resetting;
    bool is_resetted;
    bool is_reset_delaying;
    int reset_tic;
    bool is_mdi_reset;
    int mdi_reset_tic;
} RESET_DATA;

GNC_RES reset_data_init();
GNC_RES reset_data_reset();
void nc_set_resetting(RESET_TRIGGER reset_trigger);
void nc_set_resetted();
/* 仅运动线程调用：准备动作放在反馈/轨迹更新之前，以保护停车基准；
 * 完成推进放在 alarm_poll_hal 之后、hal_axis_intent_poll 之前。
 * 查询类接口既不推进硬件，也不授权重新启动。 */
void reset_prepare_motion(void);
bool reset_stop_pending(void);
void reset_stop_executor_finished(bool success);
bool reset_spindle_torque_allowed(bool ordinary);
bool reset_spindle_business_allowed(void);
bool reset_spindle_disable_needed(void);
void reset_spindle_disable_submitted(int rc);
bool reset_position_rebuild_allowed(void);
bool reset_spindle_position_required(void);
void reset_spindle_position_rebuilt(void);

#ifdef __cplusplus
}
#endif
#endif /* RESET_H */
