#ifndef DRIVER_HAL_H
#define DRIVER_HAL_H

#include "device.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <GreeMasterAPI.h>

#define MAX_DRIVERS 5

/*=========================================================
 * HAL返回值
 *=========================================================*/
typedef enum
{
    HAL_DRIVER_SUCCESS = 0,

    HAL_DRIVER_ERR_INVALID_PARAM,
    HAL_DRIVER_ERR_DRIVER_NOT_FOUND,
    HAL_DRIVER_ERR_MAX_NUMB_OVER,

    HAL_DRIVER_ERROR = -1

} hal_retcode_t;

/*=========================================================
 * 驱动器工作模式
 *
 * 对应 CiA402 Object 6060
 *=========================================================*/
typedef enum
{
    DRV_MODE_NONE = 0,

    DRV_MODE_CSP, /* 0x08 周期同步位置 */
    DRV_MODE_CSV, /* 0x09 周期同步速度 */
    DRV_MODE_CST, /* 0x0A 周期同步转矩 */

    DRV_MODE_PP, /* 0x01 轮廓位置 */
    DRV_MODE_PV, /* 0x03 轮廓速度 */
    DRV_MODE_PT, /* 0x04 轮廓转矩 */

    DRV_MODE_HOMING /* 0x06 回零 */

} drv_mode_t;

typedef enum
{
    DRV_MODE_SWITCH_IDLE = 0,
    DRV_MODE_SWITCH_DECELERATING,
    DRV_MODE_SWITCH_WAIT_READY,
    DRV_MODE_SWITCH_PREPARE_TARGET,
    DRV_MODE_SWITCH_WAIT_MODE,
    DRV_MODE_SWITCH_WAIT_ENABLED,
    DRV_MODE_SWITCH_DONE,
    DRV_MODE_SWITCH_ERROR
} drv_mode_switch_state_t;

/*=========================================================
 * 驱动器状态
 *
 * 对应 CiA402 状态机
 *=========================================================*/
typedef enum
{
    DRV_STATE_INIT = 0,

    DRV_STATE_NOT_READY_TO_SWITCH_ON,

    DRV_STATE_SWITCHED_ON_DISABLE,

    DRV_STATE_READY_TO_SWITCHED_ON,

    DRV_STATE_SWITCHED_ON,

    DRV_STATE_OPERATION_ENABLED,

    DRV_STATE_FAULT

} drv_state_t;

/* 主轴内部定向状态。定向完成后保持 DONE，直到显式解除。 */
typedef enum
{
    DRV_ORIENT_IDLE = 0,
    DRV_ORIENT_REQUESTED,
    DRV_ORIENT_RUNNING,
    DRV_ORIENT_DONE
} drv_orient_state_t;

/*=========================================================
 * 抽象驱动器
 *
 * 一个 hal_driver_t 对应一个物理驱动器
 *=========================================================*/
typedef struct
{
    /*-----------------------------------------------------
     * 标识信息
     *-----------------------------------------------------*/
    char drv_name[32];

    uint16_t axis_id;

    bool initialized;

    /*-----------------------------------------------------
     * CiA402状态
     *-----------------------------------------------------*/
    drv_state_t state;

    uint16_t status_word;

    uint16_t error_code;

    drv_state_t target_state; /* 上层请求的目标状态 */

    drv_orient_state_t orient_state;
    bool orient_command_logged;
    bool orient_target_reach_cleared;
    uint32_t orient_clear_wait_cycles;
    uint32_t orient_complete_wait_cycles;
    bool orient_status_log_initialized;
    uint16_t orient_last_status_word;

    /*-----------------------------------------------------
     * 工作模式
     *-----------------------------------------------------*/
    drv_mode_t act_mode; /* 6061 */

    drv_mode_t req_mode; /* 请求模式 */

    drv_mode_switch_state_t mode_switch_state;
    drv_mode_t mode_switch_target;
    drv_mode_t mode_switch_source;
    hal_retcode_t mode_switch_result;
    int32_t mode_switch_position;
    uint32_t mode_switch_cycles;

    /*-----------------------------------------------------
     * 实际反馈
     *-----------------------------------------------------*/
    int32_t actual_pos;

    int32_t actual_vel;

    int16_t actual_torque;

    /*-----------------------------------------------------
     * 目标值
     *-----------------------------------------------------*/
    int32_t target_pos;

    int32_t target_vel;

    int16_t target_torque;

    /*-----------------------------------------------------
     * 控制字
     *-----------------------------------------------------*/
    uint16_t control_word;

    /*-----------------------------------------------------
     * 当前驱动器对应的PDO映射
     *-----------------------------------------------------*/
    slave_addr pdo;

} hal_driver_t;

/*=========================================================
 * 全局驱动器管理器
 *=========================================================*/
typedef struct
{
    hal_driver_t *drivers[MAX_DRIVERS];

    uint8_t count;

    bool initialized;

} driver_manager;

/*=========================================================
 * 全局驱动器对象
 *=========================================================*/
extern driver_manager dri_manager;

extern hal_driver_t spindle_driver;
extern hal_driver_t x_axis_driver;
extern hal_driver_t y_axis_driver;
extern hal_driver_t z_axis_driver;

/*=========================================================
 * 目标值API
 *=========================================================*/
hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t *drv, int32_t target_pos);

hal_retcode_t Hal_Driver_Set_Target_Vel(hal_driver_t *drv, int32_t target_vel);

hal_retcode_t Hal_Driver_Set_Target_Tor(hal_driver_t *drv, int16_t target_torque);

/*=========================================================
 * 实际值API
 *=========================================================*/
hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t *drv, int32_t *actual_pos);

hal_retcode_t Hal_Driver_Get_Actual_Vel(hal_driver_t *drv, int32_t *actual_vel);

hal_retcode_t Hal_Driver_Get_Actual_Tor(hal_driver_t *drv, int16_t *actual_torque);

/*=========================================================
 * 周期数据交换
 *
 * 注意：
 * 当前最终架构中，实际CiA402控制由driver.c完成。
 *
 * 该接口保留为通用HAL接口。
 *=========================================================*/
hal_retcode_t Hal_Driver_Update(hal_driver_t *drv, bool read, bool write);

hal_retcode_t Hal_Driver_Update_All(bool read, bool write);

/*=========================================================
 * 驱动器管理器
 *=========================================================*/
hal_retcode_t Hal_Driver_Manager_Init(void);

hal_retcode_t Hal_Driver_Array_Init(void);

hal_retcode_t Hal_Driver_Register(hal_driver_t *drv);

uint8_t Hal_Driver_Get_Count(void);

bool Hal_Driver_Is_Initialized(hal_driver_t *drv);

hal_driver_t *Hal_Driver_Get_By_Axis_Id(uint16_t axis_id);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_HAL_H */
