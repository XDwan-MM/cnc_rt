#ifndef DRIVER_H
#define DRIVER_H

#include "driver_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*=========================================================
 * CiA402 Controlword
 *=========================================================*/

/* Shutdown */
#define DRV_CONTROL_SHUTDOWN 0x0006

/* Switch On */
#define DRV_CONTROL_SWITCH_ON 0x0007

/* Enable Operation */
#define DRV_CONTROL_ENABLE_OPERATION 0x000F

/* Disable Operation */
#define DRV_CONTROL_DISABLE_OPERATION 0x0007

/* Disable Voltage */
#define DRV_CONTROL_DISABLE_VOLTAGE 0x0000

/* Fault Reset */
#define DRV_CONTROL_FAULT_RESET 0x0080

/*=========================================================
 * 初始化
 *=========================================================*/
void Driver_Init(hal_driver_t *drv);

/*=========================================================
 * 单个驱动器周期处理
 *
 * 包括：
 *
 * 1. 读取反馈
 * 2. 判断CiA402状态
 * 3. 控制6040
 * 4. 控制6060
 * 5. 发送目标值
 *=========================================================*/
hal_retcode_t Driver_Cyclic_Process(hal_driver_t *drv);

/*=========================================================
 * 所有驱动器周期处理
 *=========================================================*/
hal_retcode_t Driver_Cyclic_Process_All(void);

/*=========================================================
 * 请求工作模式
 *=========================================================*/
hal_retcode_t Driver_Request_Mode(hal_driver_t *drv, drv_mode_t mode);

/*=========================================================
 * 请求驱动器使能
 *=========================================================*/
hal_retcode_t Driver_Enable(hal_driver_t *drv);

/*=========================================================
 * 驱动器下使能
 *=========================================================*/
hal_retcode_t Driver_Disable(hal_driver_t *drv);

/*=========================================================
 * 故障复位
 *=========================================================*/
hal_retcode_t Driver_Fault_Reset(hal_driver_t *drv);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_H */