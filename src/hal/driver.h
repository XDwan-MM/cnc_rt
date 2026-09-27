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

/* 主轴内部定向：保持到状态字 bit10 和 bit8 同时为1。 */
#define DRV_CONTROL_ORIENT 0x040F
#define DRV_STATUS_ORIENT_DONE_MASK 0x0500

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

/**
 * 安全切换驱动器工作模式。
 *
 * 接口只提交请求：驱动器会先回到 Ready To Switch On（下使能），
 * 再在允许写入 6060 的状态下切换模式。调用者应在确认模式切换完成
 * 后再调用 Driver_Enable()。
 */
hal_retcode_t Driver_Switch_Mode(hal_driver_t *drv, drv_mode_t mode);

hal_retcode_t Driver_Request_Mode_Switch(hal_driver_t *drv, drv_mode_t mode);
bool Driver_Is_Mode_Switching(const hal_driver_t *drv);
bool Driver_Is_Mode_Switch_Done(const hal_driver_t *drv);
hal_retcode_t Driver_Get_Mode_Switch_Result(const hal_driver_t *drv);

/*=========================================================
 * 请求驱动器使能
 *=========================================================*/
hal_retcode_t Driver_Enable(hal_driver_t *drv);

/*=========================================================
 * 驱动器下使能
 *=========================================================*/
hal_retcode_t Driver_Disable(hal_driver_t *drv);

/*=========================================================
 * 主轴内部定向
 *=========================================================*/
hal_retcode_t Driver_Orient(hal_driver_t *drv);
hal_retcode_t Driver_Release_Orient(hal_driver_t *drv);
bool Driver_Is_Orient_Done(const hal_driver_t *drv);

/*=========================================================
 * 故障复位
 *=========================================================*/
hal_retcode_t Driver_Fault_Reset(hal_driver_t *drv);

/*=========================================================
 * 所有驱动器故障复位
 *=========================================================*/
hal_retcode_t Driver_Fault_Reset_All(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_H */
