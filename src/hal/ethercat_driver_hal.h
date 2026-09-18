#ifndef ETHERCAT_DRIVER_HAL_H
#define ETHERCAT_DRIVER_HAL_H

#include "driver_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*=========================================================
 * 读取反馈
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Read_Status(hal_driver_t *drv);

hal_retcode_t Ethercat_Driver_Read_Feedback(hal_driver_t *drv);

/*=========================================================
 * 控制字 6040
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Write_Controlword(hal_driver_t *drv, uint16_t control_word);

/*=========================================================
 * 操作模式 6060
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Write_Mode(hal_driver_t *drv, drv_mode_t mode);

/*=========================================================
 * 实际模式 6061
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Read_Mode(hal_driver_t *drv);

/*=========================================================
 * 目标位置 607A
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Write_Target_Position(hal_driver_t *drv, int32_t position);

/*=========================================================
 * 目标速度 60FF
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Write_Target_Velocity(hal_driver_t *drv, int32_t velocity);

/*=========================================================
 * 目标转矩
 *=========================================================*/

hal_retcode_t Ethercat_Driver_Write_Target_Torque(hal_driver_t *drv, int16_t torque);

#ifdef __cplusplus
}
#endif

#endif /* ETHERCAT_DRIVER_HAL_H */