#include "driver.h"
#include "ethercat_driver_hal.h"

#include <stddef.h>

/*=========================================================
 * CiA402状态判断
 *=========================================================*/
static bool Driver_Is_State(uint16_t status, uint16_t mask, uint16_t value) { return ((status & mask) == value); }

/*=========================================================
 * 初始化驱动器控制状态
 *=========================================================*/
void Driver_Init(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return;
    }

    drv->state = DRV_STATE_INIT;

    drv->act_mode = DRV_MODE_NONE;

    drv->req_mode = DRV_MODE_NONE;

    drv->control_word = 0;

    drv->target_pos = 0;

    drv->target_vel = 0;

    drv->target_torque = 0;
}

/*=========================================================
 * 请求工作模式
 *=========================================================*/
hal_retcode_t Driver_Request_Mode(hal_driver_t *drv, drv_mode_t mode)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    if (mode == DRV_MODE_NONE)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    drv->req_mode = mode;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 请求使能
 *
 * 当前版本：
 * CiA402状态机自动完成使能。
 *=========================================================*/
hal_retcode_t Driver_Enable(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    /*
     * 当前不需要额外动作。
     *
     * Driver_Cyclic_Process()
     * 自动完成：
     *
     * 0x0006
     *     ↓
     * 0x0007
     *     ↓
     * 0x000F
     */

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 驱动器下使能
 *
 * 这里使用Disable Operation：
 *
 * 0x000F → 0x0007
 *
 * 而不是直接断电：
 *
 * 0x000F → 0x0000
 *=========================================================*/
hal_retcode_t Driver_Disable(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_DISABLE_OPERATION);
}

/*=========================================================
 * 故障复位
 *=========================================================*/
hal_retcode_t Driver_Fault_Reset(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_FAULT_RESET);
}

/*=========================================================
 * 单个驱动器周期状态机
 *=========================================================*/
hal_retcode_t Driver_Cyclic_Process(hal_driver_t *drv)
{
    if (drv == NULL || !drv->initialized)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    /*
     * 注意：
     *
     * 本函数不再读取EtherCAT PDO。
     *
     * 所有反馈数据已经由：
     *
     * Master_Receive()
     *      ↓
     * Hal_Driver_Update_All(true, false)
     *
     * 统一读取并缓存到 drv 中。
     */

    /* -----------------------------------------------------
     * 1. Fault判断
     *
     * Statusword bit3 = Fault
     * ----------------------------------------------------- */

    if ((drv->status_word & 0x0008) != 0)
    {
        drv->state = DRV_STATE_FAULT;

        return HAL_DRIVER_SUCCESS;
    }

    /* -----------------------------------------------------
     * 2. Not Ready To Switch On
     * ----------------------------------------------------- */

    if (Driver_Is_State(drv->status_word, 0x004F, 0x0000))
    {
        drv->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;

        return HAL_DRIVER_SUCCESS;
    }

    /* -----------------------------------------------------
     * 3. Switch On Disabled
     * ----------------------------------------------------- */

    if (Driver_Is_State(drv->status_word, 0x004F, 0x0040))
    {
        drv->state = DRV_STATE_SWITCHED_ON_DISABLE;

        /*
         * 此时可以设置工作模式。
         */

        if (drv->req_mode != DRV_MODE_NONE)
        {
            hal_retcode_t ret;

            ret = Ethercat_Driver_Write_Mode(drv, drv->req_mode);

            if (ret != HAL_DRIVER_SUCCESS)
            {
                return ret;
            }
        }

        return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SHUTDOWN);
    }

    /* -----------------------------------------------------
     * 4. Ready To Switch On
     * ----------------------------------------------------- */

    if (Driver_Is_State(drv->status_word, 0x006F, 0x0021))
    {
        drv->state = DRV_STATE_READY_TO_SWITCHED_ON;

        return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SWITCH_ON);
    }

    /* -----------------------------------------------------
     * 5. Switched On
     * ----------------------------------------------------- */

    if (Driver_Is_State(drv->status_word, 0x006F, 0x0023))
    {
        drv->state = DRV_STATE_SWITCHED_ON;

        return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_ENABLE_OPERATION);
    }

    /* -----------------------------------------------------
     * 6. Operation Enabled
     * ----------------------------------------------------- */

    if (Driver_Is_State(drv->status_word, 0x027F, 0x0237))
    {
        drv->state = DRV_STATE_OPERATION_ENABLED;

        /*
         * 这里直接使用Master本周期已经读取好的：
         *
         * drv->act_mode
         *
         * 不再调用：
         *
         * Ethercat_Driver_Read_Mode()
         */

        switch (drv->req_mode)
        {
        case DRV_MODE_CSP:

            return Ethercat_Driver_Write_Target_Position(drv, drv->target_pos);

        case DRV_MODE_CSV:

            return Ethercat_Driver_Write_Target_Velocity(drv, drv->target_vel);

        case DRV_MODE_CST:

            return Ethercat_Driver_Write_Target_Torque(drv, drv->target_torque);

        default:

            return HAL_DRIVER_SUCCESS;
        }
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 所有驱动器周期处理
 *=========================================================*/
hal_retcode_t Driver_Cyclic_Process_All(void)
{
    if (!dri_manager.initialized)
    {
        return HAL_DRIVER_ERROR;
    }

    for (uint8_t i = 0; i < dri_manager.count; i++)
    {
        hal_driver_t *drv = dri_manager.drivers[i];

        if (drv == NULL)
        {
            continue;
        }

        hal_retcode_t ret = Driver_Cyclic_Process(drv);

        if (ret != HAL_DRIVER_SUCCESS)
        {
            return ret;
        }
    }

    return HAL_DRIVER_SUCCESS;
}