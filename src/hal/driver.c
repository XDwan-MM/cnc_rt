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

    drv->target_state = DRV_STATE_SWITCHED_ON_DISABLE;

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
 * 安全切换工作模式
 *=========================================================*/
hal_retcode_t Driver_Switch_Mode(hal_driver_t *drv, drv_mode_t mode)
{
    hal_retcode_t ret;

    if (drv == NULL || mode == DRV_MODE_NONE)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    /* 模式只能在未使能状态写入，先请求下使能。 */
    drv->target_state = DRV_STATE_SWITCHED_ON_DISABLE;
    ret = Driver_Request_Mode(drv, mode);
    return ret;
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

    drv->target_state = DRV_STATE_OPERATION_ENABLED;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 驱动器下使能
 *
 * 这里使用Disable Operation：
 *
 * 请求回到 Ready to Switch On。
 *=========================================================*/
hal_retcode_t Driver_Disable(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    /* 下使能的最终状态是 Switch On Disabled。 */
    drv->target_state = DRV_STATE_SWITCHED_ON_DISABLE;
    return HAL_DRIVER_SUCCESS;
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
 * 所有驱动器故障复位
 *=========================================================*/
hal_retcode_t Driver_Fault_Reset_All(void)
{
    if (!dri_manager.initialized)
    {
        return HAL_DRIVER_ERROR;
    }

    for (uint8_t i = 0; i < dri_manager.count; i++)
    {
        hal_driver_t *drv = dri_manager.drivers[i];
        hal_retcode_t ret;

        if (drv == NULL)
        {
            continue;
        }
        if ((drv->status_word & 0x0008) == 0x0008)
        {
            // 执行清错
            ret = Driver_Fault_Reset(drv);
            // printf("%d驱动器执行了清错\n", drv->axis_id);
            if (ret != HAL_DRIVER_SUCCESS)
            {
                printf("Driver_Fault_Reset_All Driver_Fault_Reset err %d驱动器清除报警失败\n", drv->axis_id);
                return ret;
            }
        }
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 单个驱动器周期状态机
 *=========================================================*/
hal_retcode_t Driver_Cyclic_Process(hal_driver_t *drv)
{
    if (drv == NULL || !drv->initialized)
    {
        printf("单个驱动器更新Driver_Cyclic_Process 错误 drv->initialized未初始化\n");
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
        // printf("单个驱动器更新Driver_Cyclic_Process错误 %d驱动器错误状态位为高\n", drv->axis_id);

        return HAL_DRIVER_SUCCESS;
    }

    /* Switch On Disabled: select the requested mode before shutdown. */
    if (Driver_Is_State(drv->status_word, 0x004F, 0x0040))
    {
        drv->state = DRV_STATE_SWITCHED_ON_DISABLE;

        if (drv->req_mode == DRV_MODE_NONE)
        {
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SHUTDOWN);
        }

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

    /* A disable request follows the CiA402 sequence 0x0007 -> 0x0006. */
    if (drv->target_state == DRV_STATE_SWITCHED_ON_DISABLE)
    {
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0027))
        {
            drv->state = DRV_STATE_OPERATION_ENABLED;
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_DISABLE_OPERATION);
        }
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0023))
        {
            drv->state = DRV_STATE_SWITCHED_ON;
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SHUTDOWN);
        }
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0021))
        {
            drv->state = DRV_STATE_READY_TO_SWITCHED_ON;
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_DISABLE_VOLTAGE);
        }

        if (Driver_Is_State(drv->status_word, 0x004F, 0x0040))
        {
            drv->state = DRV_STATE_SWITCHED_ON_DISABLE;
            return HAL_DRIVER_SUCCESS;
        }
    }

    /* An enable request follows 0x0006 -> 0x0007 -> 0x000F. */
    if (drv->target_state == DRV_STATE_OPERATION_ENABLED)
    {
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0021))
        {
            drv->state = DRV_STATE_READY_TO_SWITCHED_ON;
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SWITCH_ON);
        }
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0023))
        {
            drv->state = DRV_STATE_SWITCHED_ON;
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_ENABLE_OPERATION);
        }
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0027))
        {
            drv->state = DRV_STATE_OPERATION_ENABLED;
            switch (drv->req_mode)
            {
            case DRV_MODE_CSP:
                return Ethercat_Driver_Write_Target_Position(drv, drv->target_pos);
            case DRV_MODE_CSV:
                return Ethercat_Driver_Write_Target_Velocity(drv, drv->target_vel);
            case DRV_MODE_CST:
                break;
            default:
                break;
            }
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
        printf("Driver_Cyclic_Process_All 驱动器管理器未初始化\n");
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
            printf("Driver_Cyclic_Process_All 更新所有驱动器时 单个驱动器更新Driver_Cyclic_Process 错误\n");
            return ret;
        }
    }

    return HAL_DRIVER_SUCCESS;
}
