#include "driver.h"
#include "ethercat_driver_hal.h"

#include <stddef.h>
#include <stdlib.h>
#include <hmisysapi.h>

#define DRIVER_MODE_SWITCH_VEL_EPSILON 1
#define DRIVER_MODE_SWITCH_TIMEOUT_CYCLES 5000U

/*=========================================================
 * CiA402状态判断
 *=========================================================*/
static bool Driver_Is_State(uint16_t status, uint16_t mask, uint16_t value) 
{ 
    return ((status & mask) == value); 
}

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

    drv->orient_state = DRV_ORIENT_IDLE;
    drv->orient_command_logged = false;

    drv->control_word = 0;

    drv->target_pos = 0;

    drv->target_vel = 0;

    drv->target_torque = 0;
    drv->mode_switch_state = DRV_MODE_SWITCH_IDLE;
    drv->mode_switch_target = DRV_MODE_NONE;
    drv->mode_switch_source = DRV_MODE_NONE;
    drv->mode_switch_result = HAL_DRIVER_SUCCESS;
    drv->mode_switch_cycles = 0;
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
    return Driver_Request_Mode_Switch(drv, mode);
}

hal_retcode_t Driver_Request_Mode_Switch(hal_driver_t *drv, drv_mode_t mode)
{
    if (drv == NULL || !drv->initialized ||
        (mode != DRV_MODE_CSP && mode != DRV_MODE_CSV))
        return HAL_DRIVER_ERR_INVALID_PARAM;
    if (drv->act_mode == mode)
    {
        drv->mode_switch_state = DRV_MODE_SWITCH_DONE;
        drv->mode_switch_result = HAL_DRIVER_SUCCESS;
        return HAL_DRIVER_SUCCESS;
    }
    if ((drv->act_mode != DRV_MODE_CSV && drv->act_mode != DRV_MODE_CSP) ||
        (drv->state != DRV_STATE_OPERATION_ENABLED &&
         drv->state != DRV_STATE_SWITCHED_ON &&
         drv->state != DRV_STATE_READY_TO_SWITCHED_ON))
        return HAL_DRIVER_ERR_INVALID_PARAM;
    if (drv->mode_switch_state != DRV_MODE_SWITCH_IDLE &&
        drv->mode_switch_state != DRV_MODE_SWITCH_DONE &&
        drv->mode_switch_state != DRV_MODE_SWITCH_ERROR)
        return HAL_DRIVER_ERROR;

    drv->mode_switch_source = drv->act_mode;
    drv->mode_switch_target = mode;
    drv->mode_switch_result = HAL_DRIVER_SUCCESS;
    drv->mode_switch_cycles = 0;
    drv->mode_switch_state = DRV_MODE_SWITCH_DECELERATING;
    drv->target_state = DRV_STATE_READY_TO_SWITCHED_ON;
    return HAL_DRIVER_SUCCESS;
}

bool Driver_Is_Mode_Switching(const hal_driver_t *drv)
{
    return drv != NULL && drv->mode_switch_state != DRV_MODE_SWITCH_IDLE &&
           drv->mode_switch_state != DRV_MODE_SWITCH_DONE &&
           drv->mode_switch_state != DRV_MODE_SWITCH_ERROR;
}

bool Driver_Is_Mode_Switch_Done(const hal_driver_t *drv)
{
    return drv != NULL && drv->mode_switch_state == DRV_MODE_SWITCH_DONE;
}

hal_retcode_t Driver_Get_Mode_Switch_Result(const hal_driver_t *drv)
{
    return drv == NULL ? HAL_DRIVER_ERR_INVALID_PARAM : drv->mode_switch_result;
}

static hal_retcode_t Driver_Process_Mode_Switch(hal_driver_t *drv)
{
    if ((drv->status_word & 0x0008U) != 0U)
    {
        drv->mode_switch_state = DRV_MODE_SWITCH_ERROR;
        drv->mode_switch_result = HAL_DRIVER_ERROR;
        drv->state = DRV_STATE_FAULT;
        return HAL_DRIVER_ERROR;
    }
    if (++drv->mode_switch_cycles > DRIVER_MODE_SWITCH_TIMEOUT_CYCLES)
    {
        drv->mode_switch_state = DRV_MODE_SWITCH_ERROR;
        drv->mode_switch_result = HAL_DRIVER_ERROR;
        drv->target_state = DRV_STATE_READY_TO_SWITCHED_ON;
        return HAL_DRIVER_ERROR;
    }

    switch (drv->mode_switch_state)
    {
    case DRV_MODE_SWITCH_DECELERATING:
        if (drv->mode_switch_source == DRV_MODE_CSV)
            drv->target_vel = 0;
        else
            drv->target_pos = drv->actual_pos;
        if (abs(drv->actual_vel) > DRIVER_MODE_SWITCH_VEL_EPSILON)
            return HAL_DRIVER_SUCCESS;
        drv->mode_switch_position = drv->actual_pos;
        drv->mode_switch_state = DRV_MODE_SWITCH_WAIT_READY;
        drv->mode_switch_cycles = 0;
        /* fall through */
    case DRV_MODE_SWITCH_WAIT_READY:
        drv->target_state = DRV_STATE_READY_TO_SWITCHED_ON;
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0027))
            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_DISABLE_OPERATION);
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0023))
        {
            drv->state = DRV_STATE_SWITCHED_ON;
            drv->mode_switch_state = DRV_MODE_SWITCH_PREPARE_TARGET;
        }
        else
            return HAL_DRIVER_SUCCESS;
        /* fall through */
    case DRV_MODE_SWITCH_PREPARE_TARGET:
        drv->state = DRV_STATE_SWITCHED_ON;
        if (drv->mode_switch_target == DRV_MODE_CSP)
            drv->target_pos = drv->mode_switch_position;
        else
            drv->target_vel = 0;
        if (Ethercat_Driver_Write_Mode(drv, drv->mode_switch_target) != HAL_DRIVER_SUCCESS)
            return HAL_DRIVER_ERROR;
        drv->mode_switch_state = DRV_MODE_SWITCH_WAIT_MODE;
        return HAL_DRIVER_SUCCESS;
    case DRV_MODE_SWITCH_WAIT_MODE:
        if (drv->act_mode != drv->mode_switch_target)
            return HAL_DRIVER_SUCCESS;
        drv->target_state = DRV_STATE_OPERATION_ENABLED;
        drv->mode_switch_state = DRV_MODE_SWITCH_WAIT_ENABLED;
        /* fall through */
    case DRV_MODE_SWITCH_WAIT_ENABLED:
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
            drv->mode_switch_state = DRV_MODE_SWITCH_DONE;
            drv->mode_switch_result = HAL_DRIVER_SUCCESS;
            return HAL_DRIVER_SUCCESS;
        }
        return HAL_DRIVER_SUCCESS;
    default:
        return HAL_DRIVER_SUCCESS;
    }
}

/*=========================================================
 * 请求使能
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

hal_retcode_t Driver_Orient(hal_driver_t *drv)
{
    if (drv == NULL || !drv->initialized)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    /* 请求定向时先确保驱动器仍保持使能，实际发送由周期状态机完成。 */
    drv->target_state = DRV_STATE_OPERATION_ENABLED;
    drv->orient_state = DRV_ORIENT_REQUESTED;
    drv->orient_command_logged = false;
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Driver_Release_Orient(hal_driver_t *drv)
{
    if (drv == NULL || !drv->initialized)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    drv->orient_state = DRV_ORIENT_IDLE;
    drv->target_state = DRV_STATE_READY_TO_SWITCHED_ON;

    /* 显式解除定向，清除 bit10；后续周期再由普通状态机接管。 */
    return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_SHUTDOWN);
}

bool Driver_Is_Orient_Done(const hal_driver_t *drv)
{
    return drv != NULL && drv->orient_state == DRV_ORIENT_DONE;
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
 * 单个驱动器状态机 周期循环处理
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
        if (Driver_Is_Mode_Switching(drv))
        {
            drv->mode_switch_state = DRV_MODE_SWITCH_ERROR;
            drv->mode_switch_result = HAL_DRIVER_ERROR;
        }
        // printf("单个驱动器更新Driver_Cyclic_Process错误 %d驱动器错误状态位为高\n", drv->axis_id);

        return HAL_DRIVER_SUCCESS;
    }

    if (Driver_Is_Mode_Switching(drv))
        return Driver_Process_Mode_Switch(drv);

    /* 定向期间不允许普通状态机或目标值流程覆盖 0x040F。 */
    if (drv->orient_state != DRV_ORIENT_IDLE)
    {
        if (Driver_Is_State(drv->status_word, 0x006F, 0x0027))
        {
            drv->state = DRV_STATE_OPERATION_ENABLED;

            if (drv->orient_state == DRV_ORIENT_REQUESTED)
            {
                drv->orient_state = DRV_ORIENT_RUNNING;
            }
            else if (drv->orient_state == DRV_ORIENT_RUNNING &&
                     (drv->status_word & DRV_STATUS_ORIENT_DONE_MASK) ==
                         DRV_STATUS_ORIENT_DONE_MASK)
            {
                drv->orient_state = DRV_ORIENT_DONE;
            }

            if (!drv->orient_command_logged)
            {
                hal_retcode_t ret =
                    Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_ORIENT);
                if (ret == HAL_DRIVER_SUCCESS)
                {
                    drv->orient_command_logged = true;
                    LOG(LOG_LEVEL_INFO,
                        "主轴定向已下发控制字 0x%04X，等待状态字 bit10 完成",
                        DRV_CONTROL_ORIENT);
                }
                return ret;
            }

            return Ethercat_Driver_Write_Controlword(drv, DRV_CONTROL_ORIENT);
        }

        /* 请求定向但尚未完成使能时，继续执行下面的CiA402使能流程。 */
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
 * 所有驱动器状态机 周期循环处理
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
