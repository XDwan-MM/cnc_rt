#include "driver_hal.h"
#include "ethercat_driver_hal.h"
#include "debug_mode.h"

#include <stddef.h>
#include <string.h>

/*=========================================================
 * EtherCAT设备编号
 *
 * 注意：
 * 这里是EtherCAT设备ID，不是逻辑轴ID。
 *=========================================================*/
typedef enum
{
    DEV_ID_CONTROL_PANEL = 0,
    DEV_ID_IO_MODULE = 1,

    DEV_ID_SPINDLE_DRIVER = 2,
    DEV_ID_X_AXIS_DRIVER = 3,
    DEV_ID_Y_AXIS_DRIVER = 4,
    DEV_ID_Z_AXIS_DRIVER = 5,

    DEV_ID_COUNT

} DeviceId_e;

/*=========================================================
 * 物理驱动器配置
 *=========================================================*/
static bool physical_sp = true;
static bool physical_x = true;
static bool physical_y = true;
static bool physical_z = true;

/*=========================================================
 * 逻辑轴ID
 *
 * 0 = 主轴
 * 1 = X
 * 2 = Y
 * 3 = Z
 *=========================================================*/
static uint16_t id_sp = 0;
static uint16_t id_x = 1;
static uint16_t id_y = 2;
static uint16_t id_z = 3;

/*=========================================================
 * 全局对象
 *=========================================================*/
driver_manager dri_manager;

hal_driver_t spindle;
hal_driver_t x_axis_driver;
hal_driver_t y_axis_driver;
hal_driver_t z_axis_driver;

/*=========================================================
 * 内部函数
 *=========================================================*/
static void Hal_Driver_Clear(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return;
    }

    memset(drv, 0, sizeof(hal_driver_t));

    drv->state = DRV_STATE_INIT;

    drv->act_mode = DRV_MODE_NONE;

    drv->req_mode = DRV_MODE_NONE;
}

/*=========================================================
 * 初始化驱动器管理器
 *=========================================================*/
hal_retcode_t Hal_Driver_Manager_Init(void)
{
    hal_driver_t* drivers[] = {&spindle, &x_axis_driver, &y_axis_driver, &z_axis_driver};
    const bool physical[] = {physical_sp, physical_x, physical_y, physical_z};
    const uint16_t axis_ids[] = {id_sp, id_x, id_y, id_z};
    const size_t driver_count = sizeof(drivers) / sizeof(drivers[0]);

    dri_manager.initialized = true;
    

    for (size_t i = 0; i < driver_count; i++)
    {
        hal_retcode_t result;

        if (!physical[i])
        {
            continue;
        }

        result = Hal_Driver_Register(drivers[i]);
        if (result != HAL_DRIVER_SUCCESS)
        {
            printf("(result != HAL_DRIVER_SUCCESS) err\n");
            return result;
        }

        drivers[i]->axis_id = axis_ids[i];
        drivers[i]->initialized = true;
        drivers[i]->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;
        drivers[i]->act_mode = DRV_MODE_NONE;
        drivers[i]->req_mode = DRV_MODE_NONE;
        drivers[i]->enable_requested = true;
    }
    dri_manager.count = driver_count;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 初始化所有驱动器
 *=========================================================*/
hal_retcode_t Hal_Driver_Array_Init(void)
{
    hal_driver_t *drivers[] = {&spindle, &x_axis_driver, &y_axis_driver, &z_axis_driver};

    const bool physical[] = {physical_sp, physical_x, physical_y, physical_z};

    const uint16_t axis_ids[] = {id_sp, id_x, id_y, id_z};

    const DeviceId_e device_ids[] = {DEV_ID_SPINDLE_DRIVER,
                                     DEV_ID_X_AXIS_DRIVER,
                                     DEV_ID_Y_AXIS_DRIVER,
                                     DEV_ID_Z_AXIS_DRIVER};

    const size_t driver_count = sizeof(drivers) / sizeof(drivers[0]);

    if (!dri_manager.initialized)
    {
        printf("(!dri_manager.initialized) err\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    for (size_t i = 0; i < driver_count; i++)
    {
        hal_driver_t *drv = drivers[i];

        if (!physical[i])
        {
            continue;
        }

        /* 清空对象 */
        // Hal_Driver_Clear(drv);

        /*-------------------------------------------------
         * 逻辑轴ID
         *-------------------------------------------------*/
        drv->axis_id = axis_ids[i];

        /*-------------------------------------------------
         * 保存该驱动器自己的PDO映射
         *
         * 不再使用：
         *
         * g_device_data[axis_id + 2]
         *-------------------------------------------------*/
        drv->pdo = g_device_data[device_ids[i]].slave;


        /*-------------------------------------------------
         * 初始化状态
         *-------------------------------------------------*/
        drv->initialized = true;

        drv->state = DRV_STATE_NOT_READY_TO_SWITCH_ON;

        // drv->enable_requested = true;

        drv->act_mode = DRV_MODE_NONE;

        drv->req_mode = DRV_MODE_NONE;

        /*-------------------------------------------------
         * 注册
         *-------------------------------------------------*/
        hal_retcode_t ret = Hal_Driver_Register(drv);

        if (ret != HAL_DRIVER_SUCCESS)
        {
            printf("(ret != HAL_DRIVER_SUCCESS) err\n");
            return ret;
        }
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 注册驱动器
 *=========================================================*/
hal_retcode_t Hal_Driver_Register(hal_driver_t *drv)
{
    if (!dri_manager.initialized || drv == NULL)
    {
        printf("Hal_Driver_Register 驱动器管理器未初始化 \n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    if (dri_manager.count >= MAX_DRIVERS)
    {
        return HAL_DRIVER_ERR_MAX_NUMB_OVER;
    }

    /* 防止重复注册 */
    for (uint8_t i = 0; i < dri_manager.count; i++)
    {
        if (dri_manager.drivers[i] == drv)
        {
            return HAL_DRIVER_SUCCESS;
        }
    }

    dri_manager.drivers[dri_manager.count] = drv;

    dri_manager.count++;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 设置目标位置
 *=========================================================*/
hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t *drv, int32_t target_pos)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    drv->target_pos = target_pos;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 设置目标速度
 *=========================================================*/
hal_retcode_t Hal_Driver_Set_Target_Vel(hal_driver_t *drv, int32_t target_vel)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    drv->target_vel = target_vel;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 设置目标转矩
 *=========================================================*/
hal_retcode_t Hal_Driver_Set_Target_Tor(hal_driver_t *drv, int16_t target_torque)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    drv->target_torque = target_torque;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 获取实际位置
 *=========================================================*/
hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t *drv, int32_t *actual_pos)
{
    if (drv == NULL || actual_pos == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    *actual_pos = drv->actual_pos;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 获取实际速度
 *=========================================================*/
hal_retcode_t Hal_Driver_Get_Actual_Vel(hal_driver_t *drv, int32_t *actual_vel)
{
    if (drv == NULL || actual_vel == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    *actual_vel = drv->actual_vel;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 获取实际转矩
 *=========================================================*/
hal_retcode_t Hal_Driver_Get_Actual_Tor(hal_driver_t *drv, int16_t *actual_torque)
{
    if (drv == NULL || actual_torque == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    *actual_torque = drv->actual_torque;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 获取驱动器数量
 *=========================================================*/
uint8_t Hal_Driver_Get_Count(void) 
{ 
    return dri_manager.count; 
}

/*=========================================================
 * 判断驱动器是否初始化
 *=========================================================*/
bool Hal_Driver_Is_Initialized(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return false;
    }

    return drv->initialized;
}

/*=========================================================
 * 根据逻辑轴ID获取驱动器
 *=========================================================*/
hal_driver_t *Hal_Driver_Get_By_Axis_Id(uint16_t axis_id)
{
    for (uint8_t i = 0; i < dri_manager.count; i++)
    {
        hal_driver_t *drv = dri_manager.drivers[i];

        if (drv != NULL && drv->axis_id == axis_id)
        {
            return drv;
        }
    }

    return NULL;
}

/*=========================================================
 * HAL周期更新：read读取反馈PDO，write按实际模式写入目标PDO。
 * 控制字6040和工作模式6060由driver.c负责。
 *=========================================================*/
hal_retcode_t Hal_Driver_Update(hal_driver_t *drv, bool read, bool write)
{
    if (drv == NULL || !drv->initialized)
    {
        printf("(drv == NULL || !drv->initialized) err\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

#ifndef DEBUG_MODE_ENABLED

    /*=====================================================
     * 1. 读取反馈PDO
     *=====================================================*/
    if (read)
    {
        hal_retcode_t ret;

        ret = Ethercat_Driver_Read_Feedback(drv);

        if (ret != HAL_DRIVER_SUCCESS)
        {
            printf("ret = Ethercat_Driver_Read_Feedback(drv); err\n");
            return ret;
        }
    }

    /*=====================================================
     * 2. 写目标PDO
     *=====================================================*/
    if (write)
    {
        hal_retcode_t ret;

        /*
         * 驱动器处于 Fault 时，仍需由 Master_Send() 发送周期帧，
         * 但不要再用上层缓存刷新 607A/60FF 等目标 PDO。
         * Fault Reset 由 Driver_Fault_Reset() 单独写 6040，不受此判断影响。
         */
        if (drv->state == DRV_STATE_FAULT)
        {
            return HAL_DRIVER_SUCCESS;
        }

        switch (drv->act_mode)
        {
        /*-------------------------------------------------
         * CSP：周期同步位置
         *
         * 6060 = 0x08
         * 目标位置 = 607A
         *-------------------------------------------------*/
        case DRV_MODE_CSP:

            ret = Ethercat_Driver_Write_Target_Position(drv, drv->target_pos);

            if (ret != HAL_DRIVER_SUCCESS)
            {
                return ret;
            }

            break;

        /*-------------------------------------------------
         * PP：轮廓位置
         *
         * 6060 = 0x01
         * 目标位置 = 607A
         *-------------------------------------------------*/
        case DRV_MODE_PP:

            ret = Ethercat_Driver_Write_Target_Position(drv, drv->target_pos);

            if (ret != HAL_DRIVER_SUCCESS)
            {
                return ret;
            }

            break;

        /*-------------------------------------------------
         * CSV：周期同步速度
         *
         * 6060 = 0x09
         * 目标速度 = 60FF
         *-------------------------------------------------*/
        case DRV_MODE_CSV:

            ret = Ethercat_Driver_Write_Target_Velocity(drv, drv->target_vel);

            if (ret != HAL_DRIVER_SUCCESS)
            {
                return ret;
            }

            break;

        /*-------------------------------------------------
         * PV：轮廓速度
         *
         * 6060 = 0x03
         * 目标速度 = 60FF
         *-------------------------------------------------*/
        case DRV_MODE_PV:

            ret = Ethercat_Driver_Write_Target_Velocity(drv, drv->target_vel);

            if (ret != HAL_DRIVER_SUCCESS)
            {
                return ret;
            }

            break;

        /*-------------------------------------------------
         * CST：周期同步转矩
         *
         * 6060 = 0x0A
         * 目标转矩 = 6071
         *-------------------------------------------------*/
        case DRV_MODE_CST:

            // ret = Ethercat_Driver_Write_Target_Torque(drv, drv->target_torque);

            // if (ret != HAL_DRIVER_SUCCESS)
            // {
            //     return ret;
            // }

            break;

        /*-------------------------------------------------
         * PT：轮廓转矩
         *
         * 6060 = 0x04
         * 目标转矩 = 6071
         *-------------------------------------------------*/
        case DRV_MODE_PT:

            // ret = Ethercat_Driver_Write_Target_Torque(drv, drv->target_torque);

            // if (ret != HAL_DRIVER_SUCCESS)
            // {
            //     return ret;
            // }

            break;

        /*-------------------------------------------------
         * HOMING
         *
         * 6060 = 0x06
         *
         * 回零模式下这里暂时不写普通目标位置/速度/转矩。
         * 回零动作主要由：
         *
         * 6098 Homing Method
         * 6099 Homing Speeds
         * 609A Homing Acceleration
         * 6040 Controlword
         *
         * 等对象控制。
         *-------------------------------------------------*/
        case DRV_MODE_HOMING:
            break;

        /*-------------------------------------------------
         * NONE
         *
         * 尚未确定实际工作模式，不写目标PDO。
         *-------------------------------------------------*/
        case DRV_MODE_NONE:
            break;

        default:

            return HAL_DRIVER_ERR_INVALID_PARAM;
        }
    }
#else
    /*
     * DEBUG模式下不进行实际EtherCAT PDO读写。
     */
    (void)read;
    (void)write;

#endif

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 更新所有驱动器
 *=========================================================*/
hal_retcode_t Hal_Driver_Update_All(bool read, bool write)
{
    if (!dri_manager.initialized)
    {
        printf("Hal_Driver_Update_All 驱动器管理器未初始化 \n");
        return HAL_DRIVER_ERROR;
    }

    for (uint8_t i = 0; i < dri_manager.count; i++)
    {
        hal_driver_t *drv = dri_manager.drivers[i];

        if (drv == NULL)
        {
            continue;
        }

        hal_retcode_t ret = Hal_Driver_Update(drv, read, write);

        if (ret != HAL_DRIVER_SUCCESS)
        {
            return ret;
        }
    }

    return HAL_DRIVER_SUCCESS;
}
