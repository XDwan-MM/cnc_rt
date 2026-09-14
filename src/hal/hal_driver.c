#include "hal_driver.h"
#include "debug_mode.h"
#include "device.h"
#include <stddef.h>
#include <stdio.h>

// 全局变量定义
static bool physical_sp = true;
static bool physical_x = true;
static bool physical_y = true;
static bool physical_z = true;

static uint16_t id_sp = 0;
static uint16_t id_x = 1;
static uint16_t id_y = 2;
static uint16_t id_z = 3;

driver_manager dri_manager;
hal_driver_t spindle;
hal_driver_t x_axis_driver;
hal_driver_t y_axis_driver;
hal_driver_t z_axis_driver;

// 初始化驱动器管理器
hal_retcode_t Hal_Driver_Manager_Init(void)
{
    for (int i = 0; i < MAX_DRIVERS; i++)
    {
        dri_manager.drivers[i] = NULL;
    }
    dri_manager.count = 0;
    dri_manager.initialized = true;
    return HAL_DRIVER_SUCCESS;
}
// 注册驱动器到管理器
hal_retcode_t Hal_Driver_Register(hal_driver_t *drv)
{
    if (!dri_manager.initialized || drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    if (dri_manager.count >= MAX_DRIVERS)
    {
        return HAL_DRIVER_ERR_MAX_NUMB_OVER;
    }

    // 检查是否已注册
    for (int i = 0; i < dri_manager.count; i++)
    {
        if (dri_manager.drivers[i] == drv)
        {
            return HAL_DRIVER_SUCCESS;
        }
    }

    // 注册新驱动器
    dri_manager.drivers[dri_manager.count] = drv;
    dri_manager.count++;
    return HAL_DRIVER_SUCCESS;
}

// 获取已注册的驱动器数量
uint8_t Hal_Driver_Get_Count(void) { return dri_manager.count; }

// 初始化所有驱动器
hal_retcode_t Hal_Driver_Array_Init(void)
{
    hal_driver_t* drivers[] = {&spindle, &x_axis_driver, &y_axis_driver, &z_axis_driver};
    const bool physical[] = {physical_sp, physical_x, physical_y, physical_z};
    const uint16_t axis_ids[] = {id_sp, id_x, id_y, id_z};
    const size_t driver_count = sizeof(drivers) / sizeof(drivers[0]);

    if (!dri_manager.initialized)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

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
            return result;
        }

        drivers[i]->axis_id = axis_ids[i];
        drivers[i]->initialized = true;
        drivers[i]->state = DRV_STATE_NOT_READY;
        drivers[i]->mode = DRV_MODE_DISABLED;
    }

    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t *drv, int32_t target_pos)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    drv->target_pos = target_pos;
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t *drv, int32_t *actual_pos)
{
    if (drv == NULL || actual_pos == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    *actual_pos = drv->actual_pos;
    return HAL_DRIVER_SUCCESS;
}
hal_retcode_t Hal_Driver_Set_Target_Vel(hal_driver_t *drv, int32_t target_vel)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    drv->target_vel = target_vel;
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Get_Actual_Vel(hal_driver_t *drv, int32_t *actual_vel)
{
    if (drv == NULL || actual_vel == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    *actual_vel = drv->actual_vel;
    return HAL_DRIVER_SUCCESS;
}
hal_retcode_t Hal_Driver_Set_Target_Tor(hal_driver_t *drv, int16_t target_torque)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    drv->target_torque = target_torque;
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Get_Actual_Tor(hal_driver_t *drv, int16_t *actual_torque)
{
    if (drv == NULL || actual_torque == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    *actual_torque = drv->actual_torque;
    return HAL_DRIVER_SUCCESS;
}
bool Hal_Driver_Is_Initialized(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return false;
    }
    return drv->initialized;
}

hal_retcode_t Hal_Driver_Update(hal_driver_t *drv, bool read, bool write)
{
    if (drv == NULL)
        return HAL_DRIVER_ERR_INVALID_PARAM;

#ifndef DEBUG_MODE_ENABLED
    if (read)
    {
        GM_TxPdoEntry_Read(g_device_data[drv->axis_id + 2].slave.act_pos, &drv->actual_pos, 4);
        GM_TxPdoEntry_Read(g_device_data[drv->axis_id + 2].slave.act_speed, &drv->actual_vel, 4);
    }
    if (write)
    {
        GM_RxPdoEntry_Write(g_device_data[drv->axis_id + 2].slave.target_pos, drv->target_pos);
        GM_RxPdoEntry_Write(g_device_data[drv->axis_id + 2].slave.target_speed, drv->target_vel);
    }
#else
    // 调试模式：数据由 master_sim.c 中的 simulate_hal_data() 更新
    (void)read;
    (void)write;
#endif
    return HAL_DRIVER_SUCCESS;
}

// 更新所有已注册的驱动器
hal_retcode_t Hal_Driver_Update_All(bool read, bool write)
{
    if (!dri_manager.initialized)
    {
        return HAL_DRIVER_ERROR;
    }
    hal_retcode_t result = HAL_DRIVER_SUCCESS;
    for (int i = 0; i < dri_manager.count; i++)
    {
        hal_driver_t *drv = dri_manager.drivers[i];
        if (drv != NULL)
        {
            result = Hal_Driver_Update(drv, read, write);
        }
        else
        {
            printf("--- driver no find ----\n");
        }
    }
    return result;
}
