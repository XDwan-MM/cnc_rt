#include "hal_driver.h"

TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式
RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置

// 全局变量定义
static bool physical_axis_x = true;
static bool physical_axis_y = true;
static bool physical_axis_z = true;

static uint16_t x_id = 0;
static uint16_t y_id = 1;
static uint16_t z_id = 2;

hal_driver_t x_axis_driver;
hal_driver_t y_axis_driver;
hal_driver_t z_axis_driver;

// 全局驱动器管理器
static struct {
    hal_driver_t* drivers[MAX_DRIVERS];
    uint8_t count;
    bool initialized;
} driver_manager;

// 初始化驱动器管理器
hal_retcode_t Hal_Driver_Manager_Init(void)
{
    for (int i = 0; i < MAX_DRIVERS; i++)
    {
        driver_manager.drivers[i] = NULL;
    }
    driver_manager.count = 0;
    driver_manager.initialized = true;
    return HAL_DRIVER_SUCCESS;
}
// 注册驱动器到管理器
hal_retcode_t Hal_Driver_Register(hal_driver_t* drv)
{
    if (!driver_manager.initialized || drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    if (driver_manager.count >= MAX_DRIVERS)
    {
        return HAL_DRIVER_ERR_MAX_NUMB_OVER;
    }

    // 检查是否已注册
    for (int i = 0; i < driver_manager.count; i++)
    {
        if (driver_manager.drivers[i] == drv)
        {
            return HAL_DRIVER_SUCCESS;
        }
    }

    // 注册新驱动器
    driver_manager.drivers[driver_manager.count] = drv;
    driver_manager.count++;
    return HAL_DRIVER_SUCCESS;
}

// 获取已注册的驱动器数量
uint8_t Hal_Driver_Get_Count(void)
{
    return driver_manager.count;
}

// 初始化驱动器
hal_retcode_t Hal_Driver_Array_Init(void)
{
    for (int i = 0; i < AXES_NUMS; i++)
    {
        if(physical_axis_x)
        {
            Hal_Driver_Register(&x_axis_driver);
            x_axis_driver.axis_id = x_id;//xml对应值
            x_axis_driver.initialized = true;
        }
        if(physical_axis_y)
        {
            Hal_Driver_Register(&y_axis_driver);
            y_axis_driver.axis_id = y_id;//xml对应值
            y_axis_driver.initialized = true;
        }
        if(physical_axis_z)
        {
            Hal_Driver_Register(&z_axis_driver);
            z_axis_driver.axis_id = z_id;//xml对应值
            z_axis_driver.initialized = true;
        }
    }
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t* drv, int32_t target_pos)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    drv->target_pos = target_pos;
    return HAL_DRIVER_SUCCESS;
}

hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t* drv, int32_t* actual_pos)
{
    if (drv == NULL || actual_pos == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }
    *actual_pos = drv->actual_pos;
    return HAL_DRIVER_SUCCESS;
}

bool Hal_Driver_Is_Initialized(hal_driver_t* drv)
{
    if(drv ==NULL)
    {
        return false;
    }
    return drv->initialized;
}

hal_retcode_t Hal_Driver_Update(hal_driver_t* drv, bool read, bool write)
{
    if (drv == NULL)
        return HAL_DRIVER_ERR_INVALID_PARAM;

    if (read)
    {
        // 这里应该从硬件读取实际数据
        GM_TxPdoEntry_Read(CurPos[drv->axis_id], &drv->actual_pos, 4);
    }
    if (write && drv->target_pos != 0)
    {
        // 这里应该向硬件写入控制数据
//        if((drv->actual_pos == 0) || (drv->target_pos == 0))
//        {
//            printf("-----%d %d------\n",drv->target_pos,drv->actual_pos);
//        }
        GM_RxPdoEntry_Write(PurPos[drv->axis_id], drv->target_pos);
    }
    return HAL_DRIVER_SUCCESS;
}

// 更新所有已注册的驱动器
hal_retcode_t Hal_Driver_Update_All(bool read, bool write)
{
    if (!driver_manager.initialized)
    {
        return HAL_DRIVER_ERROR;
    }
    hal_retcode_t result = HAL_DRIVER_SUCCESS;
    for (int i = 0; i < driver_manager.count; i++)
    {
        hal_driver_t* drv = driver_manager.drivers[i];
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


