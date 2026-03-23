#include "hal_io.h"
#include "debug_mode.h"
#include <stddef.h>
#include <stdio.h>

// 全局变量定义
hal_io_module_t hal_io_module[MAX_IO_MODULES];
uint8_t IO_MODULES_NUMB = 1;

// 全局IO管理器
static struct {
    hal_io_module_t* modules[MAX_IO_MODULES];
    uint8_t count;  // 当前加载的模块数量
    bool initialized;
} io_manager;

// 初始化驱动器管理器
hal_io_retcode_t Hal_Io_Manager_Init(void)
{
    for (int i = 0; i < IO_MODULES_NUMB; i++)
    {
        io_manager.modules[i] = NULL;
    }
    io_manager.count = 0;
    io_manager.initialized = true;
    return HAL_IO_SUCCESS;
}
// 注册驱动器到管理器
hal_io_retcode_t Hal_Io_Register(hal_io_module_t* iomod)
{
    if (!io_manager.initialized || iomod == NULL)
    {
        return HAL_IO_ERR_INVALID_PARAM;
    }

    if (io_manager.count >= MAX_IO_MODULES)
    {
        return HAL_IO_ERR_MAX_NUMB_OVER;
    }

    // 检查是否已注册
    for (int i = 0; i < io_manager.count; i++)
    {
        if (io_manager.modules[i] == iomod)
        {
            return HAL_IO_SUCCESS;
        }
    }

    // 注册新驱动器
    io_manager.modules[io_manager.count] = iomod;
    io_manager.count++;
    return HAL_IO_SUCCESS;
}


// 初始化IO模块
hal_io_retcode_t Hal_Io_Array_Init(void)
{
    for (uint16_t i = 0; i < IO_MODULES_NUMB; i++)
    {
        hal_io_module[io_manager.count].module_id = i;
        hal_io_module[io_manager.count].initialized = true;
        hal_io_module[io_manager.count].di_status = 0;
        hal_io_module[io_manager.count].do_status = 0;
        io_manager.count++;
    }
    return HAL_IO_SUCCESS;
}


// IO系统更新函数 - 核心函数
hal_io_retcode_t Hal_Io_Update(hal_io_module_t* iomod, bool read, bool write)
{
    if (iomod == NULL)
    {
        return HAL_IO_ERR_INVALID_PARAM;
    }
#ifndef DEBUG_MODE_ENABLED
    if (read)
    {
        uint16_t high = 0;
        uint16_t low = 0;
        GM_TxPdoEntry_Read(DI0[iomod->module_id], &low, 2);
        GM_TxPdoEntry_Read(DI1[iomod->module_id], &high, 2);
        iomod->di_status = ((uint32_t)high << 16) | (uint32_t)low;
    }
    if (write)
    {
        uint16_t high = 0;
        uint16_t low = 0;
        high = (iomod->do_status) >> 16 & 0xFFFF;
        low = iomod->do_status & 0xFFFF;
        GM_RxPdoEntry_Write(DO0[iomod->module_id], low);
        GM_RxPdoEntry_Write(DO1[iomod->module_id], high);
    }
#else
    // 调试模式：IO数据由模拟函数更新
    (void)read;
    (void)write;
#endif
    return HAL_IO_SUCCESS;
}
hal_io_retcode_t Hal_Io_Update_All(bool read, bool write)
{
    hal_io_retcode_t result = HAL_IO_SUCCESS;
    for (int i = 0; i < io_manager.count; i++)
    {
        hal_io_module_t* iomod = &hal_io_module[i];
        if (iomod != NULL)
        {
            result = Hal_Io_Update(iomod, read, write);
        }
        else
        {
            printf("[start] --- io no find ----\n");
        }
    }
    return result;
}

hal_io_retcode_t Hal_Io_Read_Di(hal_io_module_t* iomod,uint32_t* new_inputs)
{
    if(iomod->initialized != true || new_inputs == NULL)
    {
        return HAL_IO_ERR_DRIVER_NOT_FOUND;
    }
    *new_inputs = iomod->di_status;
    return HAL_IO_SUCCESS;
}
hal_io_retcode_t Hal_Io_Write_Do(hal_io_module_t* iomod,uint32_t new_outputs)
{
    if (iomod->initialized != true)
    {
        return HAL_IO_ERR_DRIVER_NOT_FOUND;
    }
    iomod->do_status = new_outputs;
    return HAL_IO_SUCCESS;
}
