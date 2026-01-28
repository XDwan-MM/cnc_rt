#ifndef HAL_IO_H
#define HAL_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "GreeMasterAPI.h"

// 常量定义
#define MAX_IO_MODULES 4

// 返回值定义
typedef enum {
    HAL_IO_SUCCESS = 0,
    HAL_IO_ERR_INVALID_PARAM, //传递给函数的参数无效或不合法
    HAL_IO_ERR_DRIVER_NOT_FOUND,  //指定的轴ID对应的IO不存在或未注册
    HAL_IO_ERR_MAX_NUMB_OVER,   //达到最大IO数量
    // ... 更多错误码
} hal_io_retcode_t;

// IO类型枚举
typedef enum {
    IO_TYPE_DI = 0,
    IO_TYPE_DO,
    IO_TYPE_AI,
    IO_TYPE_AO,
} io_type_t;

typedef struct
{
    uint16_t module_id;
    io_type_t module_type;
    bool initialized;
    uint32_t di_status;
    uint32_t do_status;
} hal_io_module_t;

extern uint8_t IO_MODULES_NUMB;
extern hal_io_module_t hal_io_module[MAX_IO_MODULES];
extern TxEntry_Unit DI0[MAX_IO_MODULES];
extern TxEntry_Unit DI1[MAX_IO_MODULES];
extern RxEntry_Unit DO0[MAX_IO_MODULES];
extern RxEntry_Unit DO1[MAX_IO_MODULES];

// 公共API
hal_io_retcode_t Hal_Io_Read_Di(hal_io_module_t* iomod,uint32_t* new_inputs);
hal_io_retcode_t Hal_Io_Write_Do(hal_io_module_t* iomod,uint32_t new_outputs);

// 周期性数据交换函数
hal_io_retcode_t Hal_Io_Update(hal_io_module_t* iomod, bool read, bool write);

// io管理器函数
hal_io_retcode_t Hal_Io_Manager_Init(void);
hal_io_retcode_t Hal_Io_Register(hal_io_module_t* iomod);
hal_io_retcode_t Hal_Io_Array_Init(void);
hal_io_retcode_t Hal_Io_Update_All(bool read, bool write);

#ifdef __cplusplus
}
#endif


#endif
