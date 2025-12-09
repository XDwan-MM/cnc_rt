#ifndef HAL_DRIVER_H
#define HAL_DRIVER_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "libGREEMASTER/GreeMasterAPI.h"

// 常量定义
#define MAX_DRIVERS 3
#define AXES_NUMS 8

// 返回值定义
typedef enum {
    HAL_DRIVER_SUCCESS = 0,
    HAL_DRIVER_ERR_INVALID_PARAM, //传递给函数的参数无效或不合法
    HAL_DRIVER_ERR_DRIVER_NOT_FOUND,  //指定的轴ID对应的驱动器不存在或未注册
    HAL_DRIVER_ERR_MAX_NUMB_OVER,   //达到最大驱动器数量
    HAL_DRIVER_ERROR = -1
    // ... 更多错误码
} hal_retcode_t;

// 驱动器工作模式枚举
typedef enum {
    DRV_MODE_DISABLED = 0,
    DRV_MODE_CSP,    // 周期性同步位置模式
    DRV_MODE_CSV,    // 周期性同步速度模式
    DRV_MODE_CST,    // 周期性同步转矩模式
    DRV_MODE_PP,     // 轮廓位置模式（脉冲方向）
    // ... 其他模式
} drv_mode_t;

// 驱动器状态枚举
typedef enum {
    DRV_STATE_NOT_READY = 0,
    DRV_STATE_SWITCHED_ON,
    DRV_STATE_OPERATION_ENABLED,
    DRV_STATE_FAULT,
    // ... 其他状态
} drv_state_t;

// 抽象驱动器数据结构体
typedef struct {
    // --- 标识信息 ---
    char            drv_name[32];   // 驱动器名称
    uint16_t        axis_id;
    bool            initialized;
    // --- 核心状态数据 (从驱动器读取) ---
    drv_state_t     state;          // 状态字
    drv_mode_t      mode;           // 模式字
    int32_t         actual_pos;     // 实际位置
    int32_t         actual_vel;     // 实际速度
    int16_t         actual_torque;  // 实际转矩

    // --- 核心控制数据 (写入驱动器) ---
    int32_t         target_pos;     // 目标位置
    int32_t         target_vel;     // 目标速度
    int16_t         target_torque;  // 目标转矩

    // ... 其他参数

} hal_driver_t;

extern hal_driver_t x_axis_driver;
extern hal_driver_t y_axis_driver;
extern hal_driver_t z_axis_driver;
extern TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
extern TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
extern RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式
extern RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
extern RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置


// 公共API
hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t* drv, int32_t target_pos);
//hal_retcode_t Hal_Driver_Set_Target_Vel(hal_driver_t* drv, int32_t target_vel);
//hal_retcode_t Hal_Driver_Set_Target_Tor(hal_driver_t* drv, int16_t target_tor);
hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t* drv, int32_t* actual_pos);
//hal_retcode_t Hal_Driver_Get_Actual_Vel(hal_driver_t* drv, int32_t* actual_vel);
//hal_retcode_t Hal_Driver_Get_Actual_Tor(hal_driver_t* drv, int16_t* actual_tor);

// 周期性数据交换函数
hal_retcode_t Hal_Driver_Update(hal_driver_t* drv, bool read, bool write);


// 驱动器管理器函数
hal_retcode_t Hal_Driver_Manager_Init(void);
hal_retcode_t Hal_Driver_Array_Init(void);
hal_retcode_t Hal_Driver_Update_All(bool read, bool write);
hal_retcode_t Hal_Driver_Register(hal_driver_t* drv);


uint8_t Hal_Driver_Get_Count(void);
bool Hal_Driver_Is_Initialized(hal_driver_t* drv);

#endif
