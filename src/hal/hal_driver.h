#ifndef HAL_DRIVER_H
#define HAL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 常量定义
#define MAX_DRIVERS 5

#ifdef __cplusplus
extern "C" {
#endif

#include <GreeMasterAPI.h>

// 返回值定义（驱动器返回错误码枚举）
typedef enum {
    HAL_DRIVER_SUCCESS = 0,
    HAL_DRIVER_ERR_INVALID_PARAM, //传递给函数的参数无效或不合法
    HAL_DRIVER_ERR_DRIVER_NOT_FOUND,  //指定的轴ID对应的驱动器不存在或未注册
    HAL_DRIVER_ERR_MAX_NUMB_OVER,   //达到最大驱动器数量
    HAL_DRIVER_ERROR = -1
    // ... 更多错误码
} hal_retcode_t;

// 驱动器工作模式枚举
typedef enum
{
    DRV_MODE_NONE = 0, // 未知模式
    DRV_MODE_CSP,      // 周期性同步位置模式 往 6060 发0x08
    DRV_MODE_CSV,      // 周期性同步速度模式 往 6060 发0x09
    DRV_MODE_CST,      // 周期性同步转矩模式 往 6060 发0x0A
    DRV_MODE_PP,       // 轮廓位置模式      往 6060 发0x01
    DRV_MODE_PV,       // 轮廓速度模式      往 6060 发0x03
    DRV_MODE_PT,       // 轮廓扭矩模式      往 6060 发0x04
    DRV_MODE_HOMING,   // 回零模式          往 6060 发0x06
    // ... 其他模式
} drv_mode_t;

// 驱动器状态枚举
typedef enum {
    DRV_STATE_INIT = 0,
    DRV_STATE_NOT_READY_TO_SWITCH_ON, // 驱动器未准备好(驱动器正在初始化过程中)
    DRV_STATE_SWITCHED_ON_DISABLE,    // 驱动器初始化完成 驱动器参数可配置
    DRV_STATE_READY_TO_SWITCHED_ON,   // 驱动器已经准备好，驱动器可以设置参数，可以上主电，准备进入switch on状态
    DRV_STATE_SWITCHED_ON,            // 驱动器已经准备好，主电已上，等待驱动器打开使能
    DRV_STATE_OPERATION_ENABLED,      // 驱动器已使能，正常运行，驱动器设置的参数有效
    DRV_STATE_FAULT,                  // 驱动器故障状态，驱动显示故障码，可通过控制字6040 = 0x0080复位。（6041状态字的bit3为高：表示驱动器有错误）
    // ... 其他状态
} drv_state_t;

// 抽象驱动器数据结构体
typedef struct {
    // --- 标识信息 ---
    char            drv_name[32];   // 抽象驱动器名称
    uint16_t        axis_id;        // 抽象驱动器编号
    bool            initialized;    // 抽象驱动器是否初始化
    // --- 核心状态数据 (从实际驱动器读取) ---
    drv_state_t     state;          // 状态
    drv_mode_t      act_mode;       // 当前模式
    drv_mode_t      req_mode;       // 请求模式

    int32_t         actual_pos;     // 实际位置
    int32_t         actual_vel;     // 实际速度
    int16_t         actual_torque;  // 实际转矩

    // --- 核心控制数据 (写入实际驱动器) ---
    int32_t         target_pos;     // 目标位置
    int32_t         target_vel;     // 目标速度
    int16_t         target_torque;  // 目标转矩

    // ... 其他参数

} hal_driver_t;

// 全局驱动器管理器
typedef struct{
    hal_driver_t* drivers[MAX_DRIVERS];
    uint8_t count;
    bool initialized;
} driver_manager;


extern driver_manager dri_manager;
extern hal_driver_t spindle;
extern hal_driver_t x_axis_driver;
extern hal_driver_t y_axis_driver;
extern hal_driver_t z_axis_driver;

extern TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
extern RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式
extern TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
extern TxEntry_Unit CurVel[MAX_DRIVERS];          // 当前速度
extern TxEntry_Unit CurTor[MAX_DRIVERS];          // 当前转矩

extern RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
extern RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置
extern RxEntry_Unit PurVel[MAX_DRIVERS];          // 目标速度
extern RxEntry_Unit PurTor[MAX_DRIVERS];          // 目标转矩


// 公共API
hal_retcode_t Hal_Driver_Set_Target_Pos(hal_driver_t* drv, int32_t target_pos);
hal_retcode_t Hal_Driver_Set_Target_Vel(hal_driver_t* drv, int32_t target_vel);
hal_retcode_t Hal_Driver_Set_Target_Tor(hal_driver_t* drv, int16_t target_tor);
hal_retcode_t Hal_Driver_Get_Actual_Pos(hal_driver_t* drv, int32_t* actual_pos);
hal_retcode_t Hal_Driver_Get_Actual_Vel(hal_driver_t* drv, int32_t* actual_vel);
hal_retcode_t Hal_Driver_Get_Actual_Tor(hal_driver_t* drv, int16_t* actual_tor);

// 周期性数据交换函数
hal_retcode_t Hal_Driver_Update(hal_driver_t* drv, bool read, bool write);
hal_retcode_t Hal_Driver_Update_All(bool read, bool write);

// 驱动器管理器函数
hal_retcode_t Hal_Driver_Manager_Init(void);
hal_retcode_t Hal_Driver_Array_Init(void);
hal_retcode_t Hal_Driver_Register(hal_driver_t* drv);


uint8_t Hal_Driver_Get_Count(void);
bool Hal_Driver_Is_Initialized(hal_driver_t* drv);

#ifdef __cplusplus
}
#endif

#endif
