/********************************************************************
 * @file        common.h
 * @brief       模拟运动控制外其他数据
 * @version     V1.00
 * @date        2025/10/15
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

#include "motiondata.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

// 更新：【报警模块】翟亚飞
// 返回值类型
typedef enum result
{
    GNC_OPERATOR_SUCCESS = 0,   // 操作成功
    GNC_OPERATOR_ERROR = 1,     // 操作失败
    GNC_OPERATOR_WARNING = 2,   // 操作警告

    GNC_CAL_SUCCESS = 100,      // 计算成功
    GNC_CAL_ERROR = 101,        // 计算失败
    GNC_PARAM_UNVALID = 102,    // 输入参数无效

    GNC_DATA_EMPTY = 200,      // 数据为空
    GNC_DATA_FULL = 201,       // 数据已满
    GNC_DATA_NULL = 202,       // 数据为NULL
}GNC_RES;


//判断参数是否合规
// #define VALIDATE_ARGUMENT(condition, arg, message)\
// 	({\
// 		if(!(condition)){\
// 			printf("Invalid argument: %s\r\nParameter name: %s\r\n", message, arg);\
// 		}\
// 	})

 //选择平面
typedef enum plane {
    PLANE_XY = 0,  //选择XY平面 G17
    PLANE_XZ = 1,  //选择XZ平面 G18
    PLANE_YZ = 2,  //选择YZ平面 G19
} PLANE;

// 位置
typedef struct pos{
    double x;
    double y;
    double z;
}POS;

// 切矢
typedef POS VECTOR;

// 加减速类型
typedef enum velplan_type{
    T_SHAPE = 0,
    S_SHAPE = 1,
    SIN_SHAPE = 2,
}VELPLAN_TYPE;

// 回零方式
typedef enum {
    ABSOLUTE_AXIS_HOMING = 0,
    RELATIVE_AXIS_HOMING,
}HOMING_TYPE;


// 机床参数
typedef struct params
{
    // 快速进给参数
    double rapid_vel;
    double rapid_acc;
    double rapid_jerk;
    double rapid_rate; // 快速进给倍率

    // 进给参数
    double cycle_time;
    double feed_vel;
    double feed_acc;
    double feed_jerk;
    double feed_rate;  // 进给倍率

    // 回零参数
    double homing_vel;
    double homing_acc;
    double homing_jerk;
    POS home_pos;  // 原点位置

    // 跟随误差
    POS static_follow_error;  // 静态跟随误差
    POS dynamic_follow_error; // 动态跟随误差

    // 主轴参数
    double spindle_rate;  // 主轴倍率

    VELPLAN_TYPE velplan_type;  // 加减速类型
    HOMING_TYPE homing_type;    // 回零类型（相对、绝对）
    int chan_num;               // 通道数
    bool is_lookahead;          // 是否开启前瞻 todo: 高速高精的功能之一
}PARAMS;

// NC 运行模式
typedef enum{
    MODE_IDLE = 0,         // 空闲
    MODE_ALARM,            // 报警
    MODE_HOMING,           // 回零
    MODE_MDI,              // MDI
    MODE_MEM,              // AUTO
    MODE_JOG,              // 手动
    MODE_MPG,              // 手轮
    MODE_RAPID,
    MODE_EDIT,
    MODE_DNC,
}NC_MODE;

// 硬件抽象层数据
typedef struct hal_data{
    POS current_pos;    // 计算的当前周期位置
    POS target_pos;     // 计算的下一周期位置
    POS feedback_pos;   // 伺服反馈的位置
    double current_vel; // 当前速度
    double target_vel;  // 目标速度
    double x_current_vel;
    double y_current_vel;
    double z_current_vel;
    double x_target_vel;
    double y_target_vel;
    double z_target_vel;
    double interp_pin; // 插补距离
}HAL_DATA;


// PLC G变量数据
typedef struct{
    // G 寄存器
    bool pause_signal;  // 暂停信号 feed_hold 进给保持
    bool reset_signal;  // 复位信号
    bool estop_signal;  // 急停信号
    bool cycle_start_signal; // 循环启动信号
    bool enable_signal;  // 电机使能信号，面板上的on/off
    bool manual_signal;  // 手动信号
    bool x_axis_homing_signal;
    bool y_axis_homing_signal;
    bool z_axis_homing_signal;
    // 状态
    NC_MODE nc_mode;    // NC运行模式
}PLC_REG;

typedef enum{
    SYS_EVENT_NONE = 0,
    SYS_EVENT_RESET,
    SYS_EVENT_RERUN,
    SYS_EVENT_PAUSE,
    SYS_EVENT_X,           // +X 正向
    SYS_EVENT_X_NEG,       // -X 负向
    SYS_EVENT_Y,           // +Y 正向
    SYS_EVENT_Y_NEG,       // -Y 负向
    SYS_EVENT_Z,           // +Z 正向
    SYS_EVENT_Z_NEG,       // -Z 负向
}SYS_EVENT;

// 系统状态
typedef struct{
    // button模式：按下激活
    SYS_EVENT nc_event;
    // toggle button：按下使能，再按失能
    bool is_estop_actived;
    bool is_slb_actived;
    // 状态
    NC_MODE target_mode;    // NC运行模式
    NC_MODE current_mode;   // NC运行模式
    // 临时状态
    bool is_switching_mode;
}SYS_DATA;

typedef enum {
    SPINDLE_STOP = 0, SPINDLE_CW, SPINDLE_CCW
} SpindleState;

extern HAL_DATA *hal; // 硬件抽象层数据
extern PARAMS *params;  // 机床参数
extern PLC_REG *plc;
extern SYS_DATA *sys;



double POS_Distance(POS* p1, POS* p2);
POS POS_VECTOR(POS* p1, POS* p2);
double POS_Norm(POS p);
double POS_Dot(POS p1,  POS p2);

#ifdef __cplusplus
}
#endif

#endif
