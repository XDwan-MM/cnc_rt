#ifndef STORED_STROKE_CHECK_H
#define STORED_STROKE_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <gncdatadef.h>
#include "datatype.h"

// 限位区域定义
typedef struct {
    double positive_limit; // 正方向边界值
    double negative_limit; // 负方向边界值
    uint8_t is_enabled;   // 该级限位是否启用
} LimitLevel;

// 每轴的限位配置
typedef struct {
    LimitLevel level1;   // 第一级：只能在内部运动
    LimitLevel level2;   // 第二级：可配置内外
    LimitLevel level3;   // 第三级：只能在外部运动
    uint8_t level2_mode; // 第二级模式：0=内部检查，1=外部检查
} AxisLimitConfig;

// 每轴的限位生效类型
typedef enum {
    NO_LIMIT_TRIGGERED = 0,
    LEVEL1_FWD_LIMIT,   // 第一级正
    LEVEL1_REV_LIMIT,   // 第一级负
    LEVEL2_FWD_LIMIT,   // 第二级正
    LEVEL2_REV_LIMIT,   // 第二级负
    LEVEL3_FWD_LIMIT,   // 第三级正
    LEVEL3_REV_LIMIT    // 第三级负
} AxisLimitType;

// 限位等级
typedef enum {
    NO_LEVEL = 0,
    LEVEL1,   // 一级
    LEVEL2,   // 二级
    LEVEL3    // 三级
} Level;

//行程限位功能应用
typedef enum {
    NO_USED = 0,
    LOOKAHEAD_CHECK_USED,  //移动前
    SOFT_CHEAK_USED,       //移动后
    ALARM_CHEAK_USED,
    RESET_CHEAK_USED
} AxisLimitUsedType;

// 运行时状态
typedef struct {
    AxisLimitConfig config;  // 指向配置数据
    AxisLimitConfig temp_config;  // 指向配置数据
    double current_position; // 当前位置
    double parser_position;  // 译码器位置
    uint8_t violation_flag;  // 违规标志位
    uint8_t active_levels;   // 当前生效的限位级别
    AXIS_TYPE axis;
    double limit_pos;
    AxisLimitType limit_type;
    int direction ;          //运动方向　　１正向  0负向 -1未报警
    bool is_first_run;
} AxisLimitRuntime;

extern AxisLimitRuntime runtime[SYS_AXES_NUM];


// 单轴限位检查主函数
uint8_t Check_Axis_Limit_Violation(AxisLimitRuntime *runtime , double pos, AxisLimitUsedType type);

void Axis_Limit_Manager_Init(void);

void Axis_Status_Update();

// 前瞻检查软限位
void lookahead_check_soft_limit(POS *target_pos);

// 自动模式下移动后检查软限位减速完成
void auto_check_soft_limit_del_done();

// 手动模式下移动后检查软限位减速完成
void manual_check_soft_limit_del_done(int axis_type);

// 预先检查插补位置是否超软限位
void check_for_soft_limit();

#ifdef __cplusplus
}
#endif

#endif
