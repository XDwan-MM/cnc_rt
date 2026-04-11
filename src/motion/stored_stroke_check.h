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

// 每轴的限位配置
typedef enum {
    No_Limit_Triggered = 0,
    level1_fwd_limit,   // 第一级正
    level1_rev_limit,   // 第二级负
    level2_fwd_limit,   // 第二级正
    level2_rev_limit,   // 第二级负
    level3_fwd_limit,   // 第三级正
    level3_rev_limit    // 第三级负
} AxisLimitType;


// 运行时状态
typedef struct {
    AxisLimitConfig config;  // 指向配置数据
    double current_position; // 当前位置
    double parser_position;  // 译码器位置
    uint8_t violation_flag;  // 违规标志位
    uint8_t active_levels;   // 当前生效的限位级别
    AXIS_TYPE axis;
    double limit_pos;
    bool is_fwd_limit;
    bool is_rev_limit;
    AxisLimitType limit_type;
    int direction ;          //运动方向　　１正向  0负向 -1未报警
} AxisLimitRuntime;

extern AxisLimitRuntime runtime[SYS_AXES_NUM];

// 检查位置是否在限位区域内部
uint8_t Is_Inside_Limit(float position, LimitLevel limit);

// 检查位置是否在限位区域外部
uint8_t Is_Outside_Limit(float position, LimitLevel limit);

// 单轴限位检查主函数
uint8_t Check_Axis_Limit_Violation(AxisLimitRuntime runtime , double pos,bool moving);

void Axis_Limit_Manager_Init(void);

void Axis_Status_Update();

void lookahead_check_soft_limit(POS *target_pos);

#ifdef __cplusplus
}
#endif

#endif
