#ifndef COMP_COMMON_H
#define COMP_COMMON_H

#include <stdbool.h>

/**
 * @file comp_common.h
 * @brief 误差补偿模块通用定义
 *
 * 定义物理安全边界、运动判定阈值及补偿状态机枚举。
 */

/* 物理安全边界：1000um = 1mm，各项误差补偿（螺距误差、垂直度）单次输出最大不超过1mm */
#define MAX_COMP_LIMIT_PER (1.0)

/* 运动判定速度阈值 (mm/s)：低于此值视为静止，用于触发首次运动入场逻辑 */
#define VEL_THRESHOLD (1e-5)

/* 位置增量检测阈值 (mm/周期)：1e-5 mm/s × 1ms = 1e-8 mm。
 * 实际最小运动步长 ~1e-4 mm，远大于此值。静止时指令位置逐周期相同，不会误判。 */
#define POS_DELTA_THRESHOLD (1e-5)

/* 角度转换因子：将 um/m 转换为 mm/mm 的无量纲比值 (1 um/m = 1e-6 mm/mm) */
#define ANGLE_UM_PER_M_TO_RATIO (1e-6)

/* 螺距补偿输入单位转换：int16_t 原始值 (μm) → 补偿值 (mm) */
#define PITCH_COMP_UM_TO_MM (0.001)

/* 最大轴索引 (0-based)，用于 all_axis_pos[] 数组越界保护 */
#define MAX_AXIS_INDEX (2)

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 补偿状态机阶段枚举
 *
 * 用于处理补偿开启、关闭及参数变更时的平滑过渡：
 * - IDLE:    补偿关闭或上电未运动，输出恒为 0
 * - RAMPING: 入场、退场或类型切换时的斜坡过渡
 * - ACTIVE:  正常加工中，实时叠加理想补偿值
 */
typedef enum
{
    COMP_PHASE_IDLE = 0,
    COMP_PHASE_RAMPING,
    COMP_PHASE_ACTIVE,
    COMP_PHASE_WAIT_MOTION   /* 使能变化时所有轴静止，等待任意轴运动后再斜坡 */
} CompPhase;

/**
 * @brief 螺距补偿建表返回值枚举
 *
 * generate_pitch_comp_table() 的返回状态，负值表示错误，零表示成功。
 */
typedef enum
{
    PITCH_TABLE_OK = 0,                /* 建表成功 */
    PITCH_TABLE_ERR_NULL_PTR = -1,     /* 参数指针为空 (table / meas_pos_arr_mm / fwd_comp_arr_um / rev_comp_arr_um) */
    PITCH_TABLE_ERR_NUM_POINTS = -2,   /* 测量点数无效 (<=0 或超过 COMP_ARRAY_MAX_SIZE) */
    PITCH_TABLE_ERR_NOT_ASCENDING = -3 /* 测量点未按升序排列 */
} PitchTableStatus;

#ifdef __cplusplus
}
#endif

#endif