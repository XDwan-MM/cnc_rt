#ifndef COMP_COMMON_H
#define COMP_COMMON_H

#include <stdbool.h>

/**
 * @file comp_common.h
 * @brief 误差补偿模块通用定义
 *
 * 定义物理安全边界、运动判定阈值及补偿状态机枚举。
 */

/* 物理安全边界：70um = 0.070mm，各项误差补偿（螺距误差、垂直度）单次输出最大不超过70um */
#define MAX_COMP_LIMIT_PER (0.070)

/* 运动判定速度阈值 (mm/s)：低于此值视为静止，用于触发首次运动入场逻辑 */
#define VEL_THRESHOLD (1e-5)

/* 角度转换因子：将 um/m 转换为 mm/mm 的无量纲比值 (1 um/m = 1e-6 mm/mm) */
#define ANGLE_UM_PER_M_TO_RATIO (1e-6)

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
        COMP_PHASE_ACTIVE
    } CompPhase;

    /**
     * @brief 螺距补偿建表返回值枚举
     *
     * generate_pitch_comp_table() 的返回状态，负值表示错误，零表示成功。
     */
    typedef enum
    {
        PITCH_TABLE_OK               =  0, /* 建表成功 */
        PITCH_TABLE_ERR_NULL_PTR     = -1, /* 参数指针为空 (table / measurement_points / forward_errors / reverse_errors) */
        PITCH_TABLE_ERR_NUM_POINTS   = -2, /* 测量点数无效 (<=0 或超过 COMP_ARRAY_MAX_SIZE) */
        PITCH_TABLE_ERR_NOT_ASCENDING = -3 /* 测量点未按升序排列 */
    } PitchTableStatus;

#ifdef __cplusplus
}
#endif

#endif /* COMP_COMMON_H */
