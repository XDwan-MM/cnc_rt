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

/* 位置增量检测阈值 (mm/周期)：用于统一各模块的运动/静止判定。
 * 实际最小运动步长 ~1e-4 mm，远大于此值。静止时指令位置逐周期相同，不会误判。 */
#define POS_DELTA_THRESHOLD (1e-6)

/* 角度转换因子：将 um/m 转换为 mm/mm 的无量纲比值 (1 um/m = 1e-6 mm/mm) */
#define ANGLE_UM_PER_M_TO_RATIO (1e-6)

/* 补偿输入通用单位转换：int 原始值 (μm) → 补偿值 (mm) */
#define UNIT_UM_TO_MM (0.001)

/* 最大轴索引 (0-based)，用于 all_axis_pos[] 数组越界保护 */
#define MAX_AXIS_INDEX (2)

/* 顺序表最大支持存储的测量点数 */
#define COMP_ARRAY_MAX_SIZE 256

/* 哨兵边界点数量：数组首尾各预留1个，防止游标查找越界 */
#define COMP_BOUNDARY_POINTS 2

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
 * - LOCKED:  斜坡完成，已锁定目标，每周期直接跟随输出
 */
typedef enum
{
    COMP_PHASE_IDLE = 0,
    COMP_PHASE_RAMPING,
    COMP_PHASE_LOCKED,
    COMP_PHASE_WAIT_MOTION /* 使能变化时所有轴静止，等待任意轴运动后再斜坡 */
} CompPhase;

/**
 * @brief 操作结果返回值枚举
 *
 * 适用于所有基于顺序表的补偿模块（螺距、直线度等）的建表/操作返回值。
 */
typedef enum
{
    OP_RES_OK = 0,              /* 操作成功 */
    OP_RES_ERR_NULL_PTR,        /* 参数指针为空 */
    OP_RES_ERR_OUT_OF_RANGE,    /* 测量点数无效 (<=0 或超过 COMP_ARRAY_MAX_SIZE) */
    OP_RES_ERR_NOT_ASCENDING    /* 测量点未按升序排列 */
} OP_RES;

#ifdef __cplusplus
}
#endif

#endif