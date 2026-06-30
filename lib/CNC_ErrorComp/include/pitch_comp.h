#ifndef PITCH_COMP_H
#define PITCH_COMP_H

#include "comp_common.h"
#include "comp_params.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 螺距补偿表中补偿条目结构体
 * 包含名义位置、正反向误差以及预计算的斜率
 */
typedef struct
{
    double nominal;  /* 名义位置 */
    float fwd_comp;  /* 正向补偿值 */
    float rev_comp;  /* 负向补偿值 */
    float fwd_slope; /* 正向表当前点至下一点的补偿曲线斜率 (预计算) */
    float rev_slope; /* 负向表当前点至下一点的补偿曲线斜率 (预计算) */
} pitch_comp_entry_t;

/**
 * @brief 螺距补偿数据表（数据结构：顺序存储的线性表）
 * 最大支持 256 个测量点，首尾各留一个边界点
 */
#define COMP_ARRAY_MAX_SIZE 256

/* 哨兵边界点数量：数组首尾各预留 1 个，防止游标查找越界 */
#define COMP_BOUNDARY_POINTS 2

typedef struct
{
    int entries;                                                          /* 补偿条目数量（实际测量点数）*/
    pitch_comp_entry_t *entry;                                            /* 游标指针：指向当前插值区间起始点 */
    pitch_comp_entry_t array[COMP_ARRAY_MAX_SIZE + COMP_BOUNDARY_POINTS]; /* 首尾各 1 个哨兵点 (-DBL_MAX / +DBL_MAX) */
} pitch_comp_table_t;

/**
 * @brief 螺距补偿状态结构体
 */
typedef struct
{
    double current_output;  // 该轴当前实际输出的补偿量
    double step_limit;      // 平滑过渡的步进限制
    int last_type;          // 上周期补偿类型
    double last_target_val; // 上周期目标值，监控用，每周期更新
    CompPhase phase;        // 状态机当前阶段: IDLE / RAMPING / ACTIVE
    bool has_moved;         // 上电后首动判定标志
} PitchCompState;

/* --- 接口函数 --- */

/**
 * @brief 初始化螺距补偿运行时环境
 */
void pitch_comp_init(PitchCompState *state, double step_limit);

/**
 * @brief 生成螺距误差补偿表
 *
 * 接收 HMI 下发的 int 原始补偿数据（单位 μm），内部转换为 mm 后建表。
 *
 * @param table              输出：构建完成的补偿表
 * @param meas_pos_arr_mm    测量点名义位置数组 (mm)
 * @param fwd_comp_arr_um    正向补偿值数组 (单位 μm, 1 LSB = 0.001mm)
 * @param rev_comp_arr_um    负向补偿值数组 (单位 μm, 1 LSB = 0.001mm)
 * @param num_points         测量点数量
 * @return PitchTableStatus 枚举值，0 (PITCH_TABLE_OK) 表示成功，负值表示错误
 */
PitchTableStatus generate_pitch_comp_table(pitch_comp_table_t *table,
                                           const double *meas_pos_arr_mm,
                                           const int *fwd_comp_arr_um,
                                           const int *rev_comp_arr_um,
                                           int num_points);

/**
 * @brief 实时计算螺距补偿输出
 */
double calculate_pitch_compensation(PitchCompState *state,
                                    const CompParams *par_cmp,
                                    pitch_comp_table_t *table,
                                    double pos_cmd,
                                    double vel_cmd);

#ifdef __cplusplus
}
#endif

#endif // PITCH_COMP_H
