#ifndef STRAIGHTNESS_COMP_H
#define STRAIGHTNESS_COMP_H

#include "comp_common.h"
#include "comp_params.h"
#include "straightness_comp_seqlist.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 单组直线度补偿运行时状态
 */
typedef struct
{
    double current_output;                 /* 当前实际输出的补偿量 (mm) */
    double step_limit;                     /* 平滑过渡的步进限制 (mm/周期) */
    double last_axis_pos[MAX_AXIS_INDEX + 1]; /* 上周期各轴位置快照 (mm) */
    double last_dir_sign;                  /* 上周期移动方向: +1.0/-1.0 */
    bool pos_initialized;                  /* 是否已捕获初始位置 */
    CompPhase phase;                       /* IDLE / RAMPING / LOCKED */
} StraightnessCompState;

/**
 * @brief 初始化单组直线度补偿运行时状态
 * @param state      状态指针
 * @param step_limit 平滑过渡时每周期最大变化量 (mm)
 */
void straightness_comp_init(StraightnessCompState *state, double step_limit);

/**
 * @brief 计算第一组直线度补偿值
 *
 * @param par_cmp      当前补偿轴参数
 * @param state        第一组独立运行时状态
 * @param cur_ax_idx   当前被补偿轴索引 (0:X, 1:Y, 2:Z)
 * @param all_axis_pos 所有轴当前位置 (mm)
 * @param table        第一组已建好的直线度补偿表
 * @return             第一组补偿输出值 (mm)
 */
double calcul_straightness1_comp_val(const CompParams *par_cmp,
                                     StraightnessCompState *state,
                                     int cur_ax_idx,
                                     const double *all_axis_pos,
                                     StraightnessCompSeqList_t *table);

/**
 * @brief 计算第二组直线度补偿值
 *
 * @param par_cmp      当前补偿轴参数
 * @param state        第二组独立运行时状态
 * @param cur_ax_idx   当前被补偿轴索引 (0:X, 1:Y, 2:Z)
 * @param all_axis_pos 所有轴当前位置 (mm)
 * @param table        第二组已建好的直线度补偿表
 * @return             第二组补偿输出值 (mm)
 */
double calcul_straightness2_comp_val(const CompParams *par_cmp,
                                     StraightnessCompState *state,
                                     int cur_ax_idx,
                                     const double *all_axis_pos,
                                     StraightnessCompSeqList_t *table);

/**
 * @brief 通知直线度补偿模块参数已同步
 *
 * 对第一组和第二组状态分别调用一次。当前实现不使用 modulo 参数，
 * 保留该参数供后续旋转轴取模补偿扩展。
 */
void straightness_comp_on_params_synced(StraightnessCompState *state);

#ifdef __cplusplus
}
#endif

#endif /* STRAIGHTNESS_COMP_H */
