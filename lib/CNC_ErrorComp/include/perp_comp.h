#ifndef PERP_COMP_H
#define PERP_COMP_H

#include "comp_common.h"
#include "comp_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 垂直度补偿状态结构体
 * 每一个补偿轴都应拥有一个独立的 PerpCompState 实例
 */
typedef struct
{
    double current_output; /* 当前实际输出的总补偿量 (mm) */
    double step_limit;     /* 平滑切换时的步进限制 (mm/周期) */

    CompPhase phase; /* 状态机当前阶段: IDLE / RAMPING / LOCKED / WAIT_MOTION */

    double last_axis_pos[3]; /* 上周期各轴位置快照，用于检测各轴是否运动 */
    bool   pos_initialized;  /* 惰性初始化标志：首周期用当前位填充 last_axis_pos */
} PerpCompState;

/* --- 公开接口 --- */

/**
 * @brief 初始化垂直度补偿状态结构体
 * @param state 补偿状态实例指针
 * @param step_limit 步进限制值 (如 0.0005)
 */
void perp_comp_init(PerpCompState *state, double step_limit);

/**
 * @brief 垂直度补偿主计算接口
 *
 * 运动检测完全基于逐周期位置增量，不依赖外部速度信号。
 * 首周期惰性初始化 last_axis_pos 后正式启用。
 *
 * @param par_cmp      当前补偿轴的参数指针
 * @param state        对应的补偿状态指针
 * @param cur_ax_idx   当前补偿轴的索引 (0:X, 1:Y, 2:Z)
 * @param all_axis_pos 所有轴实时坐标数组 (mm)
 * @return             补偿输出值 (mm)，范围 ±MAX_COMP_LIMIT_PER
 */
double calcul_perp_comp_val(const CompParams *par_cmp,
                                        PerpCompState *state,
                                        int cur_ax_idx,
                                        const double *all_axis_pos);

/**
 * @brief 通知垂直度补偿模块：补偿参数已被控制器同步更新
 *
 * 控制器在 Sync_LibCompPara() 之后调用。
 * 若补偿正在运行中（RAMPING/LOCKED），触发 RAMPING 平滑过渡到新 target；
 * 若处于 IDLE，无需处理——下次运动门控自然入场。
 *
 * @param state 补偿状态实例指针
 */
void perp_comp_on_params_synced(PerpCompState *state);

#ifdef __cplusplus
}
#endif

#endif /* PERP_COMP_H */
