#ifndef PITCH_COMP_H
#define PITCH_COMP_H

#include "comp_common.h"
#include "comp_params.h"
#include "pitch_comp_seqlist.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 螺距补偿状态结构体
 */
typedef struct
{
    double current_output;  // 该轴当前实际输出的补偿量
    double step_limit;      // 平滑过渡的步进限制
    double last_dir_sign;   // 上周期方向符号，由位置增量推导：+1.0正向 / -1.0负向
    double last_pos_cmd;    // 上周期指令位置 (mm)，用于运动/方向检测
    bool   pos_initialized; // 惰性初始化标志：首周期用当前pos填充last_pos_cmd
    CompPhase phase;        // 状态机当前阶段: IDLE / RAMPING / LOCKED
} PitchCompState;

/* --- 接口函数 --- */

/**
 * @brief 初始化螺距补偿运行时环境
 */
void pitch_comp_init(PitchCompState *state, double step_limit);

/**
 * @brief 实时计算螺距补偿输出
 *
 * 运动检测和方向判断完全基于逐周期位置增量 (pos_cmd - last_pos_cmd)，
 * 不依赖外部速度信号。首周期惰性初始化 last_pos_cmd 后正式启用。
 *
 * @param state   补偿状态实例指针
 * @param par_cmp 当前轴的补偿参数指针
 * @param table   螺距补偿顺序表指针
 * @param pos_cmd 当前轴指令位置 (mm)
 * @return        补偿输出值 (mm)，范围 ±MAX_COMP_LIMIT_PER
 */
double calcul_pitch_comp_val(PitchCompState *state,
                                    const CompParams *par_cmp,
                                    PitchCompSeqList_t *table,
                                    double pos_cmd);

/**
 * @brief 通知螺距补偿模块：补偿参数已被控制器同步更新
 *
 * 控制器在 Sync_LibCompPara() + generate_pitch_comp_table() 之后调用。
 * 若补偿正在运行中（RAMPING/LOCKED），触发 RAMPING 平滑过渡到新 target；
 * 若处于 IDLE，无需处理——下次运动门控自然入场。
 *
 * @param state 补偿状态实例指针
 */
void pitch_comp_on_params_synced(PitchCompState *state);

#ifdef __cplusplus
}
#endif

#endif // PITCH_COMP_H
