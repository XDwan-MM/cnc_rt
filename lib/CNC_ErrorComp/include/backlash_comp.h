#ifndef BACKLASH_COMP_H
#define BACKLASH_COMP_H

#include "comp_common.h"
#include "comp_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 反向间隙补偿状态结构体
 *
 * 每个补偿轴拥有一个独立的 BacklashCompState 实例。
 * 平滑步进量由 CompParams.bl_rate 动态驱动，无需在状态中存储。
 */
typedef struct
{
    double current_output; /* 当前实际输出的补偿量 (mm) */
    double last_dir_sign;  /* 上周期运动方向: +1.0 / -1.0，未运动时为 0.0 */
    double last_pos_cmd;   /* 上周期指令位置 (mm) */
    bool pos_initialized;  /* 是否已捕获初始指令位置 */
    CompPhase phase;       /* 当前状态: IDLE / RAMPING / LOCKED */
} BacklashCompState;

/**
 * @brief 初始化反向间隙补偿状态
 * @param state 补偿状态实例指针
 */
void backlash_comp_init(BacklashCompState *state);

/**
 * @brief 实时计算反向间隙补偿输出
 *
 * 根据运动方向（由逐周期位置增量推导）施加反向间隙补偿值。
 * 补偿值按 bl_rate 控制步进速率平滑过渡，方向切换时自动进入 RAMPING 阶段。
 *
 * @param params           对应轴的补偿参数
 * @param state            对应轴的补偿状态
 * @param position_command 当前轴指令位置 (mm)，用于位置增量推导方向
 * @param is_rapid 是否快移 (G00)，仅 bl_enable=2 时生效:
 *                 0=切削进给，非0=快移
 * @return 补偿输出值 (mm)，范围 ±MAX_COMP_LIMIT_PER
 */
double calcul_backlash_comp_val(const CompParams *params,
                                BacklashCompState *state,
                                double position_command,
                                int is_rapid);

/**
 * @brief 通知反向间隙补偿模块：补偿参数已被控制器同步更新
 *
 * 控制器在 Sync_LibCompPara() 之后调用。
 * 若补偿正在运行中（RAMPING/LOCKED），触发 RAMPING 平滑过渡到新 target；
 * 若处于 IDLE，无需处理——下次运动门控自然入场。
 *
 * @param state 补偿状态实例指针
 */
void backlash_comp_on_params_synced(BacklashCompState *state);

#ifdef __cplusplus
}
#endif

#endif /* BACKLASH_COMP_H */
