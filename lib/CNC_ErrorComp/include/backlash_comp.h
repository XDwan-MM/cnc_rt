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
    double current_output;    /* 当前实际输出的补偿量 (mm) */
    double last_vel_sign;     /* 上周期速度符号: +1.0 / 0.0 / -1.0 */
    int last_backlash_enable; /* 上周期使能状态，用于检测参数切换 */
    bool has_moved;           /* 上电后是否已经发生过运动 */
    CompPhase phase;          /* 状态机当前阶段: IDLE / RAMPING / ACTIVE */
} BacklashCompState;

/* --- 公开接口 --- */

/**
 * @brief 初始化反向间隙补偿状态
 * @param state 补偿状态实例指针
 */
void backlash_comp_init(BacklashCompState *state);

/**
 * @brief 实时计算反向间隙补偿输出
 *
 * 根据速度方向施加反向间隙补偿值。补偿值按 bl_rate 控制步进速率平滑过渡，
 * 方向切换时自动进入 RAMPING 阶段。
 *
 * @param par_cmp  当前轴的补偿参数指针
 * @param state    对应的补偿状态指针
 * @param vel_cmd  当前轴的速度指令 (mm/s)，符号决定补偿方向
 * @param is_rapid 是否快移 (G00)，仅 bl_enable=2 时生效:
 *                 0=切削进给→使用 bl_value, 1=快移→使用 bl_value2
 * @return         补偿输出值 (mm)，范围 ±MAX_COMP_LIMIT_PER
 */
double calculate_backlash_compensation(const CompParams *par_cmp,
                                       BacklashCompState *state,
                                       double vel_cmd,
                                       int is_rapid);

#ifdef __cplusplus
}
#endif

#endif /* BACKLASH_COMP_H */
