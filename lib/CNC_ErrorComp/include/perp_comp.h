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

    bool last_perp1_enabled; /* 记录上一周期项1是否使能 */
    bool last_perp2_enabled; /* 记录上一周期项2是否使能 */
    bool has_moved;          /* 关键：记录上电后是否已经发生过运动 */

    CompPhase phase; /* 状态机当前阶段: IDLE / RAMPING / ACTIVE / WAIT_MOTION */

    double last_axis_pos[3]; /* 上周期各轴位置快照，用于检测非补偿轴是否运动 */
} PerpCompState;

/* --- 公开接口 --- */

/**
 * @brief 初始化垂直度补偿状态结构体
 * @param state 补偿状态实例指针
 * @param step_limit 步进限制值 (如 0.0005)
 */
void perp_comp_init(PerpCompState *state, double step_limit);

/**
 * @brief 垂直度补偿主计算接口（仿照螺距补偿逻辑，上电时轴运动了才开始平滑入场）
 * @param par_cmp 当前补偿轴的参数指针 (如 comp_axis_y)
 * @param state 对应的补偿状态指针
 * @param cur_ax_idx 当前补偿轴的索引 (0:X, 1:Y, 2:Z)
 * @param all_axis_pos 所有轴实时坐标数组
 * @param vel_cmd 当前轴的速度指令，用于判定是否开始运动
 */
double calculate_perpendicularity_total(const CompParams *par_cmp,
                                        PerpCompState *state,
                                        int cur_ax_idx,
                                        const double *all_axis_pos,
                                        double vel_cmd);

#ifdef __cplusplus
}
#endif

#endif
