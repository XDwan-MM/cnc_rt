#ifndef PERP_COMP_H
#define PERP_COMP_H

#include "comp_common.h"
#include "rt_par.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 垂直度补偿运行时上下文
     * 每一个补偿轴都应拥有一个独立的 PerpCompRuntime 实例
     */
    typedef struct
    {
        double current_output; /* 当前实际输出的总补偿量 (mm) */
        double step_limit;     /* 平滑切换时的步进限制 (mm/周期) */

        bool last_perp1_enabled; /* 记录上一周期项1是否使能 */
        bool last_perp2_enabled; /* 记录上一周期项2是否使能 */
        bool has_moved;          /* 关键：记录上电后是否已经发生过运动 */

        CompState state; /* 补偿状态机 (IDLE(待机), RAMPING(平滑中), ACTIVE(实时叠加)) */
    } PerpCompRuntime;

    /* --- 公开接口 --- */

    /**
     * @brief 初始化运行时结构体
     * @param rt 运行时实例指针
     * @param step_limit 步进限制值 (如 0.0005)
     */
    void perp_comp_init(PerpCompRuntime *rt, double step_limit);

    /**
     * @brief 垂直度补偿主计算接口（仿照螺距补偿逻辑，上电时轴运动了才开始平滑入场）
     * @param par 当前补偿轴的参数指针 (如 comp_axis_y)
     * @param rt 对应的运行时上下文
     * @param cur_ax_idx 当前补偿轴的索引 (0:X, 1:Y, 2:Z)
     * @param all_axis_pos 所有轴实时坐标数组
     * @param vel_cmd 当前轴的速度指令，用于判定是否开始运动
     */
    double calculate_perpendicularity_total(
        const LIB_PAR_ACMP *par,
        PerpCompRuntime *rt,
        int cur_ax_idx,
        const double *all_axis_pos,
        double vel_cmd);

#ifdef __cplusplus
}
#endif

#endif
