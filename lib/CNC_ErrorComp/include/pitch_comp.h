#ifndef PITCH_COMP_H
#define PITCH_COMP_H

#include "comp_common.h"
#include "rt_par.h"

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
        float fwd_trim;  /* 正向补偿值 */
        float rev_trim;  /* 负向补偿值 */
        float fwd_slope; /* 正向表当前点至下一点的补偿曲线斜率 (预计算) */
        float rev_slope; /* 负向表当前点至下一点的补偿曲线斜率 (预计算) */
    } pitch_comp_entry_t;

/**
 * @brief 螺距补偿数据表（数据结构：顺序存储的线性表）
 * 最大支持 256 个测量点，首尾各留一个边界点
 */
#define COMP_ARRAY_MAX_SIZE 256
    typedef struct
    {
        int entries;                                       /* 在数组中存有的补偿条目数量（实际元素个数）*/
        pitch_comp_entry_t *entry;                         /* 在数组中当前补偿条目指针 (类似游标) */
        pitch_comp_entry_t array[COMP_ARRAY_MAX_SIZE + 2]; /* 存储空间 (+2用于存放边界点) */
    } pitch_comp_table_t;

    /**
     * @brief 螺距补偿运行时上下文
     */
    typedef struct
    {
        double current_output; // 该轴当前实际输出的补偿量
        double step_limit;     // 平滑过渡的步进限制
        int last_type;         // 上周期补偿类型
        CompState state;       // 状态机当前阶段
        bool has_moved;        // 上电后首动判定标志
    } PitchCompRuntime;

    /* --- 接口函数 --- */

    /**
     * @brief 初始化螺距补偿运行时环境
     */
    void pitch_comp_init(PitchCompRuntime *rt, double step_limit);

    /**
     * @brief 生成螺距误差补偿表
     */
    int generate_pitch_comp_table(
        pitch_comp_table_t *table,
        const double *measurement_points,
        const float *forward_errors,
        const float *reverse_errors,
        int num_points);

    /**
     * @brief 实时计算螺距补偿输出
     */
    double calculate_pitch_compensation(
        PitchCompRuntime *rt,
        const LIB_PAR_ACMP *par,
        pitch_comp_table_t *table,
        double pos_cmd,
        double vel_cmd);

#ifdef __cplusplus
}
#endif

#endif // PITCH_COMP_H
