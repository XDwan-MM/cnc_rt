#ifndef STRAIGHTNESS_COMP_SEQLIST_H
#define STRAIGHTNESS_COMP_SEQLIST_H

#include "comp_common.h"
#include "comp_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 直线度补偿表中补偿条目结构体
 *
 */
typedef struct
{
    double moving_ax_pos; /* 移动轴位置 (mm) */
    int fwd_comp_val_um;  /* 正向补偿值 (um) */
    int rev_comp_val_um;  /* 负向补偿值 (um) */
} StraightnessCompEntry_t;

/**
 * @brief 直线度补偿顺序表结构体
 *
 */
typedef struct
{
    StraightnessCompEntry_t data[COMP_ARRAY_MAX_SIZE + COMP_BOUNDARY_POINTS]; /* 首尾各1个哨兵点 (-DBL_MAX / +DBL_MAX) */
    int length;                                                               /* 补偿条目数量（实际测量点数）*/
    StraightnessCompEntry_t *cursor;                                          /* 游标指针：指向当前插值区间起始点 */
} StraightnessCompSeqList_t;

/* --- 顺序表基础操作 --- */

/**
 * @brief 初始化直线度补偿顺序表
 *
 * 设置左哨兵点 (nominal = -DBL_MAX)，length 清零，游标置 NULL。
 * 右哨兵点在 finalize 时根据实际测量点数确定位置。
 *
 * @param list 顺序表指针
 */
void straightness_comp_seqlist_init(StraightnessCompSeqList_t *list);

/**
 * @brief 向顺序表追加一个测量点
 *
 * 自动校验名义位置严格升序（与上一个点比较），并检查表是否已满。
 *
 * @param list    顺序表指针
 * @param element 待追加的补偿表条目（补偿值单位为 μm）
 * @return OP_RES
 */
OP_RES straightness_comp_seqlist_append(StraightnessCompSeqList_t *list, StraightnessCompEntry_t element);

/**
 * @brief 固化顺序表：设置右哨兵、初始化游标
 *
 * 在所有测量点通过 append 添加完毕后调用。
 * 右哨兵补偿值取最后一个测量点的值，
 * 左哨兵补偿值取第一个测量点的值。
 *
 * @param list 顺序表指针
 * @return OP_RES
 */
OP_RES straightness_comp_seqlist_finalize(StraightnessCompSeqList_t *list);

/* --- 便捷建表接口 --- */

OP_RES generate_straightness_comp_table(StraightnessCompSeqList_t *table,
                                        const double *moving_ax_pos_arr_mm,
                                        const int *fwd_comp_arr_um,
                                        const int *rev_comp_arr_um,
                                        int point_cnt);
/* --- 查表接口 --- */

/**
 * @brief 查表计算直线度理想补偿目标值（含线性插值、倍率、硬限幅）
 *
 * 直线度补偿的查表基于参考轴位置 (cross-axis reference)，
 * 而非被补偿轴自身位置，这一点与螺距补偿不同。
 *
 * 根据补偿组、补偿类型和移动轴位置，从补偿表中查找并插值得到原始理想值，
 * 然后应用对应组 factor 倍率并钳位到 [ -MAX_COMP_LIMIT_PER, +MAX_COMP_LIMIT_PER ]。
 * 区间长度使用对应组的 stra*_step 参数。
 * 当前版本不使用 stra*_modulo 执行位置回绕，表外位置由哨兵点保持边界补偿值。
 *
 * 副作用：修改 table->cursor 游标位置（惰性初始化 + 逐周期定位）。
 *
 * @param par_cmp     补偿参数指针
 * @param group_index 补偿组号，取 1 或 2
 * @param table       补偿表指针（含游标）
 * @param ref_pos     移动轴当前位置 (mm)
 * @param dir_sign    方向符号：+1.0=正向，-1.0=反向，0.0=静止
 * @return            理想目标补偿值 (mm)，无效输入或禁用时返回 0.0
 */
double find_straightness_target_comp_val(const CompParams *par_cmp,
                                         int group_index,
                                         StraightnessCompSeqList_t *table,
                                         double ref_pos,
                                         double dir_sign);

#ifdef __cplusplus
}
#endif

#endif /* STRAIGHTNESS_COMP_SEQLIST_H */
