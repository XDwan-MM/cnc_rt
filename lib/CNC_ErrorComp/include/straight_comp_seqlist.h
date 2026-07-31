#ifndef STRAIGHT_COMP_SEQLIST_H
#define STRAIGHT_COMP_SEQLIST_H

#include "comp_common.h"
#include "comp_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 直线度补偿表中补偿条目结构体
 *
 * 直线度补偿为单向补偿，仅需单一补偿值和斜率，
 * 与螺距补偿的双向 (fwd/rev) 结构不同。
 */
typedef struct
{
    double nominal;   /* 名义位置 */
    double comp;      /* 补偿值 (mm) */
} StraightCompEntry_t;

/**
 * @brief 直线度补偿顺序表结构体
 *
 * 与螺距补偿顺序表同构：首尾各1个哨兵点，游标支持 O(1) 摊还查表。
 */
typedef struct
{
    StraightCompEntry_t data[COMP_ARRAY_MAX_SIZE + COMP_BOUNDARY_POINTS]; /* 首尾各1个哨兵点 (-DBL_MAX / +DBL_MAX) */
    int length;                                                           /* 补偿条目数量（实际测量点数）*/
    StraightCompEntry_t *cursor;                                          /* 游标指针：指向当前插值区间起始点 */
} StraightCompSeqList_t;

/* --- 顺序表基础操作 --- */

/**
 * @brief 初始化直线度补偿顺序表
 *
 * 设置左哨兵点 (nominal = -DBL_MAX)，length 清零，游标置 NULL。
 * 右哨兵点在 finalize 时根据实际测量点数确定位置。
 *
 * @param list 顺序表指针
 */
void straight_comp_seqlist_init(StraightCompSeqList_t *list);

/**
 * @brief 向顺序表追加一个测量点
 *
 * 自动校验名义位置严格升序（与上一个点比较），并检查表是否已满。
 * 仅读取 element 的 nominal、comp 字段。
 *
 * @param list    顺序表指针
 * @param element 待追加的补偿条目（补偿值单位为 mm）
 * @return OP_RES
 */
OP_RES straight_comp_seqlist_append(StraightCompSeqList_t *list, StraightCompEntry_t element);

/**
 * @brief 固化顺序表：设置右哨兵、初始化游标
 *
 * 在所有测量点通过 append 添加完毕后调用。
 * 右哨兵补偿值取最后一个测量点的值（外推斜率=0），
 * 左哨兵补偿值取第一个测量点的值。
 *
 * @param list 顺序表指针
 * @return OP_RES
 */
OP_RES straight_comp_seqlist_finalize(StraightCompSeqList_t *list);

/* --- 便捷建表接口 --- */

/**
 * @brief 生成直线度误差补偿表
 *
 * 接收 HMI 下发的 int 原始补偿数据（单位 μm），内部转换为 mm 后建表。
 *
 * @param table              输出：构建完成的补偿表
 * @param meas_pos_arr_mm    测量点名义位置数组 (mm)
 * @param comp_arr_um        补偿值数组 (单位 μm, 1 LSB = 0.001mm)
 * @param num_points         测量点数量
 * @return OP_RES 枚举值，0 (OP_RES_OK) 表示成功
 */
OP_RES generate_straight_comp_table(StraightCompSeqList_t *table,
                                    const double *meas_pos_arr_mm,
                                    const int *comp_arr_um,
                                    int num_points);

/* --- 查表接口 --- */

/**
 * @brief 查表计算直线度理想补偿目标值（含线性插值、倍率、硬限幅）
 *
 * 直线度补偿的查表基于参考轴位置 (cross-axis reference)，
 * 而非被补偿轴自身位置，这一点与螺距补偿不同。
 *
 * 根据补偿类型 (stra1_type) 和参考轴位置，从补偿表中查找并插值得到原始理想值，
 * 然后应用 stra1_factor 倍率并钳位到 [ -MAX_COMP_LIMIT_PER, +MAX_COMP_LIMIT_PER ]。
 *
 * 副作用：修改 table->cursor 游标位置（惰性初始化 + 逐周期定位）。
 *
 * @param par_cmp  补偿参数指针
 * @param table    补偿表指针（含游标）
 * @param ref_pos  参考轴当前位置 (mm)，来自 stra1_input_ax 指定轴
 * @return         理想目标补偿值 (mm)，type==0 或表为空时返回 0.0
 */
double lookup_straight_target(const CompParams *par_cmp,
                              StraightCompSeqList_t *table,
                              double ref_pos);

#ifdef __cplusplus
}
#endif

#endif // STRAIGHT_COMP_SEQLIST_H
