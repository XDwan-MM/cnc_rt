#ifndef PITCH_COMP_SEQLIST_H
#define PITCH_COMP_SEQLIST_H

#include "comp_common.h"
#include "comp_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 螺距补偿表中补偿条目结构体
 * 包含名义位置和正反向误差补偿值
 */
typedef struct
{
    double nominal;      /* 名义位置 (mm) */
    int fwd_comp_um;     /* 正向补偿值 (μm) */
    int rev_comp_um;     /* 负向补偿值 (μm) */
} PitchCompEntry_t;

/**
 * @brief 螺距补偿顺序表结构体
 *
 */
typedef struct
{
    PitchCompEntry_t data[COMP_ARRAY_MAX_SIZE + COMP_BOUNDARY_POINTS]; /* 首尾各1个哨兵点 (-DBL_MAX / +DBL_MAX) */
    int length;                                                        /* 补偿条目数量（实际测量点数）*/
    PitchCompEntry_t *cursor;                                          /* 游标指针：指向当前插值区间起始点 */
} PitchCompSeqList_t;

/* --- 顺序表基础操作 --- */

/**
 * @brief 初始化螺距补偿顺序表
 *
 * 设置左哨兵点 (nominal = -DBL_MAX)，length 清零，游标置 NULL。
 * 右哨兵点在 finalize 时根据实际测量点数确定位置。
 *
 * @param list 顺序表指针
 */
void pitch_comp_seqlist_init(PitchCompSeqList_t *list);

/**
 * @brief 向顺序表追加一个测量点
 *
 * 自动校验名义位置严格升序（与上一个点比较），并检查表是否已满。
 * 仅读取 element 的 nominal、fwd_comp_um、rev_comp_um 字段。
 *
 * @param list    顺序表指针
 * @param element 待追加的补偿条目（补偿值单位为 μm）
 * @return OP_RES
 */
OP_RES pitch_comp_seqlist_append(PitchCompSeqList_t *list, PitchCompEntry_t element);

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
OP_RES pitch_comp_seqlist_finalize(PitchCompSeqList_t *list);

/* --- 便捷建表接口 --- */

/**
 * @brief 生成螺距误差补偿表
 *
 * 接收 HMI 下发的 int 原始补偿数据（单位 μm），以 int 精确存储于表中。
 * 补偿值在查表插值（μm 空间）后，由 lookup_pitch_target() 统一转换为 mm。
 *
 * @param table              输出：构建完成的补偿表
 * @param meas_pos_arr_mm    测量点名义位置数组 (mm)
 * @param fwd_comp_arr_um    正向补偿值数组 (单位 μm, 1 LSB = 0.001mm)
 * @param rev_comp_arr_um    负向补偿值数组 (单位 μm, 1 LSB = 0.001mm)
 * @param num_points         测量点数量
 * @return OP_RES 枚举值，0 (OP_RES_OK) 表示成功
 */
OP_RES generate_pitch_comp_table(PitchCompSeqList_t *table,
                                           const double *meas_pos_arr_mm,
                                           const int *fwd_comp_arr_um,
                                           const int *rev_comp_arr_um,
                                           int num_points);

/**
 * @brief 查表计算理想补偿目标值（游标定位 + 有理数插值 + μm→mm 转换）
 *
 * 根据补偿类型 (type) 和轴位置/方向，从补偿表中查找并做有理数线性插值，
 * 返回 mm 单位的补偿目标值。不做倍率缩放和限幅——由调用方 calcul_pitch_comp_val() 负责。
 *
 * 副作用：修改 table->cursor 游标位置（惰性初始化 + 逐周期定位）。
 *
 * @param par_cmp  补偿参数指针
 * @param table    补偿表指针（含游标）
 * @param pos_cmd  当前轴指令位置 (mm)
 * @param dir_sign 方向符号：+1.0=正向查fwd表，-1.0=反向查rev表，0.0=静止(返回0.0)
 * @return         理想目标补偿值 (mm)，type==0 或表为空时返回 0.0
 */
double lookup_pitch_target(const CompParams *par_cmp, PitchCompSeqList_t *table, double pos_cmd, double dir_sign);

#ifdef __cplusplus
}
#endif

#endif // PITCH_COMP_SEQLIST_H
