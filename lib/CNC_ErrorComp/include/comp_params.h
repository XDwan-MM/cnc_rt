#ifndef RT_PAR_H
#define RT_PAR_H

/**
 * @file comp_params.h
 * @brief 误差补偿参数结构体
 *
 * CompParams 按 PAR_CMP_* 枚举顺序扁平排列，字段命名与 gncdatadef.h
 * 中的枚举常量精确对应，便于与实时控制器共享内存 (par_acmp[][]) 对接。
 *
 * 字段命名规则：<枚举前缀>_<枚举后缀>
 *   例：PAR_CMP_SQU1_ANG  →  squ1_ang
 */

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /* ---- 反向间隙补偿 (PAR_CMP_BL_*, 索引 0~4) ---- */
    int bl_enable;
    double bl_value;
    double bl_rate;
    double bl_value2;
    double bl_mpg_value;

    /* ---- 热误差补偿占位 (PAR_CMP_HEAT_*, 索引 5~19) ---- */
    int thermal_placeholder[15];

    /* ---- 螺距误差补偿 (PAR_CMP_PITCH_*, 索引 20~26) ---- */
    int pitch_type;
    double pitch_start;
    int pitch_num;
    double pitch_step;
    int pitch_modulo;
    double pitch_factor;
    int pitch_table;

    /* ---- 垂直度补偿第1项 (PAR_CMP_SQU1_*, 索引 30~33) ---- */
    int squ1_enable;
    int squ1_input_ax;
    double squ1_refn;
    double squ1_ang;

    /* ---- 垂直度补偿第2项 (PAR_CMP_SQU2_*, 索引 40~43) ---- */
    int squ2_enable;
    int squ2_input_ax;
    double squ2_refn;
    double squ2_ang;

    /* ---- 直线度补偿 (PAR_CMP_STRA1_*, 索引 50~57) ---- */
    int stra1_input_ax;
    int stra1_type;
    double stra1_start;
    int stra1_num;
    double stra1_step;
    int stra1_modulo;
    double stra1_factor;
    int stra1_table;
} CompParams;

#ifdef __cplusplus
}
#endif

#endif /* RT_PAR_H */
