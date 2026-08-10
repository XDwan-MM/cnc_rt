#ifndef COMP_PARAMS_H
#define COMP_PARAMS_H

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
    int bl_enable;       /* 0=关闭，1=统一间隙值，2=快移使用独立间隙值 */
    double bl_value;     /* 切削进给/普通模式的间隙值 (mm) */
    double bl_rate;      /* 每伺服周期最大补偿变化量 (mm)，<=0 表示瞬时到位 */
    double bl_value2;    /* bl_enable=2 时的快移间隙值 (mm) */
    double bl_mpg_value; /* 手轮模式间隙值 (mm)，当前模块暂未使用 */

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

    /* ---- 直线度补偿第一组 (PAR_CMP_STRA1_*, 索引 50~57) ---- */
    int stra1_mov_ax;          /*移动轴轴号*/
    int stra1_type;            /*补偿类型*/
    double stra1_start_pos;    /*移动轴补偿起点位置*/
    int stra1_point_cnt;       /*补偿点数*/
    double stra1_step;         /*补偿间距*/
    int stra1_modulo;          /*补偿取模使能*/
    double stra1_factor;       /*补偿倍率*/
    int stra1_table_start_idx; /*补偿表起始点参数好*/

    /* ---- 直线度补偿第二组 (PAR_CMP_STRA2_*, 索引 65~72) ---- */
    int stra2_mov_ax;          /*移动轴轴号*/
    int stra2_type;            /*补偿类型*/
    double stra2_start_pos;    /*移动轴补偿起点位置*/
    int stra2_point_cnt;       /*补偿点数*/
    double stra2_step;         /*补偿间距*/
    int stra2_modulo;          /*补偿取模使能*/
    double stra2_factor;       /*补偿倍率*/
    int stra2_table_start_idx; /*补偿表起始点参数好*/
} CompParams;

#ifdef __cplusplus
}
#endif

#endif /* COMP_PARAMS_H */
