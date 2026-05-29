#ifndef RT_PARH_H
#define RT_PARH_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        /* 反向间隙补偿参数 (0-4) */
        int bl_enable;       /* 0: 反向间隙补偿类型 */
        double bl_value;     /* 1: 反向间隙补偿值 */
        double bl_rate;      /* 2: 反向间隙补偿率 */
        double bl_value2;    /* 3: 快移反向间隙补偿值 */
        double bl_mpg_value; /* 4: 手摇反向间隙补偿值 */

        /* 热误差补偿参数 (5-19) - 占位 */
        int heat_placeholders[15];

        /* 螺距误差补偿参数 (20-26) */
        int pitch_comp_type;         /* 20: 螺距误差补偿类型 */
        double pitch_comp_start_pos; /* 21: 螺距误差补偿起点坐标 */
        int pitch_comp_num;          /* 22: 螺距误差补偿点数 */
        double pitch_comp_step;      /* 23: 螺距误差补偿点间距 */
        int pitch_comp_modulo;       /* 24: 螺距误差取模补偿使能 */
        double pitch_comp_factor;    /* 25: 螺距误差补偿倍率 */
        int pitch_comp_table;        /* 26: 螺距误差补偿表起始参数号 */

        int reserved_27_29[3];

        /* 第1项垂直度补偿参数 (30-33) */
        int perp1_comp_enable;   /* 30: 第1项垂直度补偿使能 */
        int perp1_comp_input_ax; /* 31: 第1项垂直度补偿基准轴号 */
        double perp1_comp_refn;  /* 32: 第1项垂直度补偿参考点坐标 (单位：mm) */
        double perp1_comp_ang;   /* 33: 第1项垂直度补偿角度 (单位：um/m) */

        int reserved_34_39[6];

        /* 第2项垂直度补偿参数 (40-43) */
        int perp2_comp_enable;   /* 40: 第2项垂直度补偿使能 */
        int perp2_comp_input_ax; /* 41: 第2项垂直度补偿基准轴号 */
        double perp2_comp_refn;  /* 42: 第2项垂直度补偿参考点坐标 (单位：mm) */
        double perp2_comp_ang;   /* 43: 第2项垂直度补偿角度 (单位：um/m) */

        int reserved_44_49[6];

        /* 第1项直线度补偿参数 (50-57) */
        int stra1_input_ax;  /* 50: 第1项直线度补偿基准轴号 */
        int stra1_type;      /* 51: 第1项直线度补偿类型 */
        double stra1_start;  /* 52: 第1项直线度补偿起点坐标 */
        int stra1_num;       /* 53: 第1项直线度补偿点数 */
        double stra1_step;   /* 54: 第1项直线度补偿点间距 */
        int stra1_modulo;    /* 55: 第1项直线度取模补偿使能 */
        double stra1_factor; /* 56: 第1项直线度补偿倍率 */
        int stra1_table;     /* 57: 第1项直线度补偿表起始参数号 */

        int reserved_others[442];
    } LIB_PAR_ACMP;

#ifdef __cplusplus
}
#endif

#endif
