#include "comp_data.h"
#include "global.h"

// 螺距误差补偿数据
PitchCompRuntime pitch_rt[3];       // XYZ轴 螺距补偿状态机
pitch_comp_table_t pitch_table[3]; // XYZ轴 螺距补偿表

// 螺距误差补偿参数 (从共享内存 par_acmp 同步而来)
LIB_PAR_ACMP acmp_params[3];

void comp_data_sync_params(void) {
    for (int axis = 0; axis < 3; axis++) {
        acmp_params[axis].pitch_comp_type      = par_acmp[axis][PAR_CMP_PITCH_TYPE].value.val_int;
        acmp_params[axis].pitch_comp_start_pos = par_acmp[axis][PAR_CMP_PITCH_START].value.val_real;
        acmp_params[axis].pitch_comp_num       = par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int;
        acmp_params[axis].pitch_comp_step      = par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real;
        acmp_params[axis].pitch_comp_modulo    = par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_int;
        acmp_params[axis].pitch_comp_factor    = par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real;
        acmp_params[axis].pitch_comp_table     = par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int;
    }
}

void comp_data_init(void) {
    for (int axis = 0; axis < 3; axis++) {
        pitch_comp_init(&pitch_rt[axis], 0.001);
    }
    comp_data_sync_params();
    establish_pitch_comp_table();
}

// 从共享内存读取螺距误差补偿数据, 并建立各轴螺距误差补偿表
// 调用前需确保 comp_data_sync_params() 已将参数同步到 acmp_params
void establish_pitch_comp_table()
{
    for (int axis = 0; axis < 3; axis++) {
        LIB_PAR_ACMP *par = &acmp_params[axis];

        if (par->pitch_comp_num <= 0)
            continue;

        int num = par->pitch_comp_num;
        if (num > COMP_ARRAY_MAX_SIZE)
            num = COMP_ARRAY_MAX_SIZE;
        int table_start = par->pitch_comp_table;

        float fwd_errors[COMP_ARRAY_MAX_SIZE];
        float rev_errors[COMP_ARRAY_MAX_SIZE];
        double measurement_points[COMP_ARRAY_MAX_SIZE];

        // 正向补偿数组 (um), 负向紧随其后
        getPitchData(table_start, num, fwd_errors);
        getPitchData(table_start + num, num, rev_errors);

        for (int i = 0; i < num; i++) {
            fwd_errors[i] *= 0.001f;
            rev_errors[i] *= 0.001f;
        }

        // 生成测量点数组
        for (int i = 0; i < num; i++) {
            measurement_points[i] = par->pitch_comp_start_pos + (double)i * par->pitch_comp_step;
        }

        // 建立螺距误差补偿表
        generate_pitch_comp_table(&pitch_table[axis], measurement_points, fwd_errors, rev_errors, num);
    }
}