#include "comp_data.h"
#include "global.h"
#include <stdint.h>

// 螺距误差补偿数据
PitchCompState pitch_rt[3];             // XYZ轴 螺距补偿状态机
BacklashCompState backlash_rt[3];       // XYZ轴 反向间隙补偿状态机
pitch_comp_table_t pitch_table[3];      // XYZ轴 螺距补偿表
PitchTableStatus pitch_table_status[3]; // 螺距补偿表建立状态

// 误差补偿库中的补偿参数结构体实例
CompParams acmp_params[3];

// 同步补偿参数(实时与误差补偿库)
void sync_comp_params(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        // 反向间隙补偿参数
        acmp_params[axis].bl_enable = par_acmp[axis][PAR_CMP_BL_ENABLE].value.val_int;
        acmp_params[axis].bl_value = par_acmp[axis][PAR_CMP_BL_VALUE].value.val_real;
        acmp_params[axis].bl_rate = par_acmp[axis][PAR_CMP_BL_RATE].value.val_real;
        acmp_params[axis].bl_value2 = par_acmp[axis][PAR_CMP_BL_VALUE2].value.val_real;

        // 螺距补偿参数
        acmp_params[axis].pitch_type = par_acmp[axis][PAR_CMP_PITCH_TYPE].value.val_int;
        // printf("PAR_CMP_PITCH_TYPE %d\n", par_acmp[axis][PAR_CMP_PITCH_TYPE].value.val_int);
        acmp_params[axis].pitch_start = par_acmp[axis][PAR_CMP_PITCH_START].value.val_real;
        // printf("PAR_CMP_PITCH_START %f\n", par_acmp[axis][PAR_CMP_PITCH_START].value.val_real);
        acmp_params[axis].pitch_num = par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int;
        // printf("PAR_CMP_PITCH_NUM %d\n", par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int);
        acmp_params[axis].pitch_step = par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real;
        // printf("PAR_CMP_PITCH_STEP %f\n", par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real);
        acmp_params[axis].pitch_modulo = par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_bool;
        // printf("PAR_CMP_PITCH_MODULO %d\n", par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_int);
        acmp_params[axis].pitch_factor = par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real;
        // printf("PAR_CMP_PITCH_FACTOR %f\n", par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real);
        acmp_params[axis].pitch_table = par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int;
        // printf("PAR_CMP_PITCH_TABLE %d\n", par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int);

        // 垂直度补偿参数
    }
}

void comp_data_init(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        pitch_comp_init(&pitch_rt[axis], 0.001);
        backlash_comp_init(&backlash_rt[axis]);
    }
    sync_comp_params();
    establish_pitch_comp_table();
}

// 从共享内存读取螺距误差补偿数据, 并建立各轴螺距误差补偿表
// 调用前需确保 comp_data_sync_params() 已将参数同步到 acmp_params
void establish_pitch_comp_table(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        CompParams *par = &acmp_params[axis];

        if (par->pitch_num <= 0)
            continue;
        // 补偿点数
        int num = par->pitch_num;
        if (num > COMP_ARRAY_MAX_SIZE)
            num = COMP_ARRAY_MAX_SIZE;

        // 补偿表起始号
        int table_start = par->pitch_table;

        // 螺补表中的数据：人机交互传下来的16位整型数据(-32768 到 32767)
        int16_t fwd_errors[num];
        int16_t rev_errors[num];
        double measurement_points[num];

        /* 补偿数组，共享内存空间中连续，正向补偿数组+负向补偿数组 */
        // 正向补偿数组(um)
        getPitchData(table_start, num, fwd_errors);
        // 负向补偿数组(um)
        getPitchData(table_start + num, num, rev_errors);

        // 人机交互输入表中的数值单位为检测单位，坐标显示小数点后三位，则输入到表中的数值的单位为um。
        for (int i = 0; i < num; i++)
        {
            fwd_errors[i] *= 0.001f;
            rev_errors[i] *= 0.001f;
        }

        // for (int i = 0; i < num; i++) {
        //     printf("fwd_errors[%d] %f\n", i, fwd_errors[i]);
        //     printf("fwd_errors[%d] %f\n", i, rev_errors[i]);
        // }

        // 生成测量点数组
        for (int i = 0; i < num; i++)
        {
            measurement_points[i] = par->pitch_start + (double)i * par->pitch_step;
        }

        // 建立实时中螺距误差补偿表
        pitch_table_status[axis] = generate_pitch_comp_table(&pitch_table[axis], measurement_points, fwd_errors, rev_errors, num);
    }
}