#include "comp_data.h"
#include "global.h"
#include <hmisysapi.h>
#include <stdint.h>

// 螺距误差补偿数据
PitchCompState pitch_comp_state[3];       // XYZ轴 螺距补偿状态机
BacklashCompState backlash_comp_state[3]; // XYZ轴 反向间隙补偿状态机
PerpCompState perp_comp_state[3];         // XYZ轴 垂直度补偿状态机

PitchCompSeqList_t pitch_comp_table[3]; // XYZ轴 螺距补偿表
OP_RES pitch_comp_table_status[3];      // 螺距补偿表建立状态
static const char *axis_names[] = {"X", "Y", "Z"};

// 误差补偿库中的补偿参数结构体实例
CompParams par_comp_lib[3];

// 同步补偿参数(实时与误差补偿库)
void sync_comp_params(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        // 反向间隙补偿参数
        par_comp_lib[axis].bl_enable = par_acmp[axis][PAR_CMP_BL_ENABLE].value.val_int;
        par_comp_lib[axis].bl_value = par_acmp[axis][PAR_CMP_BL_VALUE].value.val_real;
        par_comp_lib[axis].bl_rate = par_acmp[axis][PAR_CMP_BL_RATE].value.val_real;
        par_comp_lib[axis].bl_value2 = par_acmp[axis][PAR_CMP_BL_VALUE2].value.val_real;

        // 螺距补偿参数
        par_comp_lib[axis].pitch_type = par_acmp[axis][PAR_CMP_PITCH_TYPE].value.val_int;
        // printf("PAR_CMP_PITCH_TYPE %d\n", par_acmp[axis][PAR_CMP_PITCH_TYPE].value.val_int);
        par_comp_lib[axis].pitch_start = par_acmp[axis][PAR_CMP_PITCH_START].value.val_real;
        // printf("PAR_CMP_PITCH_START %f\n", par_acmp[axis][PAR_CMP_PITCH_START].value.val_real);
        par_comp_lib[axis].pitch_num = par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int;
        // printf("PAR_CMP_PITCH_NUM %d\n", par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int);
        par_comp_lib[axis].pitch_step = par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real;
        // printf("PAR_CMP_PITCH_STEP %f\n", par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real);
        par_comp_lib[axis].pitch_modulo = par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_bool;
        // printf("PAR_CMP_PITCH_MODULO %d\n", par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_int);
        par_comp_lib[axis].pitch_factor = par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real;
        // printf("PAR_CMP_PITCH_FACTOR %f\n", par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real);
        par_comp_lib[axis].pitch_table = par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int;
        // printf("PAR_CMP_PITCH_TABLE %d\n", par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int);

        // 垂直度补偿参数
        par_comp_lib[axis].squ1_enable = par_acmp[axis][PAR_CMP_SQU1_ENABLE].value.val_int;
        par_comp_lib[axis].squ1_input_ax = par_acmp[axis][PAR_CMP_SQU1_INPUT_AX].value.val_int;
        par_comp_lib[axis].squ1_refn = par_acmp[axis][PAR_CMP_SQU1_REFN].value.val_real;
        par_comp_lib[axis].squ1_ang = par_acmp[axis][PAR_CMP_SQU1_ANG].value.val_real;
        par_comp_lib[axis].squ2_enable = par_acmp[axis][PAR_CMP_SQU2_ENABLE].value.val_int;
        par_comp_lib[axis].squ2_input_ax = par_acmp[axis][PAR_CMP_SQU2_INPUT_AX].value.val_int;
        par_comp_lib[axis].squ2_refn = par_acmp[axis][PAR_CMP_SQU2_REFN].value.val_real;
        par_comp_lib[axis].squ2_ang = par_acmp[axis][PAR_CMP_SQU2_ANG].value.val_real;

        // 直线度补偿参数 (预留)
        // TODO: stra1_input_ax, stra1_type, stra1_start, stra1_num, stra1_step,
        //       stra1_modulo, stra1_factor, stra1_table -- 等库函数实现后接入

        // TODO: 后续实现脏标记机制
        // if (par_acmp[axis][PAR_CMP_PITCH_TABLE_DIRTY].value.val_int == 1) {
        //     establish_pitch_comp_table();
        //     par_acmp[axis][PAR_CMP_PITCH_TABLE_DIRTY].value.val_int = 0;
        // }
    }
}

void comp_data_init(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        pitch_comp_seqlist_init(&pitch_comp_table[axis]);
        pitch_comp_init(&pitch_comp_state[axis], 0.001);
        backlash_comp_init(&backlash_comp_state[axis]);
        perp_comp_init(&perp_comp_state[axis], 0.001);
    }
    // 同步补偿参数
    sync_comp_params();
    // 建立补偿表
    establish_pitch_comp_table();
}

// 从共享内存读取螺距误差补偿数据, 并建立各轴螺距误差补偿表
// 调用前需确保 comp_data_sync_params() 已将参数同步到 par_comp_lib
void establish_pitch_comp_table(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        CompParams *par = &par_comp_lib[axis];
        // 补偿点数为0 重置螺补表
        if (par->pitch_num <= 0)
        {
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            pitch_comp_table_status[axis] = OP_RES_OK;
            continue;
        }
        // 非单向或双向 重置补偿表
        if (par->pitch_type != 1 && par->pitch_type != 2)
        {
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            pitch_comp_table_status[axis] = OP_RES_OK;
            continue;
        }
        // 补偿点数 (单轴不超过256个点)
        int num = par->pitch_num;
        if (num > COMP_ARRAY_MAX_SIZE)
            num = COMP_ARRAY_MAX_SIZE;

        // 补偿表起始号
        int table_start = par->pitch_table;

        // 螺补表中的数据 单位：um
        int fwd_comp_arr_um[num];
        int rev_comp_arr_um[num];
        double meas_pos_arr_mm[num];

        /**
         * @brief getPitchData 获取指定范围内的螺补数组
         * @param[in] start     获取数组的起点
         * @param[in] num       获取数组的个数
         * @param[out] pitchData    得到数组
         * @return
         */
        // 正向/负向补偿数组赋值
        getPitchData(table_start, num, fwd_comp_arr_um);
        getPitchData(table_start + num, num, rev_comp_arr_um);
// #ifdef DEBUG_MODE_ENABLED
//         for (int i = 0; i < num; i++)
//         {
//             printf("fwd%d rev%d\n", fwd_comp_arr_um[i], rev_comp_arr_um[i]);
//         }
// #endif
        // 生成测量位置数组
        for (int i = 0; i < num; i++)
        {
            meas_pos_arr_mm[i] = par->pitch_start + (double)i * par->pitch_step;
        }

        // 建立实时中螺距误差补偿表
        pitch_comp_table_status[axis] = generate_pitch_comp_table(
            &pitch_comp_table[axis],
            meas_pos_arr_mm,
            fwd_comp_arr_um,
            rev_comp_arr_um,
            num
        );

        if (pitch_comp_table_status[axis] != OP_RES_OK)
        {
            const char *reason;
            switch (pitch_comp_table_status[axis])
            {
            case OP_RES_ERR_NULL_PTR:
                reason = "null pointer";
                break;
            case OP_RES_ERR_OUT_OF_RANGE:
                reason = "invalid point count";
                break;
            case OP_RES_ERR_NOT_ASCENDING:
                reason = "positions not ascending";
                break;
            default:
                reason = "unknown error";
                break;
            }
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: %s (status=%d)",
                axis_names[axis],
                reason,
                pitch_comp_table_status[axis]);

            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
        }
    }
}

void comp_handle_event(SYS_EVENT event)
{
#ifdef DEBUG_MODE_ENABLED
    // 强制模式：忽略事件值，每周期全量同步+建表
    sync_comp_params();
    establish_pitch_comp_table();
    for (int axis = 0; axis < 3; axis++)
        pitch_comp_on_params_synced(&pitch_comp_state[axis]);
#else
    switch (event)
    {
    case SYS_EVENT_COMP_TABLE_CHANGED:
        // 螺补表数据变更：先同步参数（建表依赖 par_comp_lib），再建表，再通知
        sync_comp_params();
        establish_pitch_comp_table();
        for (int axis = 0; axis < 3; axis++)
            pitch_comp_on_params_synced(&pitch_comp_state[axis]);
        break;

    case SYS_EVENT_COMP_PARAM_CHANGED:
        // 仅参数变更：同步 + 通知各模块平滑过渡
        sync_comp_params();
        for (int axis = 0; axis < 3; axis++)
        {
            pitch_comp_on_params_synced(&pitch_comp_state[axis]);
            backlash_comp_on_params_synced(&backlash_comp_state[axis]);
            perp_comp_on_params_synced(&perp_comp_state[axis]);
        }
        break;

    default:
        break;
    }
#endif
}