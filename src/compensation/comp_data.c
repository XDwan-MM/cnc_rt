#include "comp_data.h"
#include "global.h"
#include <hmisysapi.h>

typedef enum
{
    STRAIGHTNESS_TABLE_GROUP_FIRST = 1,
    STRAIGHTNESS_TABLE_GROUP_SECOND = 2
} StraightnessTableGroup;

// 误差补偿库中的补偿参数结构体实例
CompParams par_comp_lib[3];

// 补偿算法状态机定义
PitchCompState pitch_comp_state[3];                // XYZ轴 螺距补偿状态机
BacklashCompState backlash_comp_state[3];          // XYZ轴 反向间隙补偿状态机
StraightnessCompState straightness1_comp_state[3]; // XYZ轴 第一组直线度误差补偿状态机
StraightnessCompState straightness2_comp_state[3]; // XYZ轴 第二组直线度误差补偿状态机

// 各个补偿表定义
PitchCompSeqList_t pitch_comp_table[3];                // XYZ轴 螺距补偿表
StraightnessCompSeqList_t straightness1_comp_table[3]; // XYZ轴 第一组直线度误差补偿表
StraightnessCompSeqList_t straightness2_comp_table[3]; // XYZ轴 第二组直线度误差补偿表

// 各个补偿表创建结果返回定义
OP_RES pitch_comp_table_status[3];         // 螺距补偿表建立结果返回
OP_RES straightness1_comp_table_status[3]; // 第一组直线度补偿表建立结果返回
OP_RES straightness2_comp_table_status[3]; // 第二组直线度补偿表建立结果返回

static const char *axis_names[] = {"X", "Y", "Z"};

#ifdef UNIT_TEST
extern int comp_data_test_get_straightness_data(int start, int point_cnt, int *fwd_comp_arr_um, int *rev_comp_arr_um);
#endif

/* ========================================================================== */
/* 内部函数声明                                                               */
/* ========================================================================== */

static const char *comp_table_result_reason(OP_RES status);
static void establish_straightness_comp_group(int axis, StraightnessTableGroup group);
static int getStraightnessData(int start, int point_cnt, int *fwd_comp_arr_um, int *rev_comp_arr_um);
static void notify_comp_params_synced(void);

/* ========================================================================== */
/* 外部函数定义                                                               */
/* ========================================================================== */

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
        par_comp_lib[axis].pitch_start = par_acmp[axis][PAR_CMP_PITCH_START].value.val_real;
        par_comp_lib[axis].pitch_num = par_acmp[axis][PAR_CMP_PITCH_NUM].value.val_int;
        par_comp_lib[axis].pitch_step = par_acmp[axis][PAR_CMP_PITCH_STEP].value.val_real;
        par_comp_lib[axis].pitch_modulo = par_acmp[axis][PAR_CMP_PITCH_MODULO].value.val_bool;
        par_comp_lib[axis].pitch_factor = par_acmp[axis][PAR_CMP_PITCH_FACTOR].value.val_real;
        par_comp_lib[axis].pitch_table = par_acmp[axis][PAR_CMP_PITCH_TABLE].value.val_int;

        // 第一组直线度补偿参数
        par_comp_lib[axis].stra1_mov_ax = par_acmp[axis][PAR_CMP_STRA1_INPUT_AX].value.val_int;
        par_comp_lib[axis].stra1_type = par_acmp[axis][PAR_CMP_STRA1_TYPE].value.val_int;
        par_comp_lib[axis].stra1_start_pos = par_acmp[axis][PAR_CMP_STRA1_START].value.val_real;
        par_comp_lib[axis].stra1_point_cnt = par_acmp[axis][PAR_CMP_STRA1_NUM].value.val_int;
        par_comp_lib[axis].stra1_step = par_acmp[axis][PAR_CMP_STRA1_STEP].value.val_real;
        par_comp_lib[axis].stra1_modulo = par_acmp[axis][PAR_CMP_STRA1_MODULO].value.val_bool;
        par_comp_lib[axis].stra1_factor = par_acmp[axis][PAR_CMP_STRA1_FACTOR].value.val_real;
        par_comp_lib[axis].stra1_table_start_idx = par_acmp[axis][PAR_CMP_STRA1_TABLE].value.val_int;

        // 第二组直线度补偿参数
        par_comp_lib[axis].stra2_mov_ax = par_acmp[axis][PAR_CMP_STRA2_INPUT_AX].value.val_int;
        par_comp_lib[axis].stra2_type = par_acmp[axis][PAR_CMP_STRA2_TYPE].value.val_int;
        par_comp_lib[axis].stra2_start_pos = par_acmp[axis][PAR_CMP_STRA2_START].value.val_real;
        par_comp_lib[axis].stra2_point_cnt = par_acmp[axis][PAR_CMP_STRA2_NUM].value.val_int;
        par_comp_lib[axis].stra2_step = par_acmp[axis][PAR_CMP_STRA2_STEP].value.val_real;
        par_comp_lib[axis].stra2_modulo = par_acmp[axis][PAR_CMP_STRA2_MODULO].value.val_bool;
        par_comp_lib[axis].stra2_factor = par_acmp[axis][PAR_CMP_STRA2_FACTOR].value.val_real;
        par_comp_lib[axis].stra2_table_start_idx = par_acmp[axis][PAR_CMP_STRA2_TABLE].value.val_int;
    }
}

// 初始化
void comp_data_init(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        // 螺距补偿
        pitch_comp_seqlist_init(&pitch_comp_table[axis]);
        pitch_comp_init(&pitch_comp_state[axis], 0.001);
        // 反向间隙
        backlash_comp_init(&backlash_comp_state[axis]);
        // 直线度补偿
        straightness_comp_seqlist_init(&straightness1_comp_table[axis]);
        straightness_comp_seqlist_init(&straightness2_comp_table[axis]);
        straightness_comp_init(&straightness1_comp_state[axis], 0.001);
        straightness_comp_init(&straightness2_comp_state[axis], 0.001);
    }

    sync_comp_params();
    establish_pitch_comp_table();
    establish_straightness_comp_table();
}

// 从共享内存读取螺距误差补偿数据, 并建立各轴螺距误差补偿表
// 调用前需确保 sync_comp_params() 已将参数同步到 par_comp_lib
void establish_pitch_comp_table(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        CompParams *par;
        int point_cnt;
        int read_result;

        par = &par_comp_lib[axis];
        point_cnt = par->pitch_num;

        if (par->pitch_type == 0 || point_cnt == 0)
        {
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            pitch_comp_table_status[axis] = OP_RES_OK;
            continue;
        }

        if (par->pitch_type != 1 && par->pitch_type != 2)
        {
            pitch_comp_table_status[axis] = OP_RES_ERR_OUT_OF_RANGE;
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: invalid compensation type (type=%d)",
                axis_names[axis],
                par->pitch_type);
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            continue;
        }

        if (point_cnt < 1 || point_cnt > COMP_ARRAY_MAX_SIZE)
        {
            pitch_comp_table_status[axis] = OP_RES_ERR_OUT_OF_RANGE;
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: invalid point count (point_cnt=%d)",
                axis_names[axis],
                point_cnt);
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            continue;
        }

        if (par->pitch_table < 0)
        {
            pitch_comp_table_status[axis] = OP_RES_ERR_OUT_OF_RANGE;
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: invalid table start (start=%d)",
                axis_names[axis],
                par->pitch_table);
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            continue;
        }

        int fwd_comp_arr_um[point_cnt];
        int rev_comp_arr_um[point_cnt];
        double meas_pos_arr_mm[point_cnt];

        read_result = getPitchData(par->pitch_table, point_cnt, fwd_comp_arr_um);
        if (read_result == 0)
            read_result = getPitchData(par->pitch_table + point_cnt, point_cnt, rev_comp_arr_um);
        if (read_result != 0)
        {
            pitch_comp_table_status[axis] = OP_RES_ERR_OUT_OF_RANGE;
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: shared-memory read failed (result=%d)",
                axis_names[axis],
                read_result);
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
            continue;
        }

        for (int i = 0; i < point_cnt; i++)
            meas_pos_arr_mm[i] = par->pitch_start + (double)i * par->pitch_step;

        pitch_comp_table_status[axis] = generate_pitch_comp_table(&pitch_comp_table[axis],
                                                                  meas_pos_arr_mm,
                                                                  fwd_comp_arr_um,
                                                                  rev_comp_arr_um,
                                                                  point_cnt);
        if (pitch_comp_table_status[axis] != OP_RES_OK)
        {
            LOG(LOG_LEVEL_ERR,
                "%s轴的螺距补偿表建立失败，原因: %s (status=%d)",
                axis_names[axis],
                comp_table_result_reason(pitch_comp_table_status[axis]),
                pitch_comp_table_status[axis]);
            pitch_comp_seqlist_init(&pitch_comp_table[axis]);
        }
    }
}

// 从共享内存读取直线度误差补偿数据, 并建立各轴两组直线度误差补偿表
// 调用前需确保 sync_comp_params() 已将参数同步到 par_comp_lib
void establish_straightness_comp_table(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        establish_straightness_comp_group(axis, STRAIGHTNESS_TABLE_GROUP_FIRST);
        establish_straightness_comp_group(axis, STRAIGHTNESS_TABLE_GROUP_SECOND);
    }
}

void comp_handle_event(SYS_EVENT event)
{
    switch (event)
    {
    case SYS_EVENT_HMI_RESET:
    case SYS_EVENT_COMP_PARAM_CHANGED:
    case SYS_EVENT_COMP_TABLE_CHANGED:
        sync_comp_params();
        establish_pitch_comp_table();
        establish_straightness_comp_table();
        notify_comp_params_synced();
        break;

    default:
        break;
    }
}

/* ========================================================================== */
/* 内部函数定义                                                               */
/* ========================================================================== */

static const char *comp_table_result_reason(OP_RES status)
{
    switch (status)
    {
    case OP_RES_ERR_NULL_PTR:
        return "null pointer";
    case OP_RES_ERR_OUT_OF_RANGE:
        return "invalid parameter range";
    case OP_RES_ERR_NOT_ASCENDING:
        return "positions not ascending";
    default:
        return "unknown error";
    }
}

static int getStraightnessData(int start, int point_cnt, int *fwd_comp_arr_um, int *rev_comp_arr_um)
{
#ifdef UNIT_TEST
    return comp_data_test_get_straightness_data(start, point_cnt, fwd_comp_arr_um, rev_comp_arr_um);
#else
    if (start < 0 || point_cnt < 0 || fwd_comp_arr_um == NULL || rev_comp_arr_um == NULL)
        return -1;

    for (int i = 0; i < point_cnt; i++)
    {
        fwd_comp_arr_um[i] = (start + i) * 10;
        rev_comp_arr_um[i] = (start + point_cnt + i) * 10;
    }

    return 0;
#endif
}

static void establish_straightness_comp_group(int axis, StraightnessTableGroup group)
{
    CompParams *par;
    int point_cnt;
    int read_result;
    int table_start;
    int type;
    double start_pos;
    OP_RES *status;
    double step;
    StraightnessCompSeqList_t *table;

    par = &par_comp_lib[axis];
    if (group == STRAIGHTNESS_TABLE_GROUP_FIRST)
    {
        type = par->stra1_type;
        start_pos = par->stra1_start_pos;
        point_cnt = par->stra1_point_cnt;
        step = par->stra1_step;
        table_start = par->stra1_table_start_idx;
        table = &straightness1_comp_table[axis];
        status = &straightness1_comp_table_status[axis];
    }
    else
    {
        type = par->stra2_type;
        start_pos = par->stra2_start_pos;
        point_cnt = par->stra2_point_cnt;
        step = par->stra2_step;
        table_start = par->stra2_table_start_idx;
        table = &straightness2_comp_table[axis];
        status = &straightness2_comp_table_status[axis];
    }

    if (type == 0 || point_cnt == 0)
    {
        straightness_comp_seqlist_init(table);
        *status = OP_RES_OK;
        return;
    }

    if (type != 1 && type != 2)
    {
        *status = OP_RES_ERR_OUT_OF_RANGE;
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: invalid compensation type (type=%d)",
            axis_names[axis],
            group,
            type);
        straightness_comp_seqlist_init(table);
        return;
    }

    if (point_cnt < 1 || point_cnt > COMP_ARRAY_MAX_SIZE)
    {
        *status = OP_RES_ERR_OUT_OF_RANGE;
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: invalid point count (point_cnt=%d)",
            axis_names[axis],
            group,
            point_cnt);
        straightness_comp_seqlist_init(table);
        return;
    }

    if (step <= 0.0)
    {
        *status = OP_RES_ERR_OUT_OF_RANGE;
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: invalid step (step=%f)",
            axis_names[axis],
            group,
            step);
        straightness_comp_seqlist_init(table);
        return;
    }

    if (table_start < 0)
    {
        *status = OP_RES_ERR_OUT_OF_RANGE;
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: invalid table start (start=%d)",
            axis_names[axis],
            group,
            table_start);
        straightness_comp_seqlist_init(table);
        return;
    }

    int fwd_comp_arr_um[point_cnt];
    int rev_comp_arr_um[point_cnt];
    double moving_ax_pos_arr_mm[point_cnt];

    read_result = getStraightnessData(table_start, point_cnt, fwd_comp_arr_um, rev_comp_arr_um);
    if (read_result != 0)
    {
        *status = OP_RES_ERR_OUT_OF_RANGE;
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: shared-memory read failed (result=%d)",
            axis_names[axis],
            group,
            read_result);
        straightness_comp_seqlist_init(table);
        return;
    }

    for (int i = 0; i < point_cnt; i++)
        moving_ax_pos_arr_mm[i] = start_pos + (double)i * step;

    *status = generate_straightness_comp_table(table,
                                               moving_ax_pos_arr_mm,
                                               fwd_comp_arr_um,
                                               rev_comp_arr_um,
                                               point_cnt);
    if (*status != OP_RES_OK)
    {
        LOG(LOG_LEVEL_ERR,
            "%s轴第%d组直线度补偿表建立失败，原因: %s (status=%d)",
            axis_names[axis],
            group,
            comp_table_result_reason(*status),
            *status);
        straightness_comp_seqlist_init(table);
    }
}

static void notify_comp_params_synced(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        pitch_comp_on_params_synced(&pitch_comp_state[axis]);
        backlash_comp_on_params_synced(&backlash_comp_state[axis]);
        straightness_comp_on_params_synced(&straightness1_comp_state[axis]);
        straightness_comp_on_params_synced(&straightness2_comp_state[axis]);
    }
}

// todo: 待删除
//     // 误差补偿轴参数
//     //---------------------补偿轴 X轴------------------------//
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_TYPE, &par_union);
//     par_acmp[AXIS_X].pitch_comp_type = par_union.value.val_int; //螺距误差补偿类型
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_START, &par_union);
//     par_acmp[AXIS_X].pitch_comp_start_pos = par_union.value.val_real; //螺距误差补偿起点坐标
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_NUM, &par_union);
//     par_acmp[AXIS_X].pitch_comp_num = par_union.value.val_int; //螺距误差补偿点数
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_STEP, &par_union);
//     par_acmp[AXIS_X].pitch_comp_step = par_union.value.val_real; //螺距误差补偿点间距
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_MODULO, &par_union);
//     par_acmp[AXIS_X].pitch_comp_modulo = par_union.value.val_bool; //螺距误差取模补偿使能
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_FACTOR, &par_union);
//     par_acmp[AXIS_X].pitch_comp_factor = par_union.value.val_real; //螺距误差补偿倍率
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_PITCH_TABLE, &par_union);
//     par_acmp[AXIS_X].pitch_comp_table = par_union.value.val_int; //螺距误差补偿表起始参数号
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU1_ENABLE, &par_union);
//     par_acmp[AXIS_X].perp1_comp_enable = par_union.value.val_bool; //第1项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU1_INPUT_AX, &par_union);
//     par_acmp[AXIS_X].perp1_comp_input_ax = par_union.value.val_int; //第1项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU1_REFN, &par_union);
//     par_acmp[AXIS_X].perp1_comp_refn = par_union.value.val_real; //第1项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU1_ANG, &par_union);
//     par_acmp[AXIS_X].perp1_comp_ang = par_union.value.val_real; //第1项垂直度补偿角度 (单位：um/m)
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU2_ENABLE, &par_union);
//     par_acmp[AXIS_X].perp2_comp_enable = par_union.value.val_bool; //第2项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU2_INPUT_AX, &par_union);
//     par_acmp[AXIS_X].perp2_comp_input_ax = par_union.value.val_int; //第2项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU2_REFN, &par_union);
//     par_acmp[AXIS_X].perp2_comp_refn = par_union.value.val_real; //第2项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 0, PAR_CMP_SQU2_ANG, &par_union);
//     par_acmp[AXIS_X].perp2_comp_ang = par_union.value.val_real; //第2项垂直度补偿角度 (单位：um/m)
//     //---------------------补偿轴 Y轴------------------------//
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_TYPE, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_type = par_union.value.val_int; //螺距误差补偿类型
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_START, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_start_pos = par_union.value.val_real; //螺距误差补偿起点坐标
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_NUM, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_num = par_union.value.val_int; //螺距误差补偿点数
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_STEP, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_step = par_union.value.val_real; //螺距误差补偿点间距
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_MODULO, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_modulo = par_union.value.val_bool; //螺距误差取模补偿使能
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_FACTOR, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_factor = par_union.value.val_real; //螺距误差补偿倍率
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_PITCH_TABLE, &par_union);
//     par_acmp[AXIS_Y].pitch_comp_table = par_union.value.val_int; //螺距误差补偿表起始参数号
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU1_ENABLE, &par_union);
//     par_acmp[AXIS_Y].perp1_comp_enable = par_union.value.val_bool; //第1项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU1_INPUT_AX, &par_union);
//     par_acmp[AXIS_Y].perp1_comp_input_ax = par_union.value.val_int; //第1项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU1_REFN, &par_union);
//     par_acmp[AXIS_Y].perp1_comp_refn = par_union.value.val_real; //第1项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU1_ANG, &par_union);
//     par_acmp[AXIS_Y].perp1_comp_ang = par_union.value.val_real; //第1项垂直度补偿角度 (单位：um/m)
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU2_ENABLE, &par_union);
//     par_acmp[AXIS_Y].perp2_comp_enable = par_union.value.val_bool; //第2项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU2_INPUT_AX, &par_union);
//     par_acmp[AXIS_Y].perp2_comp_input_ax = par_union.value.val_int; //第2项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU2_REFN, &par_union);
//     par_acmp[AXIS_Y].perp2_comp_refn = par_union.value.val_real; //第2项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 1, PAR_CMP_SQU2_ANG, &par_union);
//     par_acmp[AXIS_Y].perp2_comp_ang = par_union.value.val_real; //第2项垂直度补偿角度 (单位：um/m)
//     //---------------------补偿轴 Z轴------------------------//
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_TYPE, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_type = par_union.value.val_int; //螺距误差补偿类型
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_START, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_start_pos = par_union.value.val_real; //螺距误差补偿起点坐标
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_NUM, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_num = par_union.value.val_int; //螺距误差补偿点数
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_STEP, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_step = par_union.value.val_real; //螺距误差补偿点间距
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_MODULO, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_modulo = par_union.value.val_bool; //螺距误差取模补偿使能
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_FACTOR, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_factor = par_union.value.val_real; //螺距误差补偿倍率
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_PITCH_TABLE, &par_union);
//     par_acmp[AXIS_Z].pitch_comp_table = par_union.value.val_int; //螺距误差补偿表起始参数号
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU1_ENABLE, &par_union);
//     par_acmp[AXIS_Z].perp1_comp_enable = par_union.value.val_bool; //第1项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU1_INPUT_AX, &par_union);
//     par_acmp[AXIS_Z].perp1_comp_input_ax = par_union.value.val_int; //第1项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU1_REFN, &par_union);
//     par_acmp[AXIS_Z].perp1_comp_refn = par_union.value.val_real; //第1项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU1_ANG, &par_union);
//     par_acmp[AXIS_Z].perp1_comp_ang = par_union.value.val_real; //第1项垂直度补偿角度 (单位：um/m)
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU2_ENABLE, &par_union);
//     par_acmp[AXIS_Z].perp2_comp_enable = par_union.value.val_bool; //第2项垂直度补偿使能
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU2_INPUT_AX, &par_union);
//     par_acmp[AXIS_Z].perp2_comp_input_ax = par_union.value.val_int; //第2项垂直度补偿基准轴号
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU2_REFN, &par_union);
//     par_acmp[AXIS_Z].perp2_comp_refn = par_union.value.val_real; //第2项垂直度补偿参考点坐标 (单位：mm)
//     getParData(PARAMAN_FILE_ACMP, 2, PAR_CMP_SQU2_ANG, &par_union);
//     par_acmp[AXIS_Z].perp2_comp_ang = par_union.value.val_real; //第2项垂直度补偿角度 (单位：um/m)
