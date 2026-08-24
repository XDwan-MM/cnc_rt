#include "rt_data.h"
#include "global.h"
#include "rerun.h"
#include <hmisysapi.h>
#include "alarmapi.h"

extern RT_DATA* rt_data;
static void rt_reset_ncu_data();
static void rt_reset_chan_data();
static void rt_reset_axis_data();
static void rt_reboot_ncu_data();
static void rt_reboot_chan_data();
static void rt_reboot_axis_data();
static void rt_upate_reset_valid_data();
static void rt_update_reboot_valid_data();

GNC_RES rt_data_init(void)
{
    rt_data = (RT_DATA*)calloc(sizeof(RT_DATA), 1);
    if(rt_data == NULL) return GNC_DATA_NULL;
    rt_data->target_mode = MODE_EDIT;
    rt_data->current_mode = MODE_EDIT;
    rt_data->nc_event = SYS_EVENT_NONE;
    rt_data->path_data.parser_error_no = -1;
    pthread_mutex_init(&rt_data->mutex, NULL);
    rt_update_reboot_valid_data();
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES rt_data_reset(void){
    if(rt_data == NULL) return GNC_DATA_NULL;
    pthread_mutex_lock(&rt_data->mutex);
    rt_upate_reset_valid_data();
    setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);
    pthread_mutex_unlock(&rt_data->mutex);
    return GNC_OPERATOR_SUCCESS;
}

void rt_set_parser_state(int error_no, int error_line){
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->path_data.parser_error_no = error_no;
    rt_data->path_data.parser_error_line = error_line;
    pthread_mutex_unlock(&rt_data->mutex);
}

int rt_get_parser_error_no(){
    pthread_mutex_lock(&rt_data->mutex);
    int value = rt_data->path_data.parser_error_no;
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

int rt_get_parser_error_line(){
    pthread_mutex_lock(&rt_data->mutex);
    int value = rt_data->path_data.parser_error_line;
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

bool rt_get_sys_state(SysStateIndex state){
    if(state >= SYS_STATE_NUM || state < 0){
        return false;
    }
    pthread_mutex_lock(&rt_data->mutex);
    bool value = rt_data->sys_state[state];
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

void rt_set_sys_state(SysStateIndex state, bool value){
    if(state >= SYS_STATE_NUM || state < 0){
        return;
    }
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->sys_state[state] = value;
    pthread_mutex_unlock(&rt_data->mutex);
}

void rt_set_chan_state(int chan_index, ChanStateIndex state, bool value){
    if(chan_index < 0 || chan_index >= SYS_CHAN_NUM || state >= CHAN_STATE_NUM || state < 0){
        return;
    }
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->chan_state[chan_index][state] = value;
    pthread_mutex_unlock(&rt_data->mutex);
}

bool rt_get_chan_state(int chan_index, ChanStateIndex state){
    if(chan_index < 0 || chan_index >= SYS_CHAN_NUM || state >= CHAN_STATE_NUM || state < 0){
        return false;
    }
    pthread_mutex_lock(&rt_data->mutex);
    bool value = rt_data->chan_state[chan_index][state];
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

/// @brief 重启时生效的par数据，同步更新到实时数据
static void rt_update_reboot_valid_data(){
    rt_reboot_ncu_data();
    rt_reboot_chan_data();
    rt_reboot_axis_data();
    rt_upate_reset_valid_data();
    LOG(LOG_LEVEL_INFO, "重启参数生效");
}

/// @brief 复位时生效的par数据，同步更新到实时数据
static void rt_upate_reset_valid_data(){
    rt_reset_ncu_data();
    rt_reset_chan_data();
    rt_reset_axis_data();
    rt_set_zero_vel_params();
    LOG(LOG_LEVEL_INFO, "复位参数生效");
}

static void rt_reset_ncu_data(){
}

static void rt_reset_chan_data(){
    ParamUnit par_union = {};
    for(int i = 0; i < SYS_CHAN_NUM; i++){
        getParData(PARAMAN_FILE_CHAN, i, PAR_CH_DRYRUN_SPD, &par_union);
        rt_data->chan_data[i].dryrun_spd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);   // 空运行进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_CHAN, i, PAR_CH_VPLAN_MODE, &par_union);
        rt_data->chan_data[i].vplan_mode = par_union.value.val_uint;                        // 速度规划模式 0:T_SHAPE
        getParData(PARAMAN_FILE_CHAN, i, PAR_CH_MAX_ECEN_ACC, &par_union);
        rt_data->chan_data[i].max_tang_acc = par_union.value.val_real;                      // 最大向心加速度
        getParData(PARAMAN_FILE_CHAN, i, PAR_CH_LOOKAHEAD_NUM, &par_union);
        rt_data->chan_data[i].lookahead_num = par_union.value.val_uint;                     // 速度规划前瞻段数
        getParData(PARAMAN_FILE_CHAN, i, PAR_CH_PATH_TOLERANCE, &par_union);
        rt_data->chan_data[i].path_tolerance = par_union.value.val_real;                    // 轨迹轮廓允差
    }
}

static void rt_reset_axis_data(){
    ParamUnit par_union = {};
    for(int i = 0; i < SYS_AXES_NUM; i++){
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_RAPID_SPD, &par_union);
        rt_data->axis_data[i].max_rapid_vel = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);  // 轴最大快移速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_RAPID_ACC, &par_union);
        rt_data->axis_data[i].max_rapid_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real);   // 轴快移加速度 (HMI unit: ms | 实时 单位：mm/s^2)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_RAPID_JK, &par_union);
        rt_data->axis_data[i].max_rapid_jk = CONV_T_MS_TO_JERK(par_union.value.val_real);     // 轴快移加加速度 (HMI unit: ms | 实时 单位：mm/s^3)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_FEED_SPD, &par_union);
        rt_data->axis_data[i].max_feed_vel = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);   // 轴最大进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_FEED_ACC, &par_union);
        rt_data->axis_data[i].max_feed_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real);    // 轴进给加速度 (HMI unit: ms | 实时 单位：mm/s^2)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_FEED_JK, &par_union);
        rt_data->axis_data[i].max_feed_jk = CONV_T_MS_TO_JERK(par_union.value.val_real);      // 轴进给加加速度 (HMI unit: ms | 实时 单位：mm/s^3)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_JOG_LOWSPD, &par_union);
        rt_data->axis_data[i].jog_lowspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);     // 轴慢速点动速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_JOG_FASTSPD, &par_union);
        rt_data->axis_data[i].jog_fastspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);    // 轴快速点动速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_MPG_SPD, &par_union);
        rt_data->axis_data[i].mpg_spd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);        // 手摇进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_CORNER_VEL_DIFF, &par_union);
        rt_data->axis_data[i].corner_vel_diff = CONV_MM_MIN_TO_MM_S(par_union.value.val_real);// 拐角的容许速度差 (HMI unit: mm/min | 实时 单位：mm/s)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_CORNER_ACC_DIFF, &par_union);
        rt_data->axis_data[i].corner_acc_diff = par_union.value.val_real;                     // 拐角的容许加速度差 (HMI unit: 单位：mm/s2)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_MAX_DYNAMIC_LAG, &par_union);
        rt_data->axis_data[i].max_dynamic_ferror = par_union.value.val_real;                  // 轴动态最大跟随误差 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_MAX_STATIC_LAG, &par_union);
        rt_data->axis_data[i].max_static_ferror = par_union.value.val_real;                   // 轴静态最大跟随误差 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_POS_TOL, &par_union);
        rt_data->axis_data[i].pos_tol = par_union.value.val_real;                             // 轴定位允差 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_HOME_WAY, &par_union);
        rt_data->axis_data[i].home_way = par_union.value.val_uint;                            // 回参考点方式 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_REF_RANGE, &par_union);
        rt_data->axis_data[i].ref_range = par_union.value.val_real;                           // 轴回零参考点范围偏差 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_HOME_CRDS, &par_union);
        rt_data->axis_data[i].home_crds = par_union.value.val_real;                           // 参考点坐标值 (HMI unit: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_PLMT, &par_union);
        rt_data->axis_data[i].plmt = par_union.value.val_real;                                // 轴正软极限
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_NLMT, &par_union);
        rt_data->axis_data[i].nlmt = par_union.value.val_real;                                // 轴负软极限
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_PLMT2, &par_union);
        rt_data->axis_data[i].plmt2 = par_union.value.val_real;                               // 轴第二级正软极限
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_NLMT2, &par_union);
        rt_data->axis_data[i].nlmt2 = par_union.value.val_real;                               // 轴第二级负软极限
        getParData(PARAMAN_FILE_AXIS, i, PAR_SPDL_MAX_SPEED, &par_union);
        rt_data->axis_data[i].spdl_max_speed = par_union.value.val_real;                      // 主轴最大转速 (HMI unit: r/min)
    }
}

static void rt_reboot_ncu_data(){
    ParamUnit par_union = {};
    getParData(PARAMAN_FILE_NCU, 0, PAR_NCU_CYCLE, &par_union);
    rt_data->ncu_data.cycle = (double)par_union.value.val_uint / TIME_US_2_S;          // 插补周期
}

static void rt_reboot_chan_data(){
}

static void rt_reboot_axis_data(){
    ParamUnit par_union = {};
    for(int i = 0; i < SYS_AXES_NUM; i++){
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_PM_MUNIT, &par_union);
        rt_data->axis_data[i].pm_munit = CONV_UM_TO_MM(par_union.value.val_int);         // 轴电子齿轮比分子 (HMI unit: um, | 实时 单位: mm)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_PM_PULSE, &par_union);
        rt_data->axis_data[i].pm_pulse = par_union.value.val_int;                        // 轴电子齿轮比分母 (HMI unit: 脉冲)
        getParData(PARAMAN_FILE_AXIS, i, PAR_AX_ENC_OFF, &par_union);
        rt_data->axis_data[i].enc_off = par_union.value.val_real;                        // 轴编码器反馈偏置量 (HMI unit: mm)
    }
}

void rt_set_zero_vel_params(){
    for (int i = 0; i < SYS_CHAN_NUM; i++)
    {
        rt_data->chan_data[i].cmd_vel = 0;
        rt_data->chan_data[i].cmd_acc = 0;
        rt_data->chan_data[i].cmd_jk = 0;
    }
    for (int i = 0; i < SYS_AXES_NUM; i++){
        rt_data->axis_data[i].cmd_vel = 0;
        rt_data->axis_data[i].cmd_acc = 0;
        rt_data->axis_data[i].cmd_jk = 0;
    }
}