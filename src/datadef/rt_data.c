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

static void rt_reset_chan_data()
{
    for (int i = 0; i < SYS_CHAN_NUM; i++)
    {
        rt_data->chan_data[i].dryrun_spd = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_DRYRUN_SPD].value.val_real);           // 空运行进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->chan_data[i].vplan_mode = par_chan[i][PAR_CH_VPLAN_MODE].value.val_uint;                                // 速度规划模式 0:T_SHAPE
        rt_data->chan_data[i].max_tang_acc = par_chan[i][PAR_CH_MAX_ECEN_ACC].value.val_real;                            // 最大向心加速度
        rt_data->chan_data[i].lookahead_num = par_chan[i][PAR_CH_LOOKAHEAD_NUM].value.val_uint;                          // 速度规划前瞻段数
        rt_data->chan_data[i].path_tolerance = par_chan[i][PAR_CH_PATH_TOLERANCE].value.val_real;                        // 轨迹轮廓允差
        rt_data->chan_data[i].corner_dec_factor = par_chan[i][PAR_CH_CONER_DEC_FACTOR].value.val_int;                    // 拐角降速比例因子 ("def": "100", "min": "1", "max": "200")
        rt_data->chan_data[i].max_fv_arc_radius = par_chan[i][PAR_CH_MAX_FEEDRATE_ARC_RADIUS].value.val_real;            // 与最大进给速度相对应的圆弧半径值
        rt_data->chan_data[i].radius_r_max_fv = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_RADIUS_R_MAX_FEEDRATE].value.val_real); // 半径R的圆弧最大进给速度
        rt_data->chan_data[i].circular_min_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CIRCLE_MIN_VEL].value.val_real);       // 基于圆弧插补下减速的下限速度
        rt_data->chan_data[i].axis_min_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_AXIS_MIN_VEL].value.val_real);             // 基于每个轴加速度的下限速度
        rt_data->chan_data[i].ex_dec2_enable = par_chan[i][PAR_CH_EX_DEC2_ENABLE].value.val_bool;                        // 是否开启外部减速功能2
        rt_data->chan_data[i].ex_dec3_enable = par_chan[i][PAR_CH_EX_DEC3_ENABLE].value.val_bool;                        // 是否开启外部减速功能3
        rt_data->chan_data[i].ex_dec4_enable = par_chan[i][PAR_CH_EX_DEC4_ENABLE].value.val_bool;                        // 是否开启外部减速功能4
        rt_data->chan_data[i].ex_dec5_enable = par_chan[i][PAR_CH_EX_DEC5_ENABLE].value.val_bool;                        // 是否开启外部减速功能5
        rt_data->chan_data[i].cut_ex_dec1_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CUT_EX_DEC1_VEL].value.val_real); // 外部减速1对切削进给的减速速度(mm/min)
        rt_data->chan_data[i].cut_ex_dec2_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CUT_EX_DEC2_VEL].value.val_real); // 外部减速2对切削进给的减速速度(mm/min)
        rt_data->chan_data[i].cut_ex_dec3_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CUT_EX_DEC3_VEL].value.val_real); // 外部减速3对切削进给的减速速度(mm/min)
        rt_data->chan_data[i].cut_ex_dec4_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CUT_EX_DEC4_VEL].value.val_real); // 外部减速4对切削进给的减速速度(mm/min)
        rt_data->chan_data[i].cut_ex_dec5_vel = CONV_MM_MIN_TO_MM_S(par_chan[i][PAR_CH_CUT_EX_DEC5_VEL].value.val_real); // 外部减速5对切削进给的减速速度(mm/min)
        rt_data->chan_data[i].l_min_thre = par_chan[i][PAR_CH_L_MIN_THR].value.val_real;                                 //  小线段剔除
        rt_data->chan_data[i].l_theta_thre = par_chan[i][PAR_CH_L_THETA_THR].value.val_real;                             //  逆行修复：线段长度阈值（小于该值的才修复）
        rt_data->chan_data[i].theta_thre = par_chan[i][PAR_CH_THETA_THR].value.val_real;                                 //  逆行修复：线段间角度阈值（连续角度判定）  
        rt_data->chan_data[i].kappa_thre = par_chan[i][PAR_CH_KAPPA_THR].value.val_real;                                 //  逆行修复：线段间曲率阈值 未使用
        rt_data->chan_data[i].seg_theta_thre = par_chan[i][PAR_CH_SEG_THETA_THR].value.val_real;                         //  角度分段    
        rt_data->chan_data[i].kratio = par_chan[i][PAR_CH_KRATIO].value.val_real;                                        //  曲率分段 未使用
        rt_data->chan_data[i].nurbs_fiterror = par_chan[i][PAR_CH_NURBS_FIT_ERROR].value.val_real;                       //  拟合误差
        rt_data->chan_data[i].nurbs_sagittaerror = par_chan[i][PAR_CH_NURBS_SAGITTA_ERROR].value.val_real;               //  弓高误差
        rt_data->chan_data[i].nurbs_enable = par_chan[i][PAR_CH_NURBS_ENABLE].value.val_int;                             //  1 开启nurbs  0 关闭nurbs
        rt_data->chan_data[i].unif_type = par_chan[i][PAR_CH_UNIF_TYPE].value.val_int;                                   //  选择一致性类型
        // LOG(LOG_LEVEL_DEBUG, "nurbs_enable %d, unif_type %d", rt_data->chan_data[i].nurbs_enable, rt_data->chan_data[i].unif_type);
    }
}

static void rt_reset_axis_data()
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        rt_data->axis_data[i].max_rapid_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RAPID_SPD].value.val_real);         // 轴最大快移速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].max_rapid_acc = CONV_T_MS_TO_ACCEL(par_axis[i][PAR_AX_RAPID_ACC].value.val_real);          // 轴快移加速度 (HMI unit: ms | 实时 单位：mm/s^2)
        rt_data->axis_data[i].max_rapid_jk = CONV_T_MS_TO_JERK(par_axis[i][PAR_AX_RAPID_JK].value.val_real);             // 轴快移加加速度 (HMI unit: ms | 实时 单位：mm/s^3)
        rt_data->axis_data[i].max_feed_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_FEED_SPD].value.val_real);           // 轴最大进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].max_feed_acc = CONV_T_MS_TO_ACCEL(par_axis[i][PAR_AX_FEED_ACC].value.val_real);            // 轴进给加速度 (HMI unit: ms | 实时 单位：mm/s^2)
        rt_data->axis_data[i].max_feed_jk = CONV_T_MS_TO_JERK(par_axis[i][PAR_AX_FEED_JK].value.val_real);               // 轴进给加加速度 (HMI unit: ms | 实时 单位：mm/s^3)
        rt_data->axis_data[i].jog_lowspd = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_JOG_LOWSPD].value.val_real);           // 轴慢速点动速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].jog_fastspd = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_JOG_FASTSPD].value.val_real);         // 轴快速点动速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].mpg_spd = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_MPG_SPD].value.val_real);                 // 手摇进给速度 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].corner_vel_diff = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_CORNER_VEL_DIFF].value.val_real); // 拐角的容许速度差 (HMI unit: mm/min | 实时 单位：mm/s)
        rt_data->axis_data[i].corner_acc_diff = par_axis[i][PAR_AX_CORNER_ACC_DIFF].value.val_real;                      // 拐角的容许加速度差 (HMI unit: 单位：mm/s2)
        rt_data->axis_data[i].max_dynamic_lag = par_axis[i][PAR_AX_MAX_DYNAMIC_LAG].value.val_real;                      // 轴动态最大跟随误差 (HMI unit: mm)
        rt_data->axis_data[i].max_static_lag = par_axis[i][PAR_AX_MAX_STATIC_LAG].value.val_real;                        // 轴静态最大跟随误差 (HMI unit: mm)
        rt_data->axis_data[i].pos_tol = par_axis[i][PAR_AX_POS_TOL].value.val_real;                                      // 轴定位允差 (HMI unit: mm)
        rt_data->axis_data[i].home_way = par_axis[i][PAR_AX_HOME_WAY].value.val_uint;                                    // 回参考点方式 (HMI unit: mm)
        rt_data->axis_data[i].ref_range = par_axis[i][PAR_AX_REF_RANGE].value.val_real;                                  // 轴回零参考点范围偏差 (HMI unit: mm)
        rt_data->axis_data[i].home_crds = par_axis[i][PAR_AX_HOME_CRDS].value.val_real;                                  // 参考点坐标值 (HMI unit: mm)
        rt_data->axis_data[i].second_home_crds = par_axis[i][PAR_AX_HOME_CRDS2].value.val_real;                          // 第二参考点坐标值 (HMI unit: mm)
        rt_data->axis_data[i].third_home_crds = par_axis[i][PAR_AX_HOME_CRDS3].value.val_real;                           // 第三参考点坐标值 (HMI unit: mm)
        rt_data->axis_data[i].forth_home_crds = par_axis[i][PAR_AX_HOME_CRDS4].value.val_real;                           // 第四参考点坐标值 (HMI unit: mm)
        rt_data->axis_data[i].plmt = par_axis[i][PAR_AX_PLMT].value.val_real;                                            // 轴正软极限
        rt_data->axis_data[i].nlmt = par_axis[i][PAR_AX_NLMT].value.val_real;                                            // 轴负软极限
        rt_data->axis_data[i].plmt2 = par_axis[i][PAR_AX_PLMT2].value.val_real;                                          // 轴第二级正软极限
        rt_data->axis_data[i].nlmt2 = par_axis[i][PAR_AX_NLMT2].value.val_real;                                          // 轴第二级负软极限
        rt_data->axis_data[i].plmt3 = par_axis[i][PAR_AX_PLMT3].value.val_real;                                          // 轴第二级正软极限
        rt_data->axis_data[i].nlmt3 = par_axis[i][PAR_AX_NLMT3].value.val_real;                                          // 轴第二级负软极限
        rt_data->axis_data[i].lmt2_type = par_axis[i][PAR_AX_LMT2_TYPE].value.val_int;                                   // 轴第二级软极限类型
        rt_data->axis_data[i].lmt2_enable = par_axis[i][PAR_AX_LMT2_ENABLE].value.val_int;                               // 轴第二级软极限开启
        rt_data->axis_data[i].lmt3_enable = par_axis[i][PAR_AX_LMT3_ENABLE].value.val_int;                               // 轴第三级软极限开启
        rt_data->axis_data[i].spl_default_speed = par_axis[i][PAR_AX_DEFAULT_S].value.val_real;                          // 主轴缺省S速度
        rt_data->axis_data[i].spdl_max_speed = par_axis[i][PAR_SPDL_MAX_SPEED].value.val_real;                           // 主轴最大转速 (HMI unit: r/min)
        rt_data->axis_data[i].ex_dec_enable_pos_cut = par_axis[i][PAR_AX_EX_DEC_ENABLE_POS_CUT].value.val_bool;          // 外部减速信号对正向切削进给是否生效
        rt_data->axis_data[i].ex_dec_enable_neg_cut = par_axis[i][PAR_AX_EX_DEC_ENABLE_NEG_CUT].value.val_bool;          // 外部减速信号对负向切削进给是否生效
        rt_data->axis_data[i].rpd_ex_dec1_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RPD_EX_DEC1_VEL].value.val_real); // 快移外部减速1的减速速度(mm/min)
        rt_data->axis_data[i].rpd_ex_dec2_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RPD_EX_DEC2_VEL].value.val_real); // 快移外部减速2的减速速度(mm/min)
        rt_data->axis_data[i].rpd_ex_dec3_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RPD_EX_DEC3_VEL].value.val_real); // 快移外部减速3的减速速度(mm/min)
        rt_data->axis_data[i].rpd_ex_dec4_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RPD_EX_DEC4_VEL].value.val_real); // 快移外部减速4的减速速度(mm/min)
        rt_data->axis_data[i].rpd_ex_dec5_vel = CONV_MM_MIN_TO_MM_S(par_axis[i][PAR_AX_RPD_EX_DEC5_VEL].value.val_real); // 快移外部减速5的减速速度(mm/min)
    }
}

static void rt_reboot_ncu_data(){
    rt_data->ncu_data.cycle = CONV_US_TO_S(par_ncu[PAR_NCU_CYCLE].value.val_uint);          // 插补周期 (HMI unit: us | 实时 单位: s)
}

static void rt_reboot_chan_data()
{
    for (int i = 0; i < SYS_CHAN_NUM; i++)
    {
        rt_data->chan_data[i].ex_dec_enable = par_chan[i][PAR_CH_EX_DEC_ENABLE].value.val_bool; // 是否开启外部减速功能
    }
}

static void rt_reboot_axis_data()
{
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        rt_data->axis_data[i].pm_munit = CONV_UM_TO_MM(par_axis[i][PAR_AX_PM_MUNIT].value.val_int); // 轴电子齿轮比分子 (HMI unit: um, | 实时 单位: mm)
        rt_data->axis_data[i].pm_pulse = par_axis[i][PAR_AX_PM_PULSE].value.val_int;                // 轴电子齿轮比分母 (HMI unit: 脉冲)
        rt_data->axis_data[i].enc_off = par_axis[i][PAR_AX_ENC_OFF].value.val_real;                 // 轴编码器反馈偏置量 (HMI unit: mm)
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

void rt_set_axis_zero_vel_params(AXIS_TYPE axis_type){
    rt_data->axis_data[axis_type].cmd_vel = 0;
    rt_data->axis_data[axis_type].cmd_acc = 0;
    rt_data->axis_data[axis_type].cmd_jk = 0;
}