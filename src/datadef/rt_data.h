/********************************************************************
 * @file        rt_sys.h
 * @brief       实时系统数据
 * @version     V1.00
 * @date        2026/02/27
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 * @details     该文件下的数据如果存在多线程访问，请加锁使用（使用get set方法）；如为单线程使用，可直接进行操作
 ********************************************************************/

#ifndef RT_DATA_H
#define RT_DATA_H

#include "datatype.h"
#include "plan_queue.h"
#include <gncdatadef.h>
#include <pthread.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SYS_STATE_NUM 100
#define CHAN_STATE_NUM 100

// 路径和轨迹线程使用
typedef enum
{
    SYS_IS_READY = 0,           // 准备就绪
    SYS_IS_ESTOP_ACTIVE,        // 系统是否急停
    SYS_ESTOP_INPUT,            // 系统急停输入
    SYS_IS_ALARMING,            // 是否正在报警中
    SYS_IS_SWITCHING_MODE,      // 是否正在切换模式
    SYS_IS_MAC_LOCK_ON,         // 机床锁住开启
    SYS_IS_MAC_LOCK_READY,      // 机床锁住完成
    SYS_IS_IT_LOCK,             // 所有轴互锁
    SYS_IS_BSL_LOCK,            // 程序段开始锁住
    SYS_IS_CSL_LOCK,            // 切削程序段开始锁住
    SYS_IS_DRY_RUN,             // 空运行
    SYS_IS_BLOCK_SKIP,          // 程序段跳过（路径和轨迹线程使用）
    SYS_IS_SINGLE_BLOCK,        // 单段
    SYS_IS_MSBT_LOCK,           // 辅助功能锁住
    SYS_IS_RAPID,               // 是否快速
    SYS_IS_PROG_RESTAR,         // 程序再启动
    SYS_IS_STOP_SKIP,           // 外部跳转
    SYS_IS_RIGID,               // 刚性攻丝
    SYS_IS_RIGID_RERTURN,       // 攻丝回退
} SysStateIndex;

// 路径和轨迹两个线程使用
typedef enum
{
    CHAN_IS_PARSER_RESET = 0,   // 译码器复位  is_parser_reset
    CHAN_IS_PARSERING,          // 解释器是否运行中 is_parsering
    CHAN_IS_PARSER_RERUN,       // 译码器是否运行中 is_parser_running
    CHAN_IS_FIRST_RUN,          // 是否为第一次运行 is_first_run
    CHAN_IS_MOTION_SYNC,        // 等待运动完成同步握手
} ChanStateIndex;

typedef struct
{
    double last_pos;
    double current_pos;
    bool is_moving;         // 是否在运动  1运动　0静止
    bool direction;         // 运动方向　　１正向  0负向
    bool is_init;
    bool is_error;      // 驱动器是否报警 1是 0否
    bool is_decel_done; // 限位触发后减速是否完成 
    bool is_fwd_limit;  // 手动正方向限制
    bool is_rev_limit;  // 手动负方向限制
    bool plc_is_disable;
    bool is_disable;   // 轴是否被屏蔽
    bool is_lag_error; // 是否跟随误差报警 1是 0否
    // 外部减速状态：[0~4]功能号，[0]正向减速状态寄存，[1]负向减速状态寄存
    bool external_decel_active[SYS_EXTERNAL_DECEL_FUNCTION_COUNT][SYS_EXTERNAL_DECEL_DIRECTION_COUNT];
} AXIS_STATUS;

typedef struct
{
    int ref_setup;         // 建立回参考点
    int ref_on;            // 回参考点信号
    int second_ref_on;     // 回第二参考点信号
    int third_ref_on;      // 回第三参考点信号
    int forth_ref_on;      // 回第四参考点信号
}REF_STATUS;

typedef struct{
    double cycle;           // 插补周期
} RT_NCU;

// 目前仅轨迹线程使用
typedef struct
{
    double act_vel;         // 通道实际合成速度 
    double act_acc;         // 通道实际合成加速度 
    double act_jk;          // 通道实际合成加加速度 
    double cmd_vel;         // 通道指令合成速度 
    double cmd_acc;         // 通道指令合成加速度 
    double cmd_jk;          // 通道指令合成加加速度
    // 重启生效
    bool ex_dec_enable;     // 是否开启外部减速功能
    // 复位生效
    bool ex_dec2_enable;    // 是否开启外部减速功能2
    bool ex_dec3_enable;    // 是否开启外部减速功能3
    bool ex_dec4_enable;    // 是否开启外部减速功能4
    bool ex_dec5_enable;    // 是否开启外部减速功能5
    double cut_ex_dec1_vel; // 外部减速1对切削进给的减速速度
    double cut_ex_dec2_vel; // 外部减速2对切削进给的减速速度
    double cut_ex_dec3_vel; // 外部减速3对切削进给的减速速度
    double cut_ex_dec4_vel; // 外部减速4对切削进给的减速速度
    double cut_ex_dec5_vel; // 外部减速5对切削进给的减速速度
    double dryrun_spd;      // 空运行速度
    double max_tang_acc;    // 最大向心加速度
    int vplan_mode;         // 速度规划模式
    int lookahead_num;      // 速度规划前瞻段数
    double path_tolerance;  // 轨迹轮廓允差
    int corner_dec_factor;  // 拐角降速比例因子
    double l_min_thre;
    double l_theta_thre;
    double theta_thre;
    double kappa_thre;
    double seg_theta_thre;
    double kratio;
    double nurbs_fiterror;
    double nurbs_sagittaerror;
    int nurbs_enable;       // 开启nurbs拟合
    UNIF_TYPE unif_type;    // 一致性处理方式
} RT_CHAN;

// 目前仅轨迹线程使用
typedef struct 
{
    // 复位生效
    double max_feed_vel;                // 轴最大进给速度
    double max_feed_acc;                // 轴最大进给加速度
    double max_feed_jk;                 // 轴最大进给加加速度
    double max_rapid_vel;               // 轴最大快移速度
    double max_rapid_acc;               // 轴最大快移加速度
    double max_rapid_jk;                // 轴最大快移加加速度
    double jog_fastspd;                 // 轴快速点动速度
    double jog_lowspd;                  // 轴慢速点动速度
    double mpg_spd;                     // 手摇进给速度  
    double corner_vel_diff;             // 拐角的容许速度差
    double corner_acc_diff;             // 拐角的容许加速度差        
    double max_static_lag;              // 轴静态最大跟随误差
    double max_dynamic_lag;             // 轴动态最大跟随误差
    double pos_tol;                     // 轴定位允差
    unsigned int home_way;              // 回参考点方式
    double ref_range;                   // 参考点范围偏差
    double home_crds;                   // 参考点坐标值 
    double second_home_crds;            // 第二参考点坐标值
    double third_home_crds;             // 第三参考点坐标值
    double forth_home_crds;             // 第四参考点坐标值
    double plmt;                        // 正软极限
    double nlmt;                        // 负软极限
    double plmt2;                       // 第二级正软极限
    double nlmt2;                       // 第二级负软极限
    double spdl_max_speed;              // 主轴最大转速(r/min)
    bool ex_dec_enable_pos_cut;         // 外部减速信号对正向切削进给是否生效
    bool ex_dec_enable_neg_cut;         // 外部减速信号对负向切削进给是否生效
    double rpd_ex_dec1_vel;             // 外部减速1对快速进给的减速速度
    double rpd_ex_dec2_vel;             // 外部减速2对快速进给的减速速度
    double rpd_ex_dec3_vel;             // 外部减速3对快速进给的减速速度
    double rpd_ex_dec4_vel;             // 外部减速4对快速进给的减速速度
    double rpd_ex_dec5_vel;             // 外部减速5对快速进给的减速速度
    // 重启生效
    int pm_munit;                       // 电子齿轮比分子
    int pm_pulse;                       // 电子齿轮比分母
    double enc_off;                     // 编码器反馈偏置量
    // 内部
    double act_vel;                     // 轴实际速度
    double act_acc;                     // 轴实际加速度
    double act_jk;                      // 轴实际加加速度
    double cmd_vel;                     // 轴指令速度
    double cmd_acc;                     // 轴指令加速度
    double cmd_jk;                      // 轴指令加加速度
    int enc_cntr;                       // 电机位置
    double wcs_cmd_pos_cur;             // 工件插补计算位置
    double wcs_cmd_pos_pre;             // 工件插补计算位置(上一插补)
    double mac_cmd_pos;                 // 机床指令位置
    double mac_act_pos;                 // 机床实际位置 = fb_pos_no_comp
    double motor_cmd_pos;               // 驱动器指令位置（带补偿）
    double motor_act_pos;               // 驱动器反馈位置（带补偿）
    double fb_pos_no_comp;              // 反馈位置（去除补偿）
    double pitch_comp_val;              // 当前位置螺距误差补偿值 (单位:mm)
    double backlash_comp_val;           // 当前位置反向间隙补偿值 (单位:mm)
    double straightness1_comp_val;      // 当前位置第一组直线度误差补偿值 (单位:mm)
    double straightness2_comp_val;      // 当前位置第二组直线度误差补偿值 (单位:mm)
    double total_comp_cal;              // 当前位置总误差补偿值 (单位:mm)
    AXIS_STATUS status;
    REF_STATUS ref_status;
} RT_AXIS;

typedef struct{
    // 补偿参数
} RT_COMP;

typedef struct
{
    // 译码错误
    int parser_error_line;
    int parser_error_no;
} PATH_DATA;

typedef struct
{
    // 内部
    SYS_EVENT nc_event;
    NC_MODE target_mode;
    NC_MODE current_mode;
    bool sys_state[SYS_STATE_NUM];
    bool chan_state[SYS_CHAN_NUM][CHAN_STATE_NUM];
    PATH_DATA path_data;
    pthread_mutex_t mutex;
    // 参数
    RT_NCU ncu_data;
    RT_CHAN chan_data[SYS_CHAN_NUM];
    RT_AXIS axis_data[SYS_AXES_NUM];
    RT_COMP comp_data[SYS_AXES_NUM];
} RT_DATA;

GNC_RES rt_data_init(void);
GNC_RES rt_data_reset(void);
void rt_set_parser_state(int error_no, int error_line);
int rt_get_parser_error_no();
int rt_get_parser_error_line();
void rt_set_sys_state(SysStateIndex state, bool value);
bool rt_get_sys_state(SysStateIndex state);
void rt_set_chan_state(int chan_index, ChanStateIndex state, bool value);
bool rt_get_chan_state(int chan_index, ChanStateIndex state);
void rt_set_zero_vel_params();
void rt_set_axis_zero_vel_params(AXIS_TYPE axis_type);

#ifdef __cplusplus
}
#endif

#endif /* RT_DATA_H */
