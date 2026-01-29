/********************************************************************
 * @file        param 变量.h
 * @brief       除非用户主动设置，在系统运行中不会发生变化的变量，每个参数都包含权限属性，
                以便实现参数的分级管理，不同级别的用户可以读写的参数不同。
 * @version     V1.00
 * @date        2025/12/08
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef RT_PAR_H
#define RT_PAR_H

#include <gncdatadef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 主轴参数
typedef struct {
    double motor_speed_max;        // PAR_SP_MOTOR_SPEED_MAX: 电机最高速度
    double load_monitor_threshold; // 主轴负载监控阈值
    double max_cutting_speed;      // 主轴最大切削转速
    double min_cutting_speed;      // 主轴最小切削转速
    int spindle_direction_enable;  // 主轴方向控制使能
} PAR_SPINDLE_SERVO;

// 伺服轴参数
typedef struct {
    int motor_encode_type;              // PAR_SV_MOTOR_ENCODE_TYPE: 编码器反馈类型
    double torque_filter_instruct_time; // PAR_SV_TORQUE_FILTER_INSTRUCT_TIME: 转矩指令滤波时间
    int reserved_533;                   // 保留参数 533
    int servo_version;                  // PAR_SV_SERVO_VERSION: 进给轴伺服版本号
    int reserved_535_542[8];            // 保留参数 535-542
    int motor_type;                     // PAR_SV_MOTOR_TYPE: 驱动器电机类型代码
    int reserved_544_555[12];           // 保留参数 544-555
} PAR_SERVO;

// NC 参数数据结构
typedef struct {
    double cycle; // PAR_NCU_CYCLE: 插补周期
} PAR_NCU;

// 机床参数数据结构
typedef struct {
    int servo_version; // PAR_SV_SERVO_VERSION: 进给轴伺服版本号
    int motor_type;    // PAR_SV_MOTOR_TYPE: 驱动器电机类型代码
} PAR_MAC;

// 通道参数数据结构
typedef struct {
    char name[32];           // PAR_CH_NAME: 通道名称
    int x_index;             // PAR_CH_XINDEX: X轴编号
    int y_index;             // PAR_CH_YINDEX: Y轴编号
    int z_index;             // PAR_CH_ZINDEX: Z轴编号
    int spdl0;               // PAR_CH_SPDL0: 主轴0编号
    char x_name[16];         // PAR_CH_X_NAME: X轴名
    char y_name[16];         // PAR_CH_Y_NAME: Y轴名
    char z_name[16];         // PAR_CH_Z_NAME: Z轴名
    char s0_name[16];        // PAR_CH_S0_NAME: 主轴0名
    double default_feed;     // PAR_CH_DEFAULT_F: 通道缺省进给速度
    double dryrun_spd;       // PAR_CH_DRYRUN_SPD: 空运行进给速度
    double mpg_acc_rate;     // PAR_CH_MPG_ACC_RATE: 手摇加速度系数
    double mpg_jk_rate;      // PAR_CH_MPG_JK_RATE: 手摇捷度系数
    double mpg_work_rate;    // PAR_CH_MPG_WORK_RATE: 手摇加工系数
    int vplan_mode;          // PAR_CH_VPLAN_MODE: 速度规划模式
    double micr_max_len;     // PAR_CH_MICR_MAX_LEN: 微线段上限长度
    double fine_stop_ang;    // PAR_CH_FINE_STOP_ANG: 准停转角阈值
    double inter_tangt_ang;  // PAR_CH_INTER_TANGT_ANG: 内切转角阈值
    double path_tolerance;   // PAR_CH_PATH_TOLERANCE: 轨迹轮廓允差
    double coner_dec_factor; // PAR_CH_CONER_DEC_FACTOR: 拐角降速比例因子
    double hspl_min_len;     // PAR_CH_HSPL_MIN_LEN: 微线段下限长度
    double hspl_max_ang;     // PAR_CH_HSPL_MAX_ANG: 样条过渡夹角
    double hspl_max_rat;     // PAR_CH_HSPL_MAX_RAT: 样条平滑相邻段最大长度比
    double hspl_max_len;     // PAR_CH_HSPL_MAX_LEN: 样条平滑最大线段长度
    double max_ecen_acc;     // PAR_CH_MAX_ECEN_ACC: 向心加速度
    double max_tang_acc;     // PAR_CH_MAX_TANG_ACC: 切向加速度
    double max_ecen_jerk;    // PAR_CH_MAX_ECEN_JERK: 向心加加速度
    double max_tang_jerk;    // PAR_CH_MAX_TANG_AJERK: 切向加加速度
    unsigned int lookahead_num;
} PAR_CHAN;

// 坐标轴参数数据结构
typedef struct {
    char name[32];         // PAR_AX_NAME: 轴名[显示用]
    int type;              // PAR_AX_TYPE: 轴类型[直线、摆动、回转、主轴]
    int index;             // PAR_AX_INDEX: 轴编号
    int modn;              // PAR_AX_MODN: 设备号
    int dev_i;             // PAR_AX_DEV_I: 设备接口
    int pm_munit;          // PAR_AX_PM_MUNIT: 电子齿轮比分子[每转位移量mm]
    int pm_pulse;          // PAR_AX_PM_PULSE: 电子齿轮比分母[每转指令脉冲数]
    double plmt;           // PAR_AX_PUSHT: 正软极限
    double nlmt;           // PAR_AX_NLMT: 负软极限
    double plmt2;          // PAR_AX_PUSHT2: 第2正软极限
    double nlmt2;          // PAR_AX_NLMT2: 第2负软极限
    int home_way;          // PAR_AX_HOME_WAY: 回参考点方式
    int home_dir;          // PAR_AX_HOME_DIR: 回参考点方向
    double home_mask;      // PAR_AX_HOME_MASK: Z脉冲屏蔽角度
    double home_hspd;      // PAR_AX_HOME_HSPD: 回参考点高速
    double home_lspd;      // PAR_AX_HOME_LSPD: 回参考点低速
    double home_crds;      // PAR_AX_HOME_CRDS: 参考点坐标值
    double g60_off;        // PAR_AX_G60_OFF: 单向定位(G60)偏移量
    double rot_rad;        // PAR_AX_ROT_RAD: 转动轴当量半径
    double jog_lowspd;     // PAR_AX_JOG_LOWSPD: 慢速点动速度
    double jog_fastspd;    // PAR_AX_JOG_FASTSPD: 快速点动速度
    double rapid_vel;      // PAR_AX_RAPID_SPD: 快移速度
    double rapid_acc;      // PAR_AX_RAPID_ACC: 快移加速度
    double rapid_jerk;     // PAR_AX_RAPID_JK: 快移捷度
    double feed_vel;       // PAR_AX_FEED_SPD: 进给速度
    double feed_acc;       // PAR_AX_FEED_ACC: 进给加速度
    double feed_jerk;      // PAR_AX_FEED_JK: 进给捷度
    double mpg_unit_spd;   // PAR_AX_MPG_UNIT_SPD: 手摇单位速度比例系数
    int mpg_resol;         // PAR_AX_MPG_RESOL: 手摇脉冲分辨率
    double mpg_stage_vel;  // PAR_AX_MPG_STAGE_VEL: 手摇分档速度
    int mpg_inte_time;     // PAR_AX_MPG_INTE_TIME: 手摇缓冲周期
    int mpg_smooth_time;   // PAR_AX_MPG_SMOOTH_TIME: 手摇过冲周期
    double mpg_spd;        // PAR_AX_MPG_SPD: 手摇进给速度
    double rapid_rate;     // PAR_AX_RAPID_RATE: 超速报警系数
    double follow_rate;    // PAR_AX_FOLLOW_RATE: 轴跟踪误差系数
    double default_s;      // PAR_AX_DEFAULT_S: 主轴缺省转速
    double spdl_max_speed; // PAR_SPDL_MAX_SPEED: 主轴最大转速
    double spdl_spd_tol;   // PAR_SPDL_SPD_TOL: 主轴转速允许转速波动率
    int spdl_spd_time;     // PAR_SPDL_SPD_TIME: 主轴转速到达允许最大时间
    double sp_ori_pos;     // PAR_AX_SP_ORI_POS: 进给主轴定向角度
    int sp_zero_tol;       // PAR_AX_SP_ZERO_TOL: 进给主轴零速允差[脉冲]
    double pos_tol;        // PAR_AX_POS_TOL: 定位允差
    double max_lag;        // PAR_AX_MAX_LAG: 最大跟随误差
    int ateeth;            // PAR_AX_ATEETH: 传动比分子[轴侧齿数]
    int mteeth;            // PAR_AX_MTEETH: 传动比分母[电机侧齿数]
    int mt_ppr;            // PAR_AX_MT_PPR: 电机每转脉冲数
    double pitch;          // PAR_AX_PITCH: 丝杆导程
    int rack_num;          // PAR_AX_RACK_NUM: 齿条齿数
    double rack_space;     // PAR_AX_RACK_SPACE: 齿条齿间距
    int worm_num;          // PAR_AX_WORM_NUM: 蜗杆头数
    double worm_space;     // PAR_AX_WORM_SPACE: 蜗杆齿距
    union {
        PAR_SPINDLE_SERVO sf;
        PAR_SERVO sv;
    };
    bool is_set_home_offset; // 设置编码器反馈偏置使能 wcf 2026.01.14
    double enc_off;         // 编码器反馈偏置量 wcf 2026.01.14
    double vel_limit;       // 电机最大速度上限 wcf 2026.01.14
    double max_ferror;      // 最大跟随误差 wcf 2026.01.14
    double min_ferror;      // 最小跟随误差 wcf 2026.01.14
} PAR_AXIS;

typedef struct {
    /* 反向间隙补偿参数 (0-4) */
    int bl_enable;                  /* 0: 反向间隙补偿类型 */
    double bl_value;                /* 1: 反向间隙补偿值 */
    double bl_rate;                 /* 2: 反向间隙补偿率 */
    double bl_value2;               /* 3: 快移反向间隙补偿值 */
    double bl_mpg_value;            /* 4: 手摇反向间隙补偿值 */

    /* 热误差补偿参数 (5-19) - 占位 */
    int heat_placeholders[15];

    /* 螺距误差补偿参数 (20-26) */
    int pitch_comp_type;            /* 20: 螺距误差补偿类型 */
    double pitch_comp_start_pos;    /* 21: 螺距误差补偿起点坐标 */
    int pitch_comp_num;             /* 22: 螺距误差补偿点数 */
    double pitch_comp_step;         /* 23: 螺距误差补偿点间距 */
    int pitch_comp_modulo;          /* 24: 螺距误差取模补偿使能 */
    double pitch_comp_factor;       /* 25: 螺距误差补偿倍率 */
    int pitch_comp_table;           /* 26: 螺距误差补偿表起始参数号 */

    int reserved_27_29[3];

    /* 第1项垂直度补偿参数 (30-33) */
    int perp1_comp_enable;          /* 30: 第1项垂直度补偿使能 */
    int perp1_comp_input_ax;        /* 31: 第1项垂直度补偿基准轴号 */
    double perp1_comp_refn;         /* 32: 第1项垂直度补偿参考点坐标 (单位：mm) */
    double perp1_comp_ang;          /* 33: 第1项垂直度补偿角度 (单位：um/m) */

    int reserved_34_39[6];

    /* 第2项垂直度补偿参数 (40-43) */
    int perp2_comp_enable;          /* 40: 第2项垂直度补偿使能 */
    int perp2_comp_input_ax;        /* 41: 第2项垂直度补偿基准轴号 */
    double perp2_comp_refn;         /* 42: 第2项垂直度补偿参考点坐标 (单位：mm) */
    double perp2_comp_ang;          /* 43: 第2项垂直度补偿角度 (单位：um/m) */

    int reserved_44_49[6];

    /* 第1项直线度补偿参数 (50-57) */
    int stra1_input_ax;             /* 50: 第1项直线度补偿基准轴号 */
    int stra1_type;                 /* 51: 第1项直线度补偿类型 */
    double stra1_start;             /* 52: 第1项直线度补偿起点坐标 */
    int stra1_num;                  /* 53: 第1项直线度补偿点数 */
    double stra1_step;              /* 54: 第1项直线度补偿点间距 */
    int stra1_modulo;               /* 55: 第1项直线度取模补偿使能 */
    double stra1_factor;            /* 56: 第1项直线度补偿倍率 */
    int stra1_table;                /* 57: 第1项直线度补偿表起始参数号 */

    int reserved_others[442];
} PAR_ACMP;

typedef struct {
    double reserve;
} PAR_CFG;

typedef struct {
    double reserve;
} PAR_TABLE;

typedef struct {
    PAR_NCU par_ncu;                   // NC 参数
    PAR_MAC par_mac;                   // 机床用户参数
    PAR_CFG par_cfg;                   // 设备接口参数
    PAR_TABLE par_table;               //（刀具和刀库）数据表参数
    PAR_CHAN par_chan[SYS_CHAN_NUM];   // 通道参数
    PAR_AXIS par_axis[TOTAL_AXES_NUM]; // 坐标轴参数
    PAR_ACMP par_acmp[TOTAL_AXES_NUM]; // 误差补偿参数
} PAR;

#ifdef __cplusplus
}
#endif
#endif // PARDATA_H
