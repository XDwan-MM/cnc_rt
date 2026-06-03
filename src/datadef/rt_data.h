/******************************************************************** 
 * @file        rt_sys.h
 * @brief       实时系统数据
 * @version     V1.00
 * @date        2026/02/27
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 * @details     该文件下的数据如果存在多线程访问，请加锁使用（使用get set方法）；如为单线程使用，可直接进行操作
********************************************************************/

#ifndef __RT_DATA_H__
#define __RT_DATA_H__

#include <stdbool.h>
#include <pthread.h>
#include <gncdatadef.h>
#include "datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_STATE_NUM 100
#define CHAN_STATE_NUM 100

// 路径和轨迹线程使用
typedef enum{
    SYS_IS_READY = 0,           // 准备就绪
    SYS_IS_ESTOP_ON,            // 是否急停
    SYS_IS_ESTOP_PRESS,         // 急停按键是否按下
    SYS_IS_ALARMING,            // 是否正在报警中
    SYS_IS_SWITCHING_MODE,      // 是否正在切换模式
    SYS_IS_MAC_LOCK,            // 机床锁住 
    SYS_IS_MACK_LOCK_READY,     // 机床锁住完成
    SYS_IS_DRY_RUN,             // 空运行
    SYS_IS_BLOCK_SKIP,          // 程序段跳过（路径和轨迹线程使用）
    SYS_IS_SINGLE_BLOCK,        // 单段
    SYS_IS_MSBT_LOCK,           // 辅助功能锁住
    SYS_IS_RAPID,               // 是否快速
} SysStateIndex;

// 路径和轨迹两个线程使用
typedef enum{
    CHAN_IS_PARSER_RESET = 0,    // 译码器复位  is_parser_reset
    CHAN_IS_PARSERING,           // 解释器是否运行中 is_parsering
    CHAN_IS_PARSER_RERUN,        // 译码器是否运行中 is_parser_running
    CHAN_IS_GCODE_END,           // G代码是否结束 is_gcode_end   
    CHAN_IS_MCODE_END,           // M代码是否结束 is_mcode_end
    CHAN_IS_FIRST_RUN,           // 是否为第一次运行 is_first_run
    CHAN_IS_EXACT_STOP,          // 模式到达检测 is_exact_stop
    CHAN_IS_MODE_ARRIVE,         // 模式到达参考店检测 is_mode_arrive
} ChanStateIndex;

typedef struct {
    double last_pos;
    double current_pos;
    bool is_moving;              //是否在运动  1运动　0静止
    bool direction;              //运动方向　　１正向  0负向
    bool is_init;
    bool is_error;               //驱动器是否报警 １是　0否
    bool is_decel_done;          //报警减速是否完成
    bool is_fwd_limit;           //手动正方向限制
    bool is_rev_limit;           //手动负方向限制
    bool plc_is_disable;       
    bool is_disable;             //轴是否被屏蔽
} AxisStatus;

// 目前仅轨迹线程使用
typedef struct{
    double act_acc;
    double act_jk;
    double cmd_acc;
    double cmd_jk;
    double wcs_cmd_pos_cur;     // 工件插补计算位置
    double wcs_cmd_pos_pre;     // 工件插补计算位置(上一插补)
    double mac_cmd_pos;         // 机床指令位置
    double mac_comp_pos;        // 机床补偿后位置
    double motor_cmd_pos;       // 驱动器指令位置（带补偿）
    double motor_act_pos;       // 驱动器反馈位置（带补偿）
    double fb_pos_no_comp;      // 反馈位置（去除补偿）
    double pitch_comp_val;      // 当前位置螺距误差补偿值 (单位:um)  // wcf add 2026.01.23
    double perp_comp_val;       // 当前位置垂直度误差补偿值 (单位:mm)  // wcf add 2026.01.23
    AxisStatus axis_status;
}RT_AXIS;

// 目前仅轨迹线程使用
typedef struct 
{
    double act_acc;
    double act_jk;
    double cmd_acc;
    double cmd_jk;
}RT_CHAN;

typedef struct 
{
    bool is_resetting;
    bool is_resetted;
    bool is_reset_delaying;
    int reset_tic;
}SYS_RESET;

typedef struct {
    int parser_error_line;
    int parser_error_no;
} PARSER_ERROR_DATA;

typedef struct {
    SYS_EVENT nc_event;
    SYS_RESET nc_reset;
    NC_MODE target_mode; 
    NC_MODE current_mode; 
    bool sys_state[SYS_STATE_NUM];
    bool chan_state[SYS_CHAN_NUM][CHAN_STATE_NUM];
    RT_AXIS axis_data[SYS_AXES_NUM];
    RT_CHAN chan_data[SYS_CHAN_NUM];
    pthread_mutex_t mutex;
    PARSER_ERROR_DATA parser_error;
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
void nc_set_resetting();
void nc_set_resetted();

#ifdef __cplusplus
}
#endif
#endif
