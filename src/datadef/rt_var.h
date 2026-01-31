/********************************************************************
 * @file        var 变量.h
 * @brief       1、随控制指令而发送变化的数据，即每条G代码指令执行时都有可能发生变化。
                2、通过对特定的变量的访问，可以得到系统的很多状态数据。
 * @version     V1.01
 * @date        2026/01/05
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef RT_VAR_H
#define RT_VAR_H

#include <gncdatadef.h>
#include "stored_stroke_check.h"
#include "ferror_control.h"
#include "simple_tp.h"

#ifdef __cplusplus
extern "C" {
#endif

// 轴变量数据结构 (对应 VAR_AXIS_* 枚举)
typedef struct {
    char name[8];            /*!< 轴名 {Get(Bit8[PARAM_STR_LEN])} */
    int type;                /*!< 轴类型 {Get(Bit32)} */
    int chan;                /*!< 获取通道号 {Get(Bit32)} */
    int chan_index;          /*!< 获取在通道中的轴号 {Get(Bit32)} */
    int chan_sindex;         /*!< 获取在通道中的主轴号 {Get(Bit32)} */
    int lead;                /*!< 获取引导轴 {Get(Bit32)} */
    double act_pos;          /*!< 机床实际位置 {Get(Bit64)} */
    double act_pos2;         /*!< 机床实际位置2 {Get(Bit64)} */
    double cmd_pos;          /*!< 机床指令位置 {Get(Bit64)} */
    double act_pos_wcs;      /*!< 工件实际位置 {Get(Bit64)} */
    double cmd_pos_wcs;      /*!< 工件指令位置 {Get(Bit64)} */
    double act_pos_rcs;      /*!< 相对实际位置 {Get(Bit64)} */
    double cmd_pos_rcs;      /*!< 相对指令位置 {Get(Bit64)} */
    double act_pulse;        /*!< 实际脉冲位置 {Get(Bit64)} */
    double cmd_pulse;        /*!< 指令脉冲位置 {Get(Bit64)} */
    double prog_pos;         /*!< 编程位置 {Get(Bit64)} */
    double enc_cntr;         /*!< 电机位置 {Get(Bit64)} */
    double cmd_vel;          /*!< 指令速度 {Get(Bit64)} */
    double act_vel;          /*!< 实际速度 {Get(fBit64)} */
    double left_togo;        /*!< 工件剩余进给 {Get(Bit64)} */
    double wcs_zero;         /*!< 工件零点 {Get(Bit64)} */
    double wheel_off;        /*!< 手轮中断偏移量 {Get(Bit64)} */
    double follow_err;       /*!< 跟踪误差 {Get(Bit64)} */
    double syn_err;          /*!< 同步误差	{Get(Bit64)} */
    double comp;             /*!< 轴补偿值 {Get(Bit64)} */
    double zsw_dist;         /*!< Z脉冲偏移 {Get(Bit64)} */
    double real_zero;        /*!< 相对零点 {Get(Bit64)} */
    double motor_rev;        /*!< 电机转速 {Get(fBit64)} */
    double drive_cur;        /*!< 驱动单元电流 {Get(fBit64)} */
    double load_cur;         /*!< 负载电流 {Get(fBit64)} */
    double rated_cur;        /*!< 额定电流 {Get(fBit64)} */
    int is_homef;            /*!< 回零完成 {Get(Bit32)} */
    double wave_freq;        /*!< 波形频率 {Get(fBit64)} */
    char drive_ver[8];       /*!< 伺服驱动版本 {Get(Bit8[32])} */
    int motor_type;          /*!< 伺服类型 {Get(Bit8[32])} */
    int motor_type_flag;     /*!< 伺服类型出错标志 {Get(Bit32)} */
    double z_space1;         /*!< Z脉间距1 {Get(Bit64)} */
    double z_space2;         /*!< Z脉间距2 {Get(Bit64)} */
    double act_pos_ex;       /*!< 机床实际位置 {Get(fBit64)} */
    double act_pos2_ex;      /*!< 机床实际位置2 {Get(fBit64)} */
    double cmd_pos_ex;       /*!< 机床指令位置 {Get(fBit64)} */
    double act_pos_wcs_ex;   /*!< 工件实际位置 {Get(fBit64)} */
    double cmd_pos_wcs_ex;   /*!< 工件指令位置 {Get(fBit64)} */
    double act_pos_rcs_ex;   /*!< 相对实际位置 {Get(fBit64)} */
    double cmd_pos_rcs_ex;   /*!< 相对指令位置 {Get(fBit64)} */
    double prog_pos_ex;      /*!< 编程位置 {Get(fBit64)} */
    double cmd_vel_ex;       /*!< 指令速度 {Get(fBit64)} */
    double act_vel_ex;       /*!< 实际速度 {Get(fBit64)} */
    double left_togo_ex;     /*!< 工件系剩余进给 {Get(fBit64)} */
    double wcs_zero_ex;      /*!< 工件零点 {Get(fBit64)} */
    double wheel_off_ex;     /*!< 手轮中断偏移量 {Get(fBit64)} */
    double follow_err_ex;    /*!< 跟踪误差 {Get(fBit64)} */
    double syn_err_ex;       /*!< 同步误差	{Get(fBit64)} */
    double comp_ex;          /*!< 轴补偿值 {Get(fBit64)} */
    double zsw_dist_ex;      /*!< Z脉冲偏移 {Get(fBit64)} */
    double real_zero_ex;     /*!< 相对零点 {Get(fBit64)} */
    int dev_type;            /*!< 轴设备类型	{Get(Bit32)} */
    int vendorid;            /*!< 轴厂家号(目前仅ECAT) {Get(Bit32)} */
    int productcode;         /*!< 轴产品号(目前仅ECAT) {Get(Bit32)} */
    double act_pos_2;        /*!< 第2编码器计算实际位置 {Get(Bit64)} */
    double act_vel_2;        /*!< 第2编码器计算实际速度 {Get(fBit64)} */
    double power;            /*!< 实际功率 {Get(fBit64)} */
    double mcs_left_togo;    /*!< 机床系剩余进给 {Get(fBit64)} */
    double mcs_left_togo_ex; /*!< 机床系剩余进给 {Get(fBit64)} */
    double total;
    double mpg_spd;
    AxisLimitRuntime runtime;
    simple_tp_t jog_tp;                // 点动控制规划器 // wcf add 2026.01.22
    ferror_control_t ferror_control;   // 跟随误差控制器 // wcf add 2026.01.22
    double motor_cmd_pos;              // 带误差补偿下发至驱动器的指令位置 // wcf add 2026.01.22
    double motor_act_pos;              // 带误差补偿的驱动器反馈位置 // wcf add 2026.01.22

    double pitch_comp_val;             // 当前位置螺距误差补偿值 (单位:um)  // wcf add 2026.01.23
    double perp_comp_val;              // 当前位置垂直度误差补偿值 (单位:mm)  // wcf add 2026.01.23
} VAR_AXIS;

// 通道变量数据结构 (对应 VAR_CHAN_* 枚举)
typedef struct {
    int is_exist;          /*!< 通道是否存在 {Get(Bit32)} */
    int mac_type;          /*!< 通道的机床类型 {Get(Bit32)} */
    int axes_mask;         /*!< 轴掩码 {Get(Bit32)} */
    int axes_mask1;        /*!< 轴掩码1 {Get(Bit32)} */
    char name[8];          /*!< 通道名 {Get(Bit8[PARAM_STR_LEN])} */
    int cmd_type;          /*!< 读取当前G代码的标志 {Get(Bit32)} */
    double cmd_feedrate;   /*!< 指令进给瞬时速度 {Get(fBit64)} */
    double act_feedrate;   /*!< 实际进给速度 {Get(fBit64)} */
    double prog_feedrate;  /*!< 编程指令速度 {Get(fBit64)} */
    double feed_override;  /*!< 进给修调 {Get(Bit64)} */
    double rapid_override; /*!< 快移修调 {Get(Bit64)} */
    double cmd_acc;        // 仿真所需 许晓婷 2026.01.05
    double act_acc;        // 仿真所需 许晓婷 2026.01.05
    double cmd_jerk;       // 仿真所需 许晓婷 2026.01.05
    double act_jerk;       // 仿真所需 许晓婷 2026.01.05
    int mcode;             /*!< 通道的M指令 {Get(Bit32)} */
    int tcode;             /*!< 通道的T指令 {Get(Bit32)} */
    int toffs;             /*!< 通道中的刀偏号 {Get(Bit32)} */
    int tool_use;          /*!< 当前刀具 {Get(Bit32)} */
    int tool_rdy;          /*!< 准备好交换的刀具 {Get(Bit32)} */
    int mode;              /*!< 模式(返回值数据定义见下面) {Get(Bit32)} */
    int is_mdi;            /*!< MDI {Get(Bit32)} */
    int cycle;             /*!< 循环启动 {Get(Bit32); Set(NULL)} */
    int hold;              /*!< 进给保持 {Get(Bit32); Set(NULL)} */
    char is_progsel[8];    /*!< 已选程序 {Get(Bit32)} */
    int is_progend;        /*!< 程序运行完成 {Get(Bit32)} */
    int is_parsering;
    int parser_reset;
    int is_threading;      /*!< 螺纹加工 {Get(Bit32)} */
    int is_rigid;          /*!< 刚性攻丝 {Get(Bit32)} */
    int is_rewinded;       /*!< 重运行复位状态 {Get(Bit32)} */
    int is_estop;          /*!< 急停 {Get(Bit32)} */
    int is_resetting;      /*!< 复位 {Get(Bit32)} */
    int is_running;        /*!< 运行中 {Get(Bit32)} */
    int is_homing;         /*!< 回零中 {Get(Bit32)} */
    int is_moving;         /*!< 轴移动中 {Get(Bit32)} */
    int diameter;          /*!< 直半径编程 {Get(Bit32)} */
    int verify;            /*!< 校验 {Get(Bit32); Set(Bit32)} */
    int run_row;           /*!< 运行行 {Get(Bit32)} */
    int dcd_row;           /*!< 译码行 {Get(Bit32)} */
    int sel_prog;          /*!< 选择程序的编号 {Get(Bit32)} */
    int run_prog;          /*!< 运行程序的编号 {Get(Bit32)} */
    int part_cntr;         /*!< 加工计数 {Get(Bit32); Set(Bit32)} */
    int part_stati;        /*!< 工件总数 {Get(Bit32); Set(Bit32)} */
    int hmi_reset;         /*!< HMI复位 {Set(NULL)} */
    int chg_prog;          /*!< 程序修改标志 {Set(NULL)} */
    int period_total;      /*!< 过时的 */
    int lax;               /*!< 通道轴对应的逻辑轴号 {Get(Bit32)} */
    char axis_name[8];     /*!< 编程轴名 {Get(Bit8[PARAM_STR_LEN])} */
    char spdl_name[8];     /*!< 编程主轴名 {Get(Bit8[PARAM_STR_LEN])} */
    int modal;             /*!< 通道模态 共80组 {Get(Bit32)} */
    int spdl_lax;          /*!< 通道主轴对应的逻辑轴号，动态 {Get(Bit32)} */
    int spdl_para_lax;     /*!< 通道主轴对应的逻辑轴号，静态 {Get(Bit32)} */
    double cmd_spdl_speed; /*!< 主轴指令速度 {Get(fBit64)} */
    double act_spdl_speed; /*!< 主轴实际速度 {Get(fBit64)} */
    double spdl_override;  /*!< 主轴修调 {Get(Bit64)} */
    double do_hold;        /*!< 设置进给保持  */
    double bp_pos;         /*!< 断点位置 {Get(Bit64)} */
    int prog_flow;         /*!< 正在调用手动子程序 {Get(Bit32)} */
    int h_off;             /*!< 当前使用刀具长度补偿号 {Get(Bit32)} */
    int d_off;             /*!< 当前使用刀具半径补偿号 {Get(Bit32)} */
    int bp_pos_ex;
    int is_sbl;               /*!< 是否单段模式 {Get(Bit32)} */
    int part_need;            /*!< 加工需求数 {Get(Bit32); Set(Bit32)} */
    int s_code;               /*!< 通道中主轴转速 {Get(Bit32); Set(Bit32)} */
    int f_g94_speed;          /*!< 通道中记录的F分进给模态速度 {Get(Bit64); Set(Bit64)} */
    int f_g95_speed;          /*!< 通道中记录的F转进给模态速度 {Get(Bit64); Set(Bit64)} */
    int ext_file_state;       /*!< 界面告诉内核文件写/删状态(仿FANUC的DPRINT指令用) */
    int refresh_run_info;     /*!< 刷新程序行信息 */
    int mdi_in_exit_able;     /*!< 是否可以进入/退出MDi状态 */
    double g50_zero;          /*!< G50零点偏置坐标 {Get(fBit64)}*/
    double cmd_feedrate_mean; /*!< 指令进给平均速度 {Get(fBit64)} */
    int heat_forecast_k;      /*!< 热误差预测斜率K {Get(Bit64); Set(Bit64)} */
    int heat_forecast_b;      /*!< 热误差预测零点偏置B {Get(Bit64); Set(Bit64)} */
    int prog_dcd_row;
    int dcd_idx;
    int refresh_dcd_info;
    int cycle_l;          /*!< 固定循环剩余调用次数 */
    int axis_act_wcs;     /*!< 工件实际位置 {Get(Bit32)} >*/
    int abs_act_pos;      /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    int abs_cmd_pos;      /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    int abs_act_pos_safe; /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    int abs_cmd_pos_safe; /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    int total;
} VAR_CHANNEL;

// 系统变量数据结构 (对应 VAR_SYS_* 枚举)
typedef struct {
    int chan_num;           /*!< 获取系统通道数 {Get(Bit32)} */
    int move_unit;          /*!< 长度分辨率 {Get(Bit64)} */
    int turn_unit;          /*!< 角度分辨率 {Get(Bit64)} */
    int metirc_disp;        /*!< 公英制 {Get(Bit32)} */
    int show_time;          /*!< 是否显示时间 {Get(Bit32)} */
    int pop_alarm;          /*!< 报警自动显示 {Get(Bit32)} */
    int graph_erase;        /*!< 图形自动擦除 {Get(Bit32)} */
    int mac_type;           /*!< 机床类型 */
    int prec;               /*!< 坐标系精度 {Get(Bit64)} */
    int f_prec;             /*!< F精度 {Get(Bit64)} */
    int s_prec;             /*!< S精度 {Get(Bit64)} */
    int nck_ver;            /*!< NCK版本 {Get(Bit8[32])} */
    int drv_ver;            /*!< DRV版本 {Get(Bit8[32])} */
    int plc_ver;            /*!< PLC版本 {Get(Bit8[32])} */
    int cn_ver;             /*!< CNC版本(2.00不再使用) {Get(Bit32) Set(Bit32)} */
    int mcp_key;            /*!< MCP面板钥匙开关 {Get(Bit32)} */
    int active_chan;        /*!< 活动通道 {Get(Bit32) Set(Bit32)} */
    int request_chan;       /*!< 请求通道 {Get(Bit32)} */
    int mdi_chan;           /*!< (无效)MDI运行通道 {Get(Bit32)} */
    int request_chan_mask;  /*!< 请求的通道屏蔽字 {Get(Bit32)} */
    int chan_mask;          /*!< 通道屏蔽字 {Set(Bit32)} */
    int plc_stop;           /*!< plc停止 {Set(NULL)} */
    int poweroff_act;       /*!< 断电应答 {Set(NULL)} */
    int is_hold_redecode;   /*!< 进给保持后是否重新解释 {Get(Bit32)} */
    char nc_ver[8];         /*!< NC版本(API版本) {Get(Bit8[32])} */
    char sn_num[8];         /*!< 数控装置SN号 {Get(Bit8[32])} */
    char machine_type[8];   /*!< 机床型号 {Get(Bit8[48]) Set(Bit8[48])} */
    char machine_info[8];   /*!< 机床信息 {Get(Bit8[48]) Set(Bit8[48])} */
    char macfac_info[8];    /*!< 机床厂信息 {Get(Bit8[48]) Set(Bit8[48])} */
    char user_info[8];      /*!< 用户信息 {Get(Bit8[48]) Set(Bit8[48])} */
    char machine_num[8];    /*!< 机床编号 {Get(Bit8[48]) Set(Bit8[48])} */
    char exfactory_date[8]; /*!< 出厂时间 {Get(Bit8[32])} */
    int access_level;       /*!< 权限等级 {Get(Bit32)} */
    int mul_chan_reset;     /*!< 多通道复位流程启动标记 {Set(Bit32)[0/1]} */
    int sys_rights_key;     /*!< 权限钥匙锁 {Get(Bit32)} */
    int reg_days_remain;    /*!< 注册剩余天数 {Get(Bit32)} */
    int plc2_cur_cycle;     /*!< PLC2当前运行周期 {Get(Bit32)} */
    int plc2_min_cycle;     /*!< PLC2最小运行周期 {Get(Bit32)} */
    int plc2_max_cycle;     /*!< PLC2最大运行周期 {Get(Bit32)} */
    int plc_online;         /*!< PLC在线调试 {Get(Bit8[48]) Set(Bit8[48])} */
    char os_ver[8];         /*!< OS(操作系统)版本 {Get(Bit8[64])} */
    char hnc_ver[8];        /*!< HNC版本(系统版本) {Get(Bit8[64]) Set(Bit8[64])} */
    char fpga_ver[8];       /*!< FPGA版本 {Get(Bit8[64])} */
    char parm_ver[8];       /*!< 参数文件版本(主版本+副版本) {Get(Bit8[64])} */
    char plcfile_ver[8];    /*!< PLC文件版本(主版本+副版本) {Get(Bit8[64])} */
    char plconoff_ver[8];   /*!< PLC开关文件版本 {Get(Bit8[64])} */
    char plcalarm_ver[8];   /*!< PLC报警文件版本 {Get(Bit8[64])} */
    char usrp_ver[8];       /*!< 用户P参数文件版本 {Get(Bit8[64])} */
    char cycle_ver[8];      /*!< 固定循环文件版本(铣MILLING.CYC;车TURNING.CYC) {Get(Bit8[64])} */
    char usrcycle_ver[8];   /*!< 用户自定义循环文件版本(USERDEF.CYC) {Get(Bit8[64])} */
    int total;
} VAR_SYSTEM;

// 总的变量数据结构
typedef struct {
    VAR_AXIS var_axis[TOTAL_AXES_NUM];     // 轴变量数组 (进给轴 + 主轴)
    VAR_CHANNEL var_channel[SYS_CHAN_NUM]; // 通道变量数组
    VAR_SYSTEM var_system;                 // 系统变量
} RT_VAR;

#ifdef __cplusplus
}
#endif
#endif // VARDATA_H
