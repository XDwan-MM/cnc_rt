#include "global.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <shmapi.h>
#include <vardatadef.h>
#include <varapi.h>
#include <pardata.h>
#include <parapi.h>
#include <sysapi.h>
#include <sysdata.h>
#include <hmisysapi.h>
#include <pthread.h>

// ==============全局变量定义===============
sem_t exit_sem;                         // 退出信号
sem_t op_sem;                           // 已进入op信号，用于线程间通信
int InterruptFlag = 0;                  // 打断标志位
Slave_info* slave_list = NULL;          // 定义从站信息结构体以存储解析结果
int update_pos_flag = 0;                // 理论位置读取驱动器位置
int rc = 0;
PdoWarn PdoWarnInfo; //pdo警告信息结构体
int slavernum = 0;
uint16_t* statis;
uint32_t* pos;
uint32_t* pos_cmd;
uint16_t* error_code;

TxEntry_Unit DI0[MAX_IO_MODULES];
TxEntry_Unit DI1[MAX_IO_MODULES];
RxEntry_Unit DO0[MAX_IO_MODULES];
RxEntry_Unit DO1[MAX_IO_MODULES];

TxEntry_Unit StatusWord[MAX_DRIVERS];      // 状态字
TxEntry_Unit CurPos[MAX_DRIVERS];          // 当前位置
TxEntry_Unit CurVel[MAX_DRIVERS];          // 当前速度
TxEntry_Unit CurTor[MAX_DRIVERS];          // 当前转矩
TxEntry_Unit ErrorNumb[MAX_DRIVERS];       // 报警码

RxEntry_Unit ControlWord[MAX_DRIVERS];     // 控制字
RxEntry_Unit PurPos[MAX_DRIVERS];          // 目标位置
RxEntry_Unit PurVel[MAX_DRIVERS];          // 目标速度
RxEntry_Unit PurTor[MAX_DRIVERS];          // 目标转矩
RxEntry_Unit OperationMode[MAX_DRIVERS];   // 操作模式

TxEntry_Unit MCPDI[11];
RxEntry_Unit MCPDO[7];

// 共享内存
VarData* var_data = NULL;                                   // var 数据
VarUnion (*var_axis)[VAR_AXIS_NUM] = NULL;
VarUnion (*var_chan)[VAR_CHAN_NUM] = NULL;
VarUnion* var_sys = NULL;
ParData* par_data = NULL;                                   // par数据
ParamUnit *par_ncu = NULL;                                  // NC 参数
ParamUnit *par_mac = NULL;                                  // 机床用户参数
ParamUnit (*par_chan)[PARAMAN_FILE_CHAN_MAX] = NULL;        // 通道参数
ParamUnit (*par_axis)[PARAMAN_FILE_AXIS_MAX] = NULL;        // 坐标轴参数
ParamUnit (*par_acmp)[PARAMAN_FILE_ACMP_MAX] = NULL;        // 误差补偿参数
ParamUnit (*par_cfg)[PARAMAN_FILE_CFG_MAX] = NULL;          // 设备接口参数
ParamUnit *par_table = NULL;                                // 刀具和刀库参数
SysData* sys_data = NULL; 

// 实时数据
RT_DATA* rt_data;               // 实时数据
GCODE_QUEUE *gcode_queue;       // 译码队列
MCODE_QUEUE *mcode_queue;       // 译码队列
PLAN_QUEUE *pretreat_queue;     // 预处理队列
PLAN_QUEUE *plan_queue;         // 插补队列
EVENT_QUEUE* event_queue;       // 事件队列
GCODE_QUEUE *gcode_queue;       // 译码队列
MCODE_QUEUE *mcode_queue;       // 译码队列
PLAN_QUEUE *pretreat_queue;     // 预处理队列
PLAN_QUEUE *plan_queue;         // 插补队列
COORD_TP *cycle_tp;             // 循环轨迹规划
HOMING_TP *homing_tp;           // 回零轨迹规划
MCODE_DATA *mcode_data;

// 读写共享内存锁（路径规划读时加锁）
pthread_mutex_t shm_mutex;

static GNC_RES shm_var_init(); 
static GNC_RES shm_par_init();
static GNC_RES tp_init();

GNC_RES rt_global_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    res = shm_var_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = shm_par_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = rt_data_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = tp_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = task_mq_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    pthread_mutex_init(&shm_mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES shm_var_init(){
    var_data = getVarDataPtr();
    if(var_data == NULL) return GNC_DATA_NULL;
    var_axis = var_data->var_axis;
    var_chan = var_data->var_channel;
    var_sys = var_data->var_system;
    // 初始化数据
    setVarSysInt(VAR_SYS_CHAN_NUM, 1, 0);        // 通道数                     
    // 通道数据
    for(int i = 0; i < SYS_CHAN_NUM; i++){        
        setVarChanInt(i, VAR_CHAN_CMD_FEEDRATE, 0, 0);          // 通道合成指令速度
        setVarChanInt(i, VAR_CHAN_ACT_FEEDRATE, 0, 0);          // 通道实际合成速度
        setVarChanDouble(i, VAR_CHAN_FEED_OVERRIDE, 0.0, 0);    // 进给倍率    
        setVarChanDouble(i, VAR_CHAN_RAPID_OVERRIDE, 0.0, 0);   // 快速进给倍率
        setVarChanDouble(i, VAR_CHAN_SPDL_OVERRIDE, 0.0, 0);    // 主轴倍率
        setVarChanInt(i, VAR_CHAN_IS_HOMING, 0, 0);             // 回零中
        setVarChanInt(i, VAR_CHAN_IS_RUNNING, 0, 0);            // 运行中
        setVarChanInt(i, VAR_CHAN_IS_PROGEND, 0, 0);            // 程序结束
        setVarChanInt(i, VAR_CHAN_IS_ESTOP, 0, 0);              // 急停
        setVarChanInt(i, VAR_CHAN_IS_RESETTING, 0, 0);          // 复位中
        setVarChanInt(i, VAR_CHAN_IS_MOVING, 0, 0);             // 轴移动中
    }
    // 坐标轴数据
    for(int i = 0; i < SYS_AXES_NUM; i++){    
        setVarAxisDouble(i, VAR_AXIS_CMD_VEL, 0, 0);            // 轴指令速度
        setVarAxisDouble(i, VAR_AXIS_ACT_VEL, 0, 0);            // 轴实际速度
        setVarAxisDouble(i, VAR_AXIS_CMD_POS_EX, 0, 0);         // 轴指令位置
        setVarAxisDouble(i, VAR_AXIS_ACT_POS_EX, 0, 0);         // 轴实际位置   
    }
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES shm_par_init(){
    par_data = getParDataPtr();
    if(par_data == NULL) return GNC_DATA_NULL;
    par_ncu = par_data->par[0].par_ncu;            // NC 参数
    par_mac = par_data->par[0].par_mac;            // 机床用户参数
    par_cfg = par_data->par[0].par_cfg;            // 设备接口参数
    par_table = par_data->par[0].par_table;        //（刀具和刀库）数据表参数
    par_chan = par_data->par[0].par_chan;          // 通道参数
    par_axis = par_data->par[0].par_axis;          // 坐标轴参数
    par_acmp = par_data->par[0].par_acmp;          // 误差补偿参数
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES tp_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    homing_tp_init();
    cycle_tp_init();
    res = mcode_data_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    gcode_queue = (GCODE_QUEUE*)malloc(sizeof(GCODE_QUEUE));
    res = gcode_queue_init(gcode_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    pretreat_queue = (PLAN_QUEUE*)malloc(sizeof(PLAN_QUEUE));
    res = queue_init(pretreat_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    plan_queue = (PLAN_QUEUE*)malloc(sizeof(PLAN_QUEUE));
    res = queue_init(plan_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES tp_reset(){
    homing_tp_reset(homing_tp);
    cycle_tp_reset();
    gcode_queue_reset(gcode_queue);
    queue_reset(pretreat_queue);
    queue_reset(plan_queue);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES shm_var_update(){
    VarUnion crds_type_var = {0};
    double crds_remain = 0.0;
    double crds_oft = 0.0;
    double act_pos_wcs_ex = 0.0;
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        if(rt_data->current_mode == MODE_MEM || rt_data->current_mode == MODE_MDI || rt_data->current_mode == MODE_DNC){
            // 剩余移动量
            crds_remain = var_axis[i][VAR_AXIS_PROG_POS_EX].value.f - var_axis[i][VAR_AXIS_CMD_POS_EX].value.f;
            // 运行模态
            VarUnion set_modal;
            set_modal.type = 0;
            for (int i = 0; i < MODAL_GROUP; ++i)
            {
                set_modal.value.i = cycle_tp->plan_block->state.modal[i];
                setVarChanValue(0, VAR_CHAN_MODAL, i, &set_modal, 0);
            }
            // 机床指令位置
            if(queue_is_empty(plan_queue)){
                setVarAxisDouble(AXIS_X, VAR_AXIS_PROG_POS_EX, var_axis[0][VAR_AXIS_CMD_POS_EX].value.f, 0);
                setVarAxisDouble(AXIS_Y, VAR_AXIS_PROG_POS_EX, var_axis[1][VAR_AXIS_CMD_POS_EX].value.f, 0);
                setVarAxisDouble(AXIS_Z, VAR_AXIS_PROG_POS_EX, var_axis[2][VAR_AXIS_CMD_POS_EX].value.f, 0);
            }else{
                setVarAxisDouble(AXIS_X, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.x, 0);
                setVarAxisDouble(AXIS_Y, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.y, 0);
                setVarAxisDouble(AXIS_Z, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.z, 0);
            }
        }else{
            crds_remain = 0;
        }
        // 剩余移动
        setVarAxisDouble(i, VAR_AXIS_LEFT_TOGO_EX, crds_remain, 0);
        // 绝对坐标
        getVarChanValue(0, VAR_CHAN_MODAL, 14, &crds_type_var, 0);
        getCrdsValue(CRDS_TYPE_G5X + crds_type_var.value.i - 1, i, &crds_oft);
        act_pos_wcs_ex = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f - crds_oft;
        setVarAxisDouble(i, VAR_AXIS_ACT_POS_WCS_EX, act_pos_wcs_ex, 0);
        // 轴速度等参数
        setVarAxisDouble(i, VAR_AXIS_ACT_VEL, var_chan[0][VAR_AXIS_CMD_VEL].value.f, 0);
        rt_data->axis_data[i].act_acc = rt_data->axis_data[i].cmd_acc;
        rt_data->axis_data[i].act_jk = rt_data->axis_data[i].cmd_jk;
        // 相对坐标
        double real_zero = 0.0;
        getVarAxisDouble(i, VAR_AXIS_REAL_ZERO_EX, &real_zero, 0);
        double real = var_axis[i][VAR_AXIS_CMD_POS_EX].value.f - real_zero;
        setVarAxisDouble(i, VAR_AXIS_ACT_POS_RCS_EX, real, 0);
        // 判断各轴零点
        if(fabs(par_axis[i][PAR_AX_HOME_CRDS].value.val_real - var_axis[i][VAR_AXIS_CMD_POS_EX].value.f) > HOMEF_TOL){
            setVarAxisInt(i, VAR_AXIS_IS_HOMEF, 0, 0);
        }
    }
    for (int i = 0; i < SYS_CHAN_NUM; i++)
    {
        // 运行模式
        setVarChanInt(0, VAR_CHAN_MODE, rt_data->current_mode, 0);
        // 主轴转速
        setVarChanDouble(i, VAR_CHAN_ACT_SPDL_SPEED, var_chan[i][VAR_CHAN_ACT_SPDL_SPEED].value.f, 0);
        // 通道速度等参数
        setVarChanDouble(i, VAR_CHAN_ACT_FEEDRATE, var_chan[i][VAR_CHAN_CMD_FEEDRATE].value.f, 0);
        rt_data->chan_data[i].act_acc = rt_data->chan_data[i].cmd_acc;
        rt_data->chan_data[i].act_jk = rt_data->chan_data[i].cmd_jk;
    }
}

// todo：待删除
// static GNC_RES shm_par_init(){
//     ParamUnit par_union = {};
//     // NC参数
//     getParData(PARAMAN_FILE_NCU, 0, PAR_NCU_CYCLE, &par_union);
//     // 插补周期1ms
//     par_ncu.cycle = (double)par_union.value.val_uint / TIME_US_2_S;
//     // 通道参数
//     for(int i = 0; i < SYS_CHAN_NUM; i++){
//         getParData(PARAMAN_FILE_CHAN, i, PAR_CH_DEFAULT_F, &par_union);
//         par_chan[i].default_feed = par_union.value.val_real / TIME_MINUS_2_S;            // 通道缺省进给速度
//         getParData(PARAMAN_FILE_CHAN, i, PAR_CH_DRYRUN_SPD, &par_union);
//         par_chan[i].dryrun_spd = par_union.value.val_real / TIME_MINUS_2_S;              // 空运行进给速度
//         getParData(PARAMAN_FILE_CHAN, i, PAR_CH_VPLAN_MODE, &par_union);
//         par_chan[i].vplan_mode = par_union.value.val_uint;              // 速度规划模式 0:T_SHAPE
//         getParData(PARAMAN_FILE_CHAN, i, PAR_CH_MAX_ECEN_ACC, &par_union);
//         par_chan[i].max_tang_acc = par_union.value.val_real;            // 最大向心加速度
//         getParData(PARAMAN_FILE_CHAN, i, PAR_CH_LOOKAHEAD_NUM, &par_union);
//         par_chan[i].lookahead_num = par_union.value.val_uint;           // 速度规划前瞻段数
//     }
//     // 坐标轴参数
//     par_axis[AXIS_X].name[0] = 'X';               // 轴名
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_PLMT, &par_union);
//     par_axis[AXIS_X].plmt = par_union.value.val_real;  // X轴正软极限
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_NLMT, &par_union);
//     par_axis[AXIS_X].nlmt = par_union.value.val_real;  // X轴负软极限
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_HOME_WAY, &par_union);
//     par_axis[AXIS_X].home_way = par_union.value.val_uint;             // X轴回参考点方式(0:绝对式 1:相对式)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_HOME_DIR, &par_union);
//     par_axis[AXIS_X].home_crds = par_union.value.val_real;            // X轴零点位置
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_HOME_CRDS, &par_union);
//     par_axis[AXIS_X].home_dir = par_union.value.val_uint;             // X轴回参考点方向(0:正向 1:负向)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_RAPID_SPD, &par_union);
//     par_axis[AXIS_X].rapid_vel = par_union.value.val_real / TIME_MINUS_2_S;            // X轴快移速度 unit:mm/s
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_FEED_SPD, &par_union);
//     par_axis[AXIS_X].feed_vel = par_union.value.val_real / TIME_MINUS_2_S;             // X轴最高进给速度
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_ATEETH, &par_union);
//     par_axis[AXIS_X].ateeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_MTEETH, &par_union);
//     par_axis[AXIS_X].mteeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_PITCH, &par_union);
//     par_axis[AXIS_X].pitch = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_MT_PPR, &par_union);
//     par_axis[AXIS_X].mt_ppr = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_MPG_SPD, &par_union);
//     par_axis[AXIS_X].mpg_spd = par_union.value.val_real;
//     par_axis[AXIS_X].pos_tol = 0.01;              // X轴定位允差
//     //　以下 wcf maintain 2026.02.04
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_RAPID_ACC, &par_union);
//     par_axis[AXIS_X].rapid_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // X轴快移加速度上限 (HMI 快移加减速时间常数(单位：ms), default:64ms | 实时 快移加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_RAPID_JK, &par_union);
//     par_axis[AXIS_X].rapid_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // X轴快移捷度上限 (HMI 快移加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 快移捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_FEED_ACC, &par_union);
//     par_axis[AXIS_X].feed_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // X轴加工加速度上限 (HMI 加工加减速时间常数(单位：ms), default:64ms | 实时 加工加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_FEED_JK, &par_union);
//     par_axis[AXIS_X].feed_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // X轴加工捷度上限 (HMI 加工加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 加工捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_PM_MUNIT, &par_union);
//     par_axis[AXIS_X].pm_munit = CONV_UM_TO_MM(par_union.value.val_int); // X轴电子齿轮比分子 (HMI unit: um, default:16000um | 实时 单位: mm)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_PM_PULSE, &par_union);
//     par_axis[AXIS_X].pm_pulse = par_union.value.val_int; // X轴电子齿轮比分母 (HMI unit: 脉冲 | default:100000)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_ENC_OFF, &par_union);
//     par_axis[AXIS_X].enc_off = par_union.value.val_real; // X轴编码器反馈偏置量 (HMI unit: mm | default:)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_IS_SET_HOME_OFFSET, &par_union);
//     par_axis[AXIS_X].is_set_home_offset = par_union.value.val_bool; // X轴设置编码器偏置使能 (HMI | default: 0)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_JOG_LOWSPD, &par_union);
//     par_axis[AXIS_X].jog_lowspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // X轴慢速点动速度 (HMI unit: mm/min default:18000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_JOG_FASTSPD, &par_union);
//     par_axis[AXIS_X].jog_fastspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // X轴快速点动速度 (HMI unit: mm/min default:36000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_VEL_LIMIT, &par_union);
//     par_axis[AXIS_X].vel_limit = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // X轴最大速度上限 (HMI unit: mm/min default:48000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_MAX_FERROR, &par_union);
//     par_axis[AXIS_X].max_ferror = par_union.value.val_real; // X轴最大跟随误差 (HMI unit: mm | default:0.01mm)
//     getParData(PARAMAN_FILE_AXIS, 0, PAR_AX_MIN_FERROR, &par_union);
//     par_axis[AXIS_X].min_ferror = par_union.value.val_real; // X轴最小跟随误差 (HMI unit: mm | default:0.005mm)
//     //------------------------------------------------------//
//     par_axis[AXIS_Y].name[0] = 'Y';               // 轴名
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_PLMT, &par_union);
//     par_axis[AXIS_Y].plmt = par_union.value.val_real;                // Y轴正软极限
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_NLMT, &par_union);
//     par_axis[AXIS_Y].nlmt = par_union.value.val_real;               // Y轴负软极限
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_HOME_WAY, &par_union);
//     par_axis[AXIS_Y].home_way = par_union.value.val_uint;                // Y轴回参考点方式(0:绝对式 1:相对式)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_HOME_DIR, &par_union);
//     par_axis[AXIS_Y].home_crds = par_union.value.val_real;             // Y轴零点位置
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_HOME_CRDS, &par_union);
//     par_axis[AXIS_Y].home_dir = par_union.value.val_uint;                // Y轴回参考点方向(0:正向 1:负向)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_RAPID_SPD, &par_union);
//     par_axis[AXIS_Y].rapid_vel = par_union.value.val_real / TIME_MINUS_2_S;            // Y轴快移速度 unit:mm/s
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_FEED_SPD, &par_union);
//     par_axis[AXIS_Y].feed_vel = par_union.value.val_real / TIME_MINUS_2_S;              // Y轴最高进给速度
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_ATEETH, &par_union);
//     par_axis[AXIS_Y].ateeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_MTEETH, &par_union);
//     par_axis[AXIS_Y].mteeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_PITCH, &par_union);
//     par_axis[AXIS_Y].pitch = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_MT_PPR, &par_union);
//     par_axis[AXIS_Y].mt_ppr = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_MPG_SPD, &par_union);
//     par_axis[AXIS_Y].mpg_spd = par_union.value.val_real;
//     par_axis[AXIS_Y].pos_tol = 0.01;              // Y轴定位允差
//     //　以下 wcf maintain 2026.02.04
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_RAPID_ACC, &par_union);
//     par_axis[AXIS_Y].rapid_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // Y轴快移加速度上限 (HMI 快移加减速时间常数(单位：ms), default:64ms | 实时 快移加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_RAPID_JK, &par_union);
//     par_axis[AXIS_Y].rapid_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // Y轴快移捷度上限 (HMI 快移加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 快移捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_FEED_ACC, &par_union);
//     par_axis[AXIS_Y].feed_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // Y轴加工加速度上限 (HMI 加工加减速时间常数(单位：ms), default:64ms | 实时 加工加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_FEED_JK, &par_union);
//     par_axis[AXIS_Y].feed_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // Y轴加工捷度上限 (HMI 加工加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 加工捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_PM_MUNIT, &par_union);
//     par_axis[AXIS_Y].pm_munit = CONV_UM_TO_MM(par_union.value.val_int); // Y轴电子齿轮比分子 (HMI unit: um, default:16000um | 实时 单位: mm)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_PM_PULSE, &par_union);
//     par_axis[AXIS_Y].pm_pulse = par_union.value.val_int; // Y轴电子齿轮比分母 (HMI unit: 脉冲 | default:100000)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_ENC_OFF, &par_union);
//     par_axis[AXIS_Y].enc_off = par_union.value.val_real; // Y轴编码器反馈偏置量 (HMI unit: mm | default:)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_IS_SET_HOME_OFFSET, &par_union);
//     par_axis[AXIS_Y].is_set_home_offset = par_union.value.val_bool; // Y轴设置编码器偏置使能 (HMI | default: 0)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_JOG_LOWSPD, &par_union);
//     par_axis[AXIS_Y].jog_lowspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Y轴慢速点动速度 (HMI unit: mm/min default:18000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_JOG_FASTSPD, &par_union);
//     par_axis[AXIS_Y].jog_fastspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Y轴快速点动速度 (HMI unit: mm/min default:36000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_VEL_LIMIT, &par_union);
//     par_axis[AXIS_Y].vel_limit = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Y轴最大速度上限 (HMI unit: mm/min default:48000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_MAX_FERROR, &par_union);
//     par_axis[AXIS_Y].max_ferror = par_union.value.val_real; // Y轴最大跟随误差 (HMI unit: mm | default:0.01mm)
//     getParData(PARAMAN_FILE_AXIS, 1, PAR_AX_MIN_FERROR, &par_union);
//     par_axis[AXIS_Y].min_ferror = par_union.value.val_real; // Y轴最小跟随误差 (HMI unit: mm | default:0.005mm)
//     //------------------------------------------------------//
//     par_axis[AXIS_Z].name[0] = 'Z';               // 轴名
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_PLMT, &par_union);
//     par_axis[AXIS_Z].plmt = par_union.value.val_real;                // Z轴正软极限
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_NLMT, &par_union);
//     par_axis[AXIS_Z].nlmt = par_union.value.val_real;               // Z轴负软极限
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_HOME_WAY, &par_union);
//     par_axis[AXIS_Z].home_way = par_union.value.val_uint;                // Z轴回参考点方式(0:绝对式 1:相对式)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_HOME_DIR, &par_union);
//     par_axis[AXIS_Z].home_crds = par_union.value.val_real;             // Z轴零点位置
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_HOME_CRDS, &par_union);
//     par_axis[AXIS_Z].home_dir = par_union.value.val_uint;                // Z轴回参考点方向(0:正向 1:负向)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_RAPID_SPD, &par_union);
//     par_axis[AXIS_Z].rapid_vel = par_union.value.val_real / TIME_MINUS_2_S;            // Z轴快移速度 unit:mm/s
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_FEED_SPD, &par_union);
//     par_axis[AXIS_Z].feed_vel = par_union.value.val_real / TIME_MINUS_2_S;              // Z轴最高进给速度
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_ATEETH, &par_union);
//     par_axis[AXIS_Z].ateeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_MTEETH, &par_union);
//     par_axis[AXIS_Z].mteeth = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_PITCH, &par_union);
//     par_axis[AXIS_Z].pitch = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_MT_PPR, &par_union);
//     par_axis[AXIS_Z].mt_ppr = par_union.value.val_uint;
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_MPG_SPD, &par_union);
//     par_axis[AXIS_Z].mpg_spd = par_union.value.val_real;
//     par_axis[AXIS_Z].pos_tol = 0.01; // Z轴定位允差
//     //　以下 wcf maintain 2026.02.04
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_RAPID_ACC, &par_union);
//     par_axis[AXIS_Z].rapid_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // Z轴快移加速度上限 (HMI 快移加减速时间常数(单位：ms), default:64ms | 实时 快移加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_RAPID_JK, &par_union);
//     par_axis[AXIS_Z].rapid_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // Z轴快移捷度上限 (HMI 快移加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 快移捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_FEED_ACC, &par_union);
//     par_axis[AXIS_Z].feed_acc = CONV_T_MS_TO_ACCEL(par_union.value.val_real); // Z轴加工加速度上限 (HMI 加工加减速时间常数(单位：ms), default:64ms | 实时 加工加速度上限(单位：mm/s^2))
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_FEED_JK, &par_union);
//     par_axis[AXIS_Z].feed_jerk = CONV_T_MS_TO_JERK(par_union.value.val_real); // Z轴加工捷度上限 (HMI 加工加减速捷度时间常数(单位：ms)，通常设定在8～150间，default:128ms | 实时 加工捷度上限(单位：mm/s^3))
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_PM_MUNIT, &par_union);
//     par_axis[AXIS_Z].pm_munit = CONV_UM_TO_MM(par_union.value.val_int); // Z轴电子齿轮比分子 (HMI unit: um, default:16000um | 实时 单位: mm)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_PM_PULSE, &par_union);
//     par_axis[AXIS_Z].pm_pulse = par_union.value.val_int; // Z轴电子齿轮比分母 (HMI unit: 脉冲 | default:100000)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_ENC_OFF, &par_union);
//     par_axis[AXIS_Z].enc_off = par_union.value.val_real; // Z轴编码器反馈偏置量 (HMI unit: mm | default:)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_IS_SET_HOME_OFFSET, &par_union);
//     par_axis[AXIS_Z].is_set_home_offset = par_union.value.val_bool; // Z轴设置编码器偏置使能 (HMI | default: 0)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_JOG_LOWSPD, &par_union);
//     par_axis[AXIS_Z].jog_lowspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Z轴慢速点动速度 (HMI unit: mm/min default:18000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_JOG_FASTSPD, &par_union);
//     par_axis[AXIS_Z].jog_fastspd = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Z轴快速点动速度 (HMI unit: mm/min default:36000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_VEL_LIMIT, &par_union);
//     par_axis[AXIS_Z].vel_limit = CONV_MM_MIN_TO_MM_S(par_union.value.val_real); // Z轴最大速度上限 (HMI unit: mm/min default:48000mm/min | 实时 单位：mm/s)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_MAX_FERROR, &par_union);
//     par_axis[AXIS_Z].max_ferror = par_union.value.val_real; // Z轴最大跟随误差 (HMI unit: mm | default:0.01mm)
//     getParData(PARAMAN_FILE_AXIS, 2, PAR_AX_MIN_FERROR, &par_union);
//     par_axis[AXIS_Z].min_ferror = par_union.value.val_real; // Z轴最小跟随误差 (HMI unit: mm | default:0.005mm)
//     //------------------------------------------------------//
//     par_axis[AXIS_SP].name[0] = 'SP';               // 轴名
//     getParData(PARAMAN_FILE_AXIS, 3, PAR_AX_DEFAULT_S, &par_union);
//     par_axis[AXIS_SP].default_s = par_union.value.val_real;                // sp缺省速度
//     getParData(PARAMAN_FILE_AXIS, 3, PAR_AX_FEED_ACC, &par_union);
//     par_axis[AXIS_SP].feed_acc = par_union.value.val_real;
//     getParData(PARAMAN_FILE_AXIS, 3, PAR_SPDL_MAX_SPEED, &par_union);
//     par_axis[AXIS_SP].spdl_max_speed = par_union.value.val_real;
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
//     return GNC_OPERATOR_SUCCESS;
// }

