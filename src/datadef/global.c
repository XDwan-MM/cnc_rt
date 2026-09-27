#include "global.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <shmapi.h>
#include <vardatadef.h>
#include <varapi.h>
#include <pardata.h>
#include <parapi.h>
#include <sysapi.h>
#include <sysdata.h>
#include <hmisysapi.h>
#include <pthread.h>
#include "tapreturn.h"

// ==============全局变量定义===============
sem_t exit_sem;                         // 退出信号
sem_t op_sem;                           // 已进入op信号，用于线程间通信
int InterruptFlag = 0;                  // 打断标志位
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
RERUN_DATA* rerun_data;
RESET_DATA* nc_reset;
EVENT_QUEUE* event_queue;       // 事件队列
PLAN_QUEUE *parser_queue;       // 译码队列
PLAN_QUEUE *pretreat_queue;     // 预处理队列
PLAN_QUEUE *plan_queue;         // 插补队列
COORD_TP *cycle_tp;             // 循环轨迹规划
REF_TP *ref_tp;                 // 回零轨迹规划

// 读写共享内存锁（路径规划读时加锁）
pthread_mutex_t shm_mutex;

// ==============全局变量定义===============

static GNC_RES shm_var_init(); 
static GNC_RES shm_par_init();
static GNC_RES tp_init();
static bool shm_mutex_initialized = false;

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
        setVarChanInt(i, VAR_CHAN_IS_ESTOP, 0, 0);              // 急停
        setVarChanInt(i, VAR_CHAN_IS_RESETTING, 0, 0);          // 复位中
        setVarChanInt(i, VAR_CHAN_IS_MOVING, 0, 0);             // 轴移动中
        setVarChanInt(0, VAR_CHAN_IS_PROGEND, 1, 0);            // 程序完成
    }
    // 坐标轴数据
    for(int i = 0; i < SYS_AXES_NUM; i++){    
        setVarAxisDouble(i, VAR_AXIS_CMD_VEL, 0, 0);            // 轴指令速度
        setVarAxisDouble(i, VAR_AXIS_ACT_VEL, 0, 0);            // 轴实际速度
        setVarAxisDouble(i, VAR_AXIS_CMD_POS_WCS_EX, 0, 0);     // 绝对坐标系
        setVarAxisDouble(i, VAR_AXIS_CMD_POS_EX, 0, 0);         // 轴指令位置 
        setVarAxisDouble(i, VAR_AXIS_ACT_POS_EX, 0, 0);         // 轴实际位置   
        setVarAxisDouble(i, VAR_AXIS_IS_DISABLE, 0, 0);         // 轴拆卸  
        setVarAxisDouble(i, VAR_AXIS_IS_MAC_LOCK, 0, 0);        // 机械锁住  
        setVarAxisDouble(i, VAR_AXIS_IS_INTER_LOCK, 0, 0);      // 互锁  
        setVarAxisDouble(i, VAR_AXIS_IS_MOVING, 0, 0);          // 移动中
        setVarAxisDouble(i, VAR_AXIS_IS_SERVO_OFF, 0, 0);       // 伺服关断
        setVarAxisDouble(i, VAR_AXIS_IS_MIRROR, 0, 0);          // 镜像   
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
    res = ref_tp_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = cycle_tp_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    parser_queue = (PLAN_QUEUE*)calloc(sizeof(PLAN_QUEUE), 1);
    if(parser_queue == NULL) return GNC_DATA_NULL;
    res = queue_init(parser_queue);
    pretreat_queue = (PLAN_QUEUE*)calloc(sizeof(PLAN_QUEUE), 1);
    if(pretreat_queue == NULL) return GNC_DATA_NULL;
    res = queue_init(pretreat_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    plan_queue = (PLAN_QUEUE*)calloc(sizeof(PLAN_QUEUE), 1);
    if(plan_queue == NULL) return GNC_DATA_NULL;
    res = queue_init(plan_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES tp_reset(){
    ref_tp_reset();
    cycle_tp_reset();
    queue_reset(parser_queue);
    queue_reset(pretreat_queue);
    queue_reset(plan_queue);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES rt_global_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    res = shm_var_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = shm_par_init();
    if(res != GNC_OPERATOR_SUCCESS) return res;
    res = rt_data_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
    res = tp_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
    res = task_mq_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
    res = rerun_data_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
    res = reset_data_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
        res = sysprog_init();
    if(res != GNC_OPERATOR_SUCCESS){
        rt_global_delete();
        return res;
    }
    tapreturn_data_restore();
    if(pthread_mutex_init(&shm_mutex, NULL) != 0){
        rt_global_delete();
        return GNC_OPERATOR_ERROR;
    }
    shm_mutex_initialized = true;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES rt_global_delete(){
    if(event_queue != NULL){
        event_queue_delete(event_queue);
        free(event_queue);
        event_queue = NULL;
    }
    if(parser_queue != NULL){
        queue_delete(parser_queue);
        free(parser_queue);
        parser_queue = NULL;
    }
    if(pretreat_queue != NULL){
        queue_delete(pretreat_queue);
        free(pretreat_queue);
        pretreat_queue = NULL;
    }
    if(plan_queue != NULL){
        queue_delete(plan_queue);
        free(plan_queue);
        plan_queue = NULL;
    }
    cycle_tp_delete();
    ref_tp_delete();
    if(rt_data != NULL){
        pthread_mutex_destroy(&rt_data->mutex);
        free(rt_data);
        rt_data = NULL;
    }
    if(rerun_data != NULL){
        free(rerun_data);
        rerun_data = NULL;
    }
    if(nc_reset != NULL){
        free(nc_reset);
        nc_reset = NULL;
    }
    if(shm_mutex_initialized){
        pthread_mutex_destroy(&shm_mutex);
        shm_mutex_initialized = false;
    }
    var_data = NULL;
    var_axis = NULL;
    var_chan = NULL;
    var_sys = NULL;
    par_data = NULL;
    par_ncu = NULL;
    par_mac = NULL;
    par_chan = NULL;
    par_axis = NULL;
    par_acmp = NULL;
    par_cfg = NULL;
    par_table = NULL;
    sys_data = NULL;
    return GNC_OPERATOR_SUCCESS;
}

/// @brief 实时数据更新到共享内存var数据中
void shm_var_update(){
    VarUnion crds_type_var = {0};
    double crds_remain = 0.0;
    double crds_oft = 0.0;
    double act_pos_wcs_ex = 0.0;
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        // 绝对坐标 = 工件指令位置
        if (!rt_data->axis_data[i].status.is_disable)
        {
            getVarChanValue(0, VAR_CHAN_MODAL, 14, &crds_type_var, 0);
            getCrdsValue(CRDS_TYPE_G5X + crds_type_var.value.i - 1, i, &crds_oft);
            act_pos_wcs_ex = rt_data->axis_data[i].wcs_cmd_pos_cur - crds_oft;
            setVarAxisDouble(i, VAR_AXIS_CMD_POS_WCS_EX, act_pos_wcs_ex, 0);
        }
        if(rt_data->current_mode == MODE_MEM || rt_data->current_mode == MODE_MDI || rt_data->current_mode == MODE_DNC){
            // 工件指令位置 = 编程位置
            if(queue_is_empty(plan_queue)){
                setVarAxisDouble(AXIS_X, VAR_AXIS_PROG_POS_EX, var_axis[0][VAR_AXIS_CMD_POS_WCS_EX].value.f, 0);
                setVarAxisDouble(AXIS_Y, VAR_AXIS_PROG_POS_EX, var_axis[1][VAR_AXIS_CMD_POS_WCS_EX].value.f, 0);
                setVarAxisDouble(AXIS_Z, VAR_AXIS_PROG_POS_EX, var_axis[2][VAR_AXIS_CMD_POS_WCS_EX].value.f, 0);
            }else{
                setVarAxisDouble(AXIS_X, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.x, 0);
                setVarAxisDouble(AXIS_Y, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.y, 0);
                setVarAxisDouble(AXIS_Z, VAR_AXIS_PROG_POS_EX, cycle_tp->plan_block->end_pos.z, 0);
            }
            // 剩余移动量
            crds_remain = var_axis[i][VAR_AXIS_PROG_POS_EX].value.f - rt_data->axis_data[i].wcs_cmd_pos_cur;
        }else{
            crds_remain = 0;
        }
        // 剩余移动
        setVarAxisDouble(i, VAR_AXIS_LEFT_TOGO_EX, crds_remain, 0);
        // 相对坐标 = 相对指令位置
        double real_zero = 0.0;
        getVarAxisDouble(i, VAR_AXIS_REAL_ZERO_EX, &real_zero, 0);
        double real = var_axis[i][VAR_AXIS_CMD_POS_WCS_EX].value.f - real_zero;
        setVarAxisDouble(i, VAR_AXIS_CMD_POS_RCS_EX, real, 0);
        // 机械坐标 = 机床指令位置
        setVarAxisDouble(i, VAR_AXIS_CMD_POS_EX, rt_data->axis_data[i].mac_cmd_pos, 0);
        // 机床实际位置
        setVarAxisDouble(i, VAR_AXIS_ACT_POS_EX, rt_data->axis_data[i].motor_act_pos, 0);
        // 电机脉冲
        setVarAxisInt(i, VAR_AXIS_ENC_CNTR, rt_data->axis_data[i].enc_cntr, 0);
        // 实际轴速度
        setVarAxisDouble(i, VAR_AXIS_ACT_VEL_EX, rt_data->axis_data[i].act_vel * TIME_MINUS_2_S, 0); 
        // 指令轴速度
        setVarAxisDouble(i, VAR_AXIS_CMD_VEL_EX, rt_data->axis_data[i].cmd_vel * TIME_MINUS_2_S, 0);
        // 轴状态显示：机械锁住、互锁、移动中
        if(rt_data->sys_state[SYS_IS_MAC_LOCK_READY]) setVarAxisInt(i, VAR_AXIS_IS_MAC_LOCK, 1, 0);
        else setVarAxisInt(i, VAR_AXIS_IS_MAC_LOCK, 0, 0);
        if(rt_data->sys_state[SYS_IS_IT_LOCK]) setVarAxisInt(i, VAR_AXIS_IS_INTER_LOCK, 1, 0);
        else setVarAxisInt(i, VAR_AXIS_IS_INTER_LOCK, 0, 0);
        if(IsEqualTo(rt_data->axis_data[i].cmd_vel, 0)) setVarAxisInt(i, VAR_AXIS_IS_MOVING, 0, 0);
        else setVarAxisInt(i, VAR_AXIS_IS_MOVING, 1, 0);
    }
    for (int i = 0; i < SYS_CHAN_NUM; i++)
    {
        // 运行模式
        setVarChanInt(0, VAR_CHAN_MODE, rt_data->current_mode, 0);
        // 实际速度
        setVarChanDouble(i, VAR_CHAN_ACT_FEEDRATE, CONV_MM_S_TO_MM_MIN(rt_data->chan_data[i].act_vel), 0);
        // 指令速度
        setVarChanDouble(i, VAR_CHAN_CMD_FEEDRATE, CONV_MM_S_TO_MM_MIN(rt_data->chan_data[i].cmd_vel), 0);
        // 刚性攻丝中
        setVarChanInt(0, VAR_CHAN_IS_RIGID, rt_data->sys_state[SYS_IS_RIGID], 0);
    }
}