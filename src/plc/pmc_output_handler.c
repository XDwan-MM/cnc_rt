/**
 * @file pmc_output_handler.c
 */
#include "pmc_output_handler.h"
#include "regapi.h"   /* 引入寄存器访问 API */
#include "regdata.h"  /* 引入 RegType 定义 */
#include "global.h"
#include <hmisysapi.h>
#include "spl_control.h"

/* ==========================================================================
 * 外部函数定义
 * ========================================================================== */

static void pmc_output_mode();
static void pmc_output_home();
static void pmc_output_auto();
static void pmc_output_reset();
static void pmc_output_axis_status();

void pmc_output_feedback() {
    /**
     * 映射策略：
     * F0000.0 (第0字节第0位) -> 运行状态反馈
     * F0001.0 (第1字节第0位) -> 报警状态反馈
     */
    unsigned char is_running = var_chan[0][VAR_CHAN_IS_RUNNING].value.i ? 0x01 : 0x00;
    unsigned char is_estop = rt_data->sys_state[SYS_IS_ESTOP_ON] ? 0x01 : 0x00;
    /* 使用底层 API 写入 F 寄存器区 */
    setRegValue8(REG_F, 0, is_running); /* REG_F 定义见 regdata.h */
    setRegValue8(REG_F, 1, is_estop);

    // 急停报警中
    // if(rt_data->sys_state[SYS_IS_ESTOP_ON]){
    //     setRegBit(REG_F, 1, 1); //主轴状态重置 
    // }

    // 报警中
    if(rt_data->sys_state[SYS_IS_ALARMING]) setRegBit(REG_F, 1, 0); 
    else clearRegBit(REG_F, 1, 0);

    pmc_output_mode();
    pmc_output_auto();
    pmc_output_home();
    pmc_output_reset();
    pmc_output_axis_status();
}

// 自动运行启动中、运行中信号、休止中信号、复位
static void pmc_output_auto(){
    // OP：运行中信号
    if(var_chan[0][VAR_CHAN_IS_RUNNING].value.i) setRegBit(REG_F, 0, 7); 
    else clearRegBit(REG_F, 0, 7);
    // STL：启动中
    if(cycle_tp->mem_exec_state == EXEC_RUNNING) setRegBit(REG_F, 0, 5);
    else clearRegBit(REG_F, 0, 5);
    // SPL：休止中
    if(cycle_tp->mem_exec_state == EXEC_PAUSING) setRegBit(REG_F, 0, 4); 
    else clearRegBit(REG_F, 0, 4);
}

// 各轴返回零点完成、参考点建立
static void pmc_output_home(){
    // 急停后清除回参考点建立
    if(rt_data->sys_state[SYS_IS_ESTOP_ON]){
        for (int i = 0; i < SYS_AXES_NUM; i++){
            clearRegBit(REG_F, 120, i);
        }
        return;
    }
    for (int i = 0; i < SYS_AXES_NUM; i++){
        if(var_axis[i][VAR_AXIS_IS_HOMEF].value.i){
            setRegBit(REG_F, 94, i);
            setRegBit(REG_F, 120, i);
        } 
        else{
            clearRegBit(REG_F, 94, i);
        }
    }
}

static void pmc_output_reset(){
    static bool last_reset_state = false;
    static int reset_tic = 0;
    // RST：复位
    if(var_chan[0][VAR_CHAN_IS_RESETTING].value.i){
        setRegBit(REG_F, 1, 1);
        last_reset_state = true;
    }
    // 复位减速后延时
    if(last_reset_state && !var_chan[0][VAR_CHAN_IS_RESETTING].value.i){
        if(reset_tic < par_mac[PAR_MAC_HARD_RESET_DELAY].value.val_int){
            reset_tic++;
            setRegBit(REG_F, 1, 1);
            // LOG(LOG_LEVEL_DEBUG, "复位减速后延时 %d", reset_tic);
        }
        else{
            clearRegBit(REG_F, 1, 1);
            last_reset_state = false;
            reset_tic = 0;   
        }
    }
}

// 方式选择
static void pmc_output_mode(){
    if(rt_data->current_mode == MODE_IDLE){
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5); 
    }

    if(rt_data->current_mode == MODE_EDIT) { 
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 6); 
    } 

    if(rt_data->current_mode == MODE_MEM) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 5); 
    } 

    // 自动运行选择检测 F3.5
    if(rt_data->current_mode == MODE_MDI) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 3);
    } 

    if(rt_data->current_mode == MODE_DNC) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 4); 
    } 

    if(rt_data->current_mode == MODE_MPG) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 1); 
    } 

    if(rt_data->current_mode == MODE_JOG) {
        clearRegBit(REG_F, 4, 5);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 6);
        setRegBit(REG_F, 3, 2);
    } 

    if(rt_data->current_mode == MODE_RAPID_JOG) {
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 2, 1); 
    } 
    
    if(rt_data->current_mode == MODE_HOMING) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        setRegBit(REG_F, 4, 5);
    }
}

//轴移动状态输出
static void pmc_output_axis_status()
{
    for (int i = 0; i < 3; i++)
    {
        if(rt_data->axis_data[i].axis_status.is_moving)
        {
            if(rt_data->axis_data[i].axis_status.direction)
            {
                clearRegBit(REG_F, 106, i);
            }
            else
            {
                setRegBit(REG_F, 106, i);
            }
            setRegBit(REG_F, 102, i);
        }
        else
        {
            clearRegBit(REG_F, 102, i);
        }
    }
}
