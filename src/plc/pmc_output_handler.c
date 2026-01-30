/**
 * @file pmc_output_handler.c
 */
#include "pmc_output_handler.h"
#include "regapi.h"   /* 引入寄存器访问 API */
#include "regdata.h"  /* 引入 RegType 定义 */
#include "global.h"

/* ==========================================================================
 * 外部函数定义
 * ========================================================================== */

void pmc_output_feedback() {
    /**
     * 映射策略：
     * F0000.0 (第0字节第0位) -> 运行状态反馈
     * F0001.0 (第1字节第0位) -> 报警状态反馈
     */
    unsigned char is_running = var_chan[0].is_running ? 0x01 : 0x00;
    unsigned char is_estop = var_chan[0].is_estop ? 0x01 : 0x00;
    /* 使用底层 API 写入 F 寄存器区 */
    setRegValue8(REG_F, 0, is_running); /* REG_F 定义见 regdata.h */
    setRegValue8(REG_F, 1, is_estop);

    if(sys->current_mode == MODE_EDIT) { 
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 6); 
    } 

    if(sys->current_mode == MODE_MEM) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 5); 
    } 

    if(sys->current_mode == MODE_MDI) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 3);
    } 

    if(sys->current_mode == MODE_DNC) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 4); 
    } 

    if(sys->current_mode == MODE_MPG) {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 1); 
    } 

    if(sys->current_mode == MODE_JOG) {
        clearRegBit(REG_F, 4, 5);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        setRegBit(REG_F, 3, 2);
    } 

    if(sys->current_mode == MODE_RAPID_JOG) {
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 2, 1); 
    } 
    
    if(sys->current_mode == MODE_HOMING) {
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
