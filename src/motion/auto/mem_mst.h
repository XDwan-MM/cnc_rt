
/********************************************************************
 * @file        mem_mcode.h
 * @brief       记忆模式
 * @version     V1.00
 * @date        2026/03/16
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __MEM_MCODE_H__
#define __MEM_MCODE_H__

#include "mcode_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REG_F10 256
#define REG_F11 65536
#define REG_F12 4294967296

typedef enum mcode_state{
    MCODE_EXECUTE_IDLE = 0,
    MCODE_EXECUTE_RUNNING,
    MCODE_EXECUTE_PAUSING,
    MCODE_EXECUTE_RESET,
}MCODE_STATE;

typedef struct{
    MCODE_STATE mcode_state;
    MCODE_BLOCK mcode_block[M_BUF_SIZE]; // 单程序段多个M指令
    int mcode_size;         // 多个辅助功能个数
    bool signal_mf;         // 选通脉冲信号
    bool signal_den;        // 分配结束信号
    bool signal_fin;        // 结束信号
    bool pmc_end;           // pmc侧动作完成
    int tfin_count;         // 完成信号计数
    int tmf_count;          // 选通脉冲延时计数
}MCODE_DATA;

GNC_RES mcode_data_init();
void execute_mcode();
void mcode_handle_estop();

#ifdef __cplusplus
}
#endif

#endif