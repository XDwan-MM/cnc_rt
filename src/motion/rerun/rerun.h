/********************************************************************
 * @file        rerun.h
 * @brief       程序再启动数据
 * @version     V1.00
 * @date        2026/08/27
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 * @details     
 ********************************************************************/

#ifndef RERUN_H
#define RERUN_H

#include "datatype.h"
#include "plan_queue.h"
#include "record_queue.h"
#include <stdbool.h>
#include "hmisysapi.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define RECORD_M_NUM 35
#define RECORD_T_NUM 2
#define RECORD_S_NUM 1

typedef enum{
    BEFORE_THE_RERUN_ROW,
    EXACT_THE_RERUN_ROW,
    AFTER_THE_RERUN_ROW,
}ROW_STATE;

typedef struct
{
    bool is_rerun;
    int rerun_row;                     // 再启动行号
    bool is_mem_rerun;
    int mem_rerun_row;
    ROW_STATE row_state;
    // 断点信息
    char parser_file_path[NCFILEPATH_CHAR_LEN];
    POS breakpoint;                    // 断点位置
    PLAN_BLOCK break_block;            // 再启动程序段
    RECORD_QUEUE record_mcode;         // 保存的M指令
    RECORD_QUEUE record_tcode;         // 保存的T指令
    RECORD_QUEUE record_scode;         // 保存的S指令
    bool is_first_time_click_rerun;
} RERUN_DATA;

GNC_RES rerun_data_init();
void rerun_data_reset();
void rerun_update_when_swithing();
void rerun_data_store(PLAN_BLOCK* break_block);
void rerun_store_breakpoint();
bool check_is_program_rerun();
void rerun_reset_is_rerun();
void create_rerun_block(PLAN_BLOCK* rerun_block);
bool check_rerun_row_is_valid();

#ifdef __cplusplus
}
#endif
#endif /* RT_DATA_H */
