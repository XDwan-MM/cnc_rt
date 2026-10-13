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
#include <stdbool.h>
#include "hmisysapi.h"
#include "rerunsaveapi.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum{
    BEFORE_THE_RERUN_ROW,
    EXACT_THE_RERUN_ROW,
    AFTER_THE_RERUN_ROW,
}ROW_STATE;


typedef enum {
    MST_GROUP_NONE = -1,     /* 不属于任何组 */
    MST_GROUP_SPINDLE = 0,   /* 主轴转向：M03 正 / M04 反 / M05 停 */
    MST_GROUP_COOLANT,       /* 冷却：M08 开 / M09 关 */
    MST_GROUP_COUNT,
} MST_MODAL_GROUP;

#define MST_REPLAY_MAX   (35)
#define MST_S_HISTORY    (2)

typedef struct
{
    int  m_code[MST_REPLAY_MAX];   /* 按序：要重演的 M 码（最近 MST_REPLAY_MAX 条） */
    int  count;                    /* 序列长度 */
    int  s[MST_S_HISTORY];
    int  t;                        /* 刀号 */
} MST_RESTORE;

typedef struct
{
    bool        valid;
    int         line_number;   /* 目标段行号 */
    pmc_mode    mode;          /* PMC_MODE_*（判 TAP / TAP_RETURE / STOPSKIP / M_CMD） */
    INTERP_TYPE type;          /* LINE / CIRCLE_* / RIGID_TAP / TAP_RETURN / SPLINE… */
    int         m_code;        /* 判 M06 / M19 */
} BREAK_POINT;

typedef struct
{
    bool is_rerun;
    int rerun_row;                     // 本次运行跳转的行号
    bool is_mem_rerun;
    ROW_STATE row_state;
    // 断点信息
    char parser_file_path[NCFILEPATH_CHAR_LEN]; //文件名
    POS breakpoint;                    // 断点位置（机床坐标 XYZ）
    POS program_start;                 // 程序起点（机床坐标 XYZ）
    bool program_start_valid;
    BREAK_POINT break_block;           // 目标段（行号 + 留档字段）
    POS break_end;                     // 目标段译码终点
    MST_RESTORE mst;                   // 辅助功能恢复（扫描时现收，见上）
    bool is_first_time_click_rerun;
} RERUN_DATA;

GNC_RES rerun_data_init();
void rerun_data_reset();
void rerun_data_reset_keep_breakpoint();

void rerun_update_when_switching();
void rerun_data_store(PLAN_BLOCK* break_block);
void rerun_store_breakpoint(bool is_breakpoint);
bool check_is_program_rerun();
void rerun_reset_is_rerun();

bool check_rerun_row_is_valid(const PLAN_BLOCK* target);

bool rerun_program_changed(const PLAN_BLOCK* target);

void rerun_store_program_start(void);

void create_rerun_block(const PLAN_BLOCK* target, PLAN_BLOCK* rerun_block);

void rerun_mst_collect(const PLAN_BLOCK* block);

/*! 清空重演序列（每次再启动开始时、以及复位/换程序时）。 */
void rerun_mst_reset(void);

void rerun_make_mst_block(const MST_RESTORE* r, int index,
                          const PLAN_BLOCK* target, PLAN_BLOCK* out);

// typedef struct {
//     bool    valid;
//     int     line_number; /* 目标段行号 */
//     int     mode;        /* PMC_MODE_*（ TAP / TAP_RETURE / STOPSKIP / M_CMD） */
//     int     type;        /* LINE / CIRCLE_* / RIGID_TAP / TAP_RETURN / SPLINE… */
//     int     m_code;      /* 判 M06 / M19 */
// } RERUN_SAVE_BLOCK;

// typedef struct
// {
//     char        parser_file_path[NCFILEPATH_CHAR_LEN];  /*!< 程序标识（路径） */
//     POS         program_start;                          /*!< 程序起点（机床坐标 XYZ） */
//     bool        program_start_valid;
//     POS         breakpoint;                             /*!< 断点位置（机床坐标 XYZ） */
//     int         rerun_row;                              /*!< 目标段行号 */
//     RERUN_SAVE_BLOCK break_block;                            /*!< 目标段（行号 + 留档字段） */
//     POS         break_end;                              /*!< 目标段那行第一个包的译码终点（绝对） */
//     bool        is_mem_rerun;                           /*!< 这一轮记忆模式被打断过 */
// } RERUN_SAVE;

void rerun_save_pack(RERUN_SAVE* out);

bool rerun_save_unpack(const RERUN_SAVE* in);

void rerun_persist(void);

void rerun_restore(void);

#ifdef __cplusplus
}
#endif
#endif /* RT_DATA_H */
