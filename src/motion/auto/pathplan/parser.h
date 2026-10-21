#ifndef __PARSER_H__
#define __PARSER_H__

/******************************************************************** 
 * @file        parser.h
 * @brief       译码结果处理
 * @version     V1.00
 * @date        2025/12/18
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include "ncitpapi.h"
#include "datatype.h"
#include "hmisysdata.h"

#include "macrovarapi.h"
#include "macrovardata.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! 宏变量相关 {*/
// #define MacroVarCount (99999)
// extern double cmv[MacroVarCount];
int t_macroVarGetValue(int no, double *mac);
int t_macroVarSetValue(int no, double mac);
/*! }*/

#define PARSER_BUF_SIZE 1000 //译码器最大处理长度
#define PARSER_STOP_NONE (-1) // 无待完成的阻止缓冲块

typedef enum{
    PARSER_IDLE = 0,
    PARSER_READING,
    PARSER_WAITING_FOR_MOTION,
    PARSER_WAITING_FOR_IO,
    PARSER_WAITING_FOR_BUF,
    PARSER_WAITING_TO_END,
    PARSER_WAITING_FOR_BUFSTOP,     // 停在阻止缓冲块上，等该块运行完（CHAN_IS_BUFSTOP_PASSED）
}PARSER_STATE;

// 阻止译码器缓冲状态
typedef struct{
    int code;        // 诊断用：M 码号或 G 值
    int is_g;        // 0 = M码；1 = G53
    int hold;        // 是否处于"阻止译码推进"阶段
    unsigned int id; // 当前等待的停止块 ID（0=无）。跨线程，用 __atomic 读写
} BufStop;

static inline bool buf_stop_active(const BufStop *s) { return s->id != 0; }

static inline void buf_stop_release(BufStop *s)
{
    __atomic_store_n(&s->id, 0u, __ATOMIC_RELEASE);
    s->code = PARSER_STOP_NONE;
    s->is_g = 0;
}

static inline void buf_stop_hold(BufStop *s) { s->hold = 1; }
static inline void buf_stop_hold_done(BufStop *s) { s->hold = 0; }
static inline void buf_stop_clear(BufStop *s)
{
    __atomic_store_n(&s->id, 0u, __ATOMIC_RELEASE);
    s->code = PARSER_STOP_NONE;
    s->is_g = 0;
    s->hold = 0;
}

typedef struct{
    // ItpParam itp_parameter;          // 译码参数 todo:调整译码后删除
    NcItpData itp_datas;                // 译码结果
    NcItpState itp_state;               // 模态
    int interp_list_index;              // 译码结果索引
    int itp_line_no;                    // 当前行号
    double cmv[MacroVarCount];          // 宏变量 todo:译码调整
    PARSER_STATE parser_state;
    unsigned int buf_stop_next_id;      // 停止块 ID 分配器
    BufStop bufstop;                    // 阻止缓冲状态
    char parser_file_path[NCFILEPATH_CHAR_LEN];
    char mdiContent[MDICONTENT_CHAR_LEN];
}PARSER_DATA;

extern PARSER_DATA* parser_data;

void parser_new();
GNC_RES parser_init(NC_MODE nc_mode);
GNC_RES parser_reading();
void parser_reset();
void parser_run_mode();

#ifdef __cplusplus
}
#endif

#endif
