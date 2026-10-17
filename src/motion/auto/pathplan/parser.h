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
int t_getParamValue(int parId,double *val);
int t_setParamValue(int parId,double val);
/*! }*/

#define PARSER_BUF_SIZE 1000 //译码器最大处理长度

#define PARSER_STOP_NONE (-1) //-1 表示无待完成的停止程序

typedef enum{
    PARSER_IDLE = 0,
    PARSER_READING,
    PARSER_WAITING_FOR_MOTION,
    PARSER_WAITING_FOR_IO,
    PARSER_WAITING_FOR_BUF,
    PARSER_WAITING_TO_END,
}PARSER_STATE;

// 阻止译码器缓冲状态
typedef struct{
    int code;       // PARSER_STOP_NONE = 无；否则M 码号或G值
    int is_g;       // 0 = code是M码；1 = 非模态G
    int seen;       // 是否已看到它成为当前块
    int hold;       // 
} BufStop;

static inline bool buf_stop_active(const BufStop *s) { return s->code != PARSER_STOP_NONE; }

static inline void buf_stop_release(BufStop *s)
{
    s->code = PARSER_STOP_NONE;
    s->is_g = 0;
    s->seen = 0;
}

// 阻止译码器推进
static inline void buf_stop_hold(BufStop *s) { s->hold = 1; }
static inline void buf_stop_hold_done(BufStop *s) { s->hold = 0; }
static inline void buf_stop_clear(BufStop *s)
{
    s->code = PARSER_STOP_NONE;
    s->is_g = 0;
    s->seen = 0;
    s->hold = 0;
}

typedef struct{
    // ItpParam itp_parameter;          // 译码参数 todo:调整译码后删除
    NcItpData itp_datas;                // 译码结果
    NcItpState itp_state;               // 模态
    int interp_list_index;              // 译码结果索引
    int itp_line_no;                    // 当前行号
    PARSER_STATE parser_state;
    BufStop      bufstop;               // 阻止缓冲标志                     
    char parser_file_path[NCFILEPATH_CHAR_LEN];
    char mdiContent[MDICONTENT_CHAR_LEN];
}PARSER_DATA;

extern PARSER_DATA* parser_data;

void parser_new();
GNC_RES parser_init(NC_MODE nc_mode);
GNC_RES parser_reading();
void parser_reset();
void parser_run_mode();
bool parser_isBufStopMCode(int m_code); 


#ifdef __cplusplus
}
#endif

#endif
