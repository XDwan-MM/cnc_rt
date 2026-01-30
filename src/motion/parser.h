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

#ifdef __cplusplus
extern "C" {
#endif

#define PARSER_BUF_SIZE 1000 //译码器最大处理长度

typedef enum{
    PARSER_IDLE = 0,
    PARSER_READING,
    PARSER_WAITING_FOR_MOTION,
    PARSER_WAITING_FOR_IO,
    PARSER_WAITING_FOR_BUF,
}PARSER_STATE;

typedef struct{
    ItpParam itp_parameter;
    NcItpData itp_datas;
    NcItpState itp_state;
    int interp_list_index;
    char parser_file_path[NCFILEPATH_CHAR_LEN];
    MacroVar cmv[30000];
    PARSER_STATE parser_state;
}PARSER_DATA;

extern PARSER_DATA* parser_data;

void parser_new();
GNC_RES parser_init(const char* parser_file_path);
GNC_RES parser_reading();
void parser_reset();
void parser_run_mode();


#ifdef __cplusplus
}
#endif

#endif
