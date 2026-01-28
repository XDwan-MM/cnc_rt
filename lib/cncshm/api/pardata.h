/********************************************************************
 * @file        parda_structs.h
 * @brief       系统参数结构体定义 、
 * @version     V4.0
 * @date        2025
 ********************************************************************/
 #ifndef PARDATA_H
 #define PARDATA_H

#include "pardatadef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    union
    {
        double  val_real;
        unsigned int val_uint;
        int     val_int;
        char    val_string[PARAM_STRING_LEN];
        char    val_bool;
        int     val_hex4;
        char    val_byte[PARAM_BYTE_LEN];
    } value;
    unsigned char type;     // 数据类型
}ParamUnit;

typedef struct
{
    ParamUnit par_ncu[PARAMAN_FILE_NCU_MAX];
    ParamUnit par_mac[PARAMAN_FILE_MAC_MAX];
    ParamUnit par_chan[SYS_CHAN_NUM][PARAMAN_FILE_CHAN_MAX];
    ParamUnit par_axis[SYS_AXES_NUM][PARAMAN_FILE_AXIS_MAX];
    ParamUnit par_acmp[SYS_AXES_NUM][PARAMAN_FILE_ACMP_MAX];
    ParamUnit par_cfg[SYS_NCBRD_CFG_NUM][PARAMAN_FILE_CFG_MAX];
    ParamUnit par_table[PARAMAN_FILE_TABLE_MAX];
} ParProp;

typedef struct
{
    ParProp par[PARA_PROP_COUNT];
} ParData;


#ifdef __cplusplus
}
#endif
#endif  // PARDATA_STRUCTS_H
