/******************************************************************** 
 * @file        var 变量数据结构.h
 * @brief       
 * @version     V1.00
 * @date        2026/01/20
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef VARDATA_H
#define VARDATA_H

#include "gncdatadef.h"
#include "vardatadef.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    int modeVal;        // 模态值
    char modeStr[8];    // 模态字符串
}ModalData;

// 总的变量数据结构
typedef struct {
    VarUnion var_axis[TOTAL_AXES_NUM][VAR_AXIS_NUM];   // 轴变量数组 (进给轴 + 主轴)
    VarUnion var_channel[SYS_CHAN_NUM][VAR_CHAN_NUM];  // 通道变量数组
    VarUnion var_system[VAR_SYS_NUM];                  // 系统变量
    ModalData var_modal[MODAL_GROUP];
} VarData;

#ifdef __cplusplus
}
#endif
#endif // VARDATA_H
