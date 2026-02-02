#ifndef NCITPDATADEF_H
#define NCITPDATADEF_H

#include "motiondata.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PMC_DATA_COUNT_MAX 100     // 单行译码最大结果数

typedef struct {
    pmc_data datas[PMC_DATA_COUNT_MAX];
    int count;
} NcItpData;

typedef struct {
    int value;
    bool valid;
} IntV;

typedef struct {
    double value;
    bool valid;
} DoubleV;

/*!  模态 */
typedef struct {
    int modal[80];
    DoubleV f;
    IntV s;
    IntV t;
    IntV h;
    IntV d;
} NcItpState;

/*!  译码组包 */
typedef struct {
    NcItpData ncItpData;    // 译码数据
    int lineNo;             // 译码当前行号
    NcItpState ncItpState;  // 译码模态
} NcItpDataPacket;

#ifdef __cplusplus
}
#endif

#endif // NCITPDATADEF_H
