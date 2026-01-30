#ifndef NCITPDATADEF_H
#define NCITPDATADEF_H

#include "motiondata.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pmc_data *datas;
    int count;
} NcItpData;

typedef struct {
    NcItpData ncItpData;
    int lineCount;          // 译码总行数
    int lineNumber;         // 译码当前行号
//    int dataIndex;          // 实包个数
} NcItpDataPacket;

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

#ifdef __cplusplus
}
#endif

#endif // NCITPDATADEF_H
