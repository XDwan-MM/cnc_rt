#ifndef NCITPDATADEF_H
#define NCITPDATADEF_H

#include "motiondata.h"
#include "ncitpmacrodef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PMC_DATA_COUNT_MAX 100     // 单行译码最大结果数

typedef struct {
    int cnv;    //干涉检查模式
    int cav;    //干涉检查动作
    int naa;    //危险/二次干涉处理(No.19607#6)
    int sup;    // No.5003#0: 起刀类型低位
    int suv;    // No.5003#1: 起刀类型高位
    int ccc;    // No.19607#2: 外边拐角连接 0=直线 1=圆弧
    int lookAheadSegments;      // 刀补开启时预读段数，缓冲区容量 = 预读段数 + 1
} ItpParam_temp;

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
