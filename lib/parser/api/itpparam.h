#ifndef ITPPARAM_H
#define ITPPARAM_H

#include <QList>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    QList<double> toolLenList;      //刀具长度偏置
    QList<double> toolRadList;      //刀具半径偏置
    bool blkSkip = false;           //单节忽略
    bool nullRunFeed = false;       //空运行进给
} ItpParam;

typedef struct
{
    char read = 6;
    char write = 6;
    bool valid = false;
    double varDouble = 0;
} MacroVar;

#ifdef __cplusplus
}
#endif

#endif // ITPPARAM_H
