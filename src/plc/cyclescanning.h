/*****************************************************************
 * Copyright @ GreeElectric Co.,Ltd.2025-2035.All right reserved
 * File name :     cyclescanning.h
 * Description:    1.简单叙述本文的内容，程序完成的主要功能；
 *                 2.与其他模块或函数的接口；
 *                 3.其他内容的说明
 *
 * ***************************************************************
 *作者：     黄松清
 *日期：     2025.9.19
 *版本：     V2.0
 * ***************************************************************/
#ifndef CYCLESCANNING_H
#define CYCLESCANNING_H
#include <string.h>
#include <stdio.h>
#include <QList>
#include "plcdatatype.h"
#include "mt_data.h"
#include "ladapi.h"
#include "regapi.h"
#include "regdata.h"
#include "laddata.h"

#ifdef __cplusplus
extern "C" {
#endif

extern LadData*Ladderdata;

void CycleScanning();//*******循环扫描************

long GetReg(char reg,int index);

void SetReg(char reg ,int index ,long value);

void SetRegBit(char reg,int index,int bit);

bool GetRegBit(char reg,int index,int bit);

void ClrRegBit(char reg,int index,int bit);

bool get_para_bit(int No);

void set_para_bit(int No);

void clr_para_bit(int No);

long get_para_reg(int No,int n=0);

void set_para_reg(int No,long value);

#ifdef __cplusplus
}
#endif

#endif // CYCLESCANNING_H
