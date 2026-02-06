#pragma once
#include "regdata.h"

class RegManager {
public:
    RegManager(RegData& regData);
    ~RegManager();

    // 寄存器接口
    int getRegNum(RegType type, int *num);
    int setRegBit(RegType type, int idx, int bit);
    int clearRegBit(RegType type, int idx, int bit);
    int getRegBit(RegType type, int idx, int bit, bool *value);
    int setRegValue8(RegType type, int idx, unsigned char value);
    int setRegValue16(RegType type, int idx, unsigned short value);
    int setRegValue32(RegType type, int idx, unsigned int value);
    int getRegValue8(RegType type, int idx, unsigned char *value);
    int getRegValue16(RegType type, int idx, unsigned short *value);
    int getRegValue32(RegType type, int idx, unsigned int* value);

private:
    RegData& _regData;
};
