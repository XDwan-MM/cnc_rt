#pragma once
#include "regdata.h"
#include <QSystemSemaphore>

class RegManager {
public:
    RegManager(RegData& regData);
    ~RegManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) { _isShmInit = isShmInit; }
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
    // 定时器计数器接口
    int setTimerCounterCurrent(RegType type, int idx, int value);
    int setTimerCounterPreset(RegType type, int idx, int value);
    int getTimerCounter(RegType type, int idx, RegTimerCounter* value);
    int getTimerCounterCurrent(RegType type, int idx, int* value);
    int getTimerCounterPreset(RegType type, int idx, int* value);

private:
    enum RegManagerError {
        REG_MANAGER_SUCCESS = 0,
        REG_MANAGER_SHM_INIT_FAILED = -1,
        REG_MANAGER_TYPE_ERROR = -2,
        REG_MANAGER_IDX_ERROR = -3,
        REG_MANAGER_BIT_ERROR = -4,
        REG_MANAGER_VALUE_ERROR = -5,
    };
    RegData& _regData;
    QSystemSemaphore _sem;
    bool _isShmInit = false;
};
