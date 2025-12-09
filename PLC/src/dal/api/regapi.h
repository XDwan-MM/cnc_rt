/**
 * @file regapi.h
 * @brief regapi.h 为业务层提供统一、线程安全的寄存器访问接口。所有函数返回 int，统一错误码见 RegApiError 枚举定义。
 * @version 1.0
 */
#ifndef REGAPI_H
#define REGAPI_H

#include "dalglobal.h"
#include "regdata.h"

enum RegApiError {
    REG_API_SUCCESS = 0,
    REG_API_SHM_INIT_FAILED = -1,
    REG_API_TYPE_ERROR = -2,
    REG_API_IDX_ERROR = -3,
    REG_API_BIT_ERROR = -4,
    REG_API_VALUE_ERROR = -5,
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置寄存器位
 * 
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param bit 寄存器位
 * @return int 错误码
 */
DAL_EXPORT int setRegBit(RegType type, int idx, int bit);
DAL_EXPORT int clearRegBit(RegType type, int idx, int bit);
DAL_EXPORT int getRegBit(RegType type, int idx, int bit, int* value);
DAL_EXPORT int setRegValue8(RegType type, int idx, int value);
DAL_EXPORT int setRegValue16(RegType type, int idx, int value);
DAL_EXPORT int setRegValue32(RegType type, int idx, int value);
DAL_EXPORT int getRegValue8(RegType type, int idx, int* value);
DAL_EXPORT int getRegValue16(RegType type, int idx, int* value);
DAL_EXPORT int getRegValue32(RegType type, int idx, int* value);
DAL_EXPORT int setTimerCounterCurrent(RegType type, int idx, int value);
DAL_EXPORT int setTimerCounterPreset(RegType type, int idx, int value);
DAL_EXPORT int getTimerCounter(RegType type, int idx, RegTimerCounter* value);
DAL_EXPORT int getTimerCounterCurrent(RegType type, int idx, int* value);
DAL_EXPORT int getTimerCounterPreset(RegType type, int idx, int* value);

#ifdef __cplusplus
}
#endif

#endif // REGAPI_H
