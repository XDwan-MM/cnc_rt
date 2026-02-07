/**
 * @file regapi.h
 * @brief regapi.h 为业务层提供统一、线程安全的寄存器访问接口。所有函数返回 int，统一错误码见 RegApiError 枚举定义。
 * @version 1.0
 */
#ifndef REGAPI_H
#define REGAPI_H

#include "shmglobal.h"
#include "regdata.h"
#include <stdbool.h>

/**
 * @brief 寄存器API错误码
 */
enum RegApiError {
    REG_API_SUCCESS = 0,          // 成功
    REG_API_SHM_INIT_FAILED = -1, // 共享内存初始化失败
    REG_API_TYPE_ERROR = -2,      // 类型错误
    REG_API_IDX_ERROR = -3,       // 索引错误
    REG_API_BIT_ERROR = -4,       // 位错误
    REG_API_VALUE_ERROR = -5,     // 值错误
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取寄存器数目
 *
 * @param type 寄存器类型
 * @param num 寄存器最大数目
 * @return int 错误码
 */
SHM_EXPORT int getRegNum(RegType type, int* num);

/**
 * @brief 设置寄存器位
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param bit 寄存器位
 * @return int 错误码
 */
SHM_EXPORT int setRegBit(RegType type, int idx, int bit);

/**
 * @brief 清除寄存器位
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param bit 寄存器位
 * @return int 错误码
 */
SHM_EXPORT int clearRegBit(RegType type, int idx, int bit);

/**
 * @brief 获取寄存器位
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param bit 寄存器位
 * @param[out] value
 * @return int 错误码
 */
SHM_EXPORT int getRegBit(RegType type, int idx, int bit, bool* value);

/**
 * @brief 设置寄存器8位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int setRegValue8(RegType type, int idx, unsigned char value);

/**
 * @brief 设置寄存器16位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int setRegValue16(RegType type, int idx, unsigned short value);

/**
 * @brief 设置寄存器32位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int setRegValue32(RegType type, int idx, unsigned int value);

/**
 * @brief 获取寄存器8位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getRegValue8(RegType type, int idx, unsigned char* value);

/**
 * @brief 获取寄存器16位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getRegValue16(RegType type, int idx, unsigned short* value);

/**
 * @brief 获取寄存器32位值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getRegValue32(RegType type, int idx, unsigned int* value);

/**
 * @brief 设置定时器计数器当前值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int setTimerCounterCurrent(RegType type, int idx, int value);

/**
 * @brief 设置定时器计数器预设值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int setTimerCounterPreset(RegType type, int idx, int value);

/**
 * @brief 获取定时器计数器
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getTimerCounter(RegType type, int idx, RegTimerCounter* value);

/**
 * @brief 获取定时器计数器当前值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getTimerCounterCurrent(RegType type, int idx, int* value);

/**
 * @brief 获取定时器计数器预设值
 *
 * @param type 寄存器类型
 * @param idx 寄存器索引
 * @param[out] value 寄存器值
 * @return int 错误码
 */
SHM_EXPORT int getTimerCounterPreset(RegType type, int idx, int* value);

#ifdef __cplusplus
}
#endif

#endif // REGAPI_H
