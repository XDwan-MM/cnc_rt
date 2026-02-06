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

#ifdef __cplusplus
}
#endif

#endif // REGAPI_H
