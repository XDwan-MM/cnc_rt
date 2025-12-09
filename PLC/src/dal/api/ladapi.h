/**
 * @file ladapi.h
 * @brief ladapi.h 为业务层提供统一、线程安全的梯形图数据访问接口。
 * @version 1.0
 */
#ifndef LADAPI_H
#define LADAPI_H

#include "dalglobal.h"
#include "laddata.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief getLadDataPtr
 * @return LadData指针
 */
DAL_EXPORT LadData* getLadDataPtr();

#ifdef __cplusplus
}
#endif

#endif // LADAPI_H
