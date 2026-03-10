/******************************************************************** 
 * @file        var变量接口.h
 * @brief       为业务层提供统一、线程安全的变量数据访问接口。
 * 1、随控制指令而发送变化的数据，即每条G代码指令执行时都有可能发生变化。
 * 2、通过对特定的变量的访问，可以得到系统的很多状态数据。
 * @version     V1.01
 * @date        2025/12/31
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef VARAPI_H
#define VARAPI_H

#include "shmglobal.h"
#include "vardatadef.h"
#include "vardata.h"

#ifdef __cplusplus
extern "C" {
#endif

SHM_EXPORT VarData* getVarDataPtr();

/// @brief 获取 var 变量
/// @param type 变量类型，（VAR_TYPE_AXIS、VAR_TYPE_CHANNEL、VAR_TYPE_SYSTEM）
/// @param no 轴号或通道号（VAR_TYPE_AXIS：0~3，VAR_TYPE_CHANNEL：0）
/// @param index 索引号或枚举类型（VAR_TYPE_AXIS：0~99，VAR_TYPE_CHANNEL：0~1999，VAR_TYPE_SYSTEM：0~9999）
/// @param value 变量值
/// @param clientNo 网络连接号（默认为0）
/// @return 变量API错误码
/// VAR_MANAGER_SUCCESS = 0,            // 成功
/// VAR_MANAGER_TYPE_ERR = -1, 	        // 函数与变量类型不匹配 
/// VAR_MANAGER_AXISNO_ERR = -2,		// 轴号超过限制
/// VAR_MANAGER_CHANNO_ERR = -3,		// 通道号超过限制
/// VAR_MANAGER_AXISINDEX_ERR = -4,		// 轴变量索引号超过限制
/// VAR_MANAGER_CHANINDEX_ERR = -5,		// 通道变量索引号超过限制
/// VAR_MANAGER_SYSINDEX_ERR = -6,		// 系统变量索引号超过限制
/// VAR_MANAGER_BITNO_ERR = -7,			// 变量位号索引号超过限制
/// VAR_MANAGER_POINT_NULL_ERR = -8,	// 变量值指针为空
/// VAR_MANAGER_SHM_INIT_FAILED = -9,   // 共享内存未初始化
SHM_EXPORT int getVarValue(int type, int no, int index, void* value, int clientNo);

/// @brief 设置 var 变量
/// @param type 变量类型，（VAR_TYPE_AXIS、VAR_TYPE_CHANNEL、VAR_TYPE_SYSTEM）
/// @param no 轴号或通道号（VAR_TYPE_AXIS：0~3，VAR_TYPE_CHANNEL：0）
/// @param index 索引号或枚举类型（VAR_TYPE_AXIS：0~99，VAR_TYPE_CHANNEL：0~1999，VAR_TYPE_SYSTEM：0~9999）
/// @param value 变量值
/// @param clientNo 网络连接号（默认为0）
/// @return 变量API错误码
/// VAR_MANAGER_SUCCESS = 0,            // 成功
/// VAR_MANAGER_TYPE_ERR = -1, 	        // 函数与变量类型不匹配 
/// VAR_MANAGER_AXISNO_ERR = -2,		// 轴号超过限制
/// VAR_MANAGER_CHANNO_ERR = -3,		// 通道号超过限制
/// VAR_MANAGER_AXISINDEX_ERR = -4,		// 轴变量索引号超过限制
/// VAR_MANAGER_CHANINDEX_ERR = -5,		// 通道变量索引号超过限制
/// VAR_MANAGER_SYSINDEX_ERR = -6,		// 系统变量索引号超过限制
/// VAR_MANAGER_BITNO_ERR = -7,			// 变量位号索引号超过限制
/// VAR_MANAGER_POINT_NULL_ERR = -8,	// 变量值指针为空
/// VAR_MANAGER_SHM_INIT_FAILED = -9,   // 共享内存未初始化
SHM_EXPORT int setVarValue(int type, int no, int index, const void* value, int clientNo);

/// @brief 
/// @param no：通道号
/// @param index：通道数据类型，gncdatadef 中定义的枚举类型
/// @param sub_index：通道轴号/通道主轴号，当type为VAR_CHAN_LAX、VAR_CHAN_AXIS_NAME、VAR_CHAN_SPDL_NAME、
/// VAR_CHAN_BP_POS、 VAR_CHAN_MODAL、VAR_CHAN_SPDL_LAX、VAR_CHAN_SPDL_PARA_LAX、VAR_CHAN_CMD_SPDL_SPEED、
/// VAR_CHAN_ACT_SPDL_SPEED、VAR_CHAN_SPDL_OVERRIDE、VAR_CHAN_MODAL 时需要配置 index。其他情况无限制。
/// @param value：返回值
/// @param clientNo：网络连接号
/// @return 
SHM_EXPORT int getVarChanValue(int no, int index, int sub_index, void* value, int clientNo);

/// @brief 
/// @param no：通道号
/// @param index：通道数据类型，gncdatadef 中定义的枚举类型
/// @param sub_index：通道轴号/通道主轴号，无限制
/// @param value：返回值
/// @param clientNo：网络连接号
/// @return 
SHM_EXPORT int setVarChanValue(int no, int index, int sub_index, void* value, int clientNo);

SHM_EXPORT int getVarAxisValue(int no, int index, void* value, int clientNo);
SHM_EXPORT int setVarAxisValue(int no, int index, void* value, int clientNo);
SHM_EXPORT int getVarSysValue(int index, void* value, int clientNo);
SHM_EXPORT int setVarSysValue(int index, void* value, int clientNo);

SHM_EXPORT int getVarInt(int type, int no, int index, int* value, int clientNo);
SHM_EXPORT int setVarInt(int type, int no, int index, int value, int clientNo);
SHM_EXPORT int getVarDouble(int type, int no, int index, double* value, int clientNo);
SHM_EXPORT int setVarDouble(int type, int no, int index, double value, int clientNo);
SHM_EXPORT int getVarChar(int type, int no, int index, char* value, int clientNo);
SHM_EXPORT int setVarChar(int type, int no, int index, const char* value, int clientNo);
SHM_EXPORT int getVarUint(int type, int no, int index, unsigned int* value, int clientNo);
SHM_EXPORT int setVarUint(int type, int no, int index, unsigned int value, int clientNo);

SHM_EXPORT int getVarAxisInt(int no, int index, int* value, int clientNo);
SHM_EXPORT int setVarAxisInt(int no, int index, int value, int clientNo);
SHM_EXPORT int getVarAxisDouble(int no, int index, double* value, int clientNo);
SHM_EXPORT int setVarAxisDouble(int no, int index, double value, int clientNo);
SHM_EXPORT int getVarAxisChar(int no, int index, char* value, int clientNo);
SHM_EXPORT int setVarAxisChar(int no, int index, const char* value, int clientNo);
SHM_EXPORT int getVarAxisUint(int no, int index, unsigned int* value, int clientNo);
SHM_EXPORT int setVarAxisUint(int no, int index, unsigned int value, int clientNo);

SHM_EXPORT int getVarChanInt(int no, int index, int* value, int clientNo);
SHM_EXPORT int setVarChanInt(int no, int index, int value, int clientNo);
SHM_EXPORT int getVarChanDouble(int no, int index, double* value, int clientNo);
SHM_EXPORT int setVarChanDouble(int no, int index, double value, int clientNo);
SHM_EXPORT int getVarChanChar(int no, int index, char* value, int clientNo);
SHM_EXPORT int setVarChanChar(int no, int index, const char* value, int clientNo);
SHM_EXPORT int getVarChanUint(int no, int index, unsigned int* value, int clientNo);
SHM_EXPORT int setVarChanUint(int no, int index, unsigned int value, int clientNo);

SHM_EXPORT int getVarSysInt(int index, int* value, int clientNo);
SHM_EXPORT int setVarSysInt(int index, int value, int clientNo);
SHM_EXPORT int getVarSysDouble(int index, double* value, int clientNo);
SHM_EXPORT int setVarSysDouble(int index, double value, int clientNo);
SHM_EXPORT int getVarSysChar(int index, char* value, int clientNo);
SHM_EXPORT int setVarSysChar(int index, const char* value, int clientNo);
SHM_EXPORT int getVarSysUint(int index, unsigned int* value, int clientNo);
SHM_EXPORT int setVarSysUint(int index, unsigned int value, int clientNo);

// int VarSetBit(int type, int no, int index, int bit, int clientNo);
// int VarClrBit(int type, int no, int index, int bit, int clientNo);

#ifdef __cplusplus
}
#endif

#endif // VARAPI_H
