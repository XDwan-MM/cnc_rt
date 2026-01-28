/**
 * @file parapi.h
 * @brief parapi.h 为业务层提供统一、线程安全的参数数据访问接口。
 * @version 1.0
 */
#ifndef PARAPI_H
#define PARAPI_H

#include "shmglobal.h"
#include "pardata.h"
#include "gncdatadef.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************** !!!  接口用例 !!!  *********************/
/**
ParamUnit data = {};       // 实例参数单元
getParData(PARAMAN_FILE_AXIS, 0, PAR_NCU_CYCLE, &data);    // 调用接口，传入需要获取的值
int c = data.value.val_int;     // 获取该参数所对应的数据类型
*/

/**
 * @brief getParData 获取索引得到的参数单元
 * @param file[in]  参数类别    :ncu\mac\chan等
 * @param sub[in]   子表号     :轴X对应0,轴Y对应1
 * @param index[in] 索引号     :对应配置文件中的索引号
 * @param data[out] 参数单元    :指针
 * @return
 * 错误码
 */
SHM_EXPORT int getParData(int file, int sub, int index, ParamUnit *data);

/**
 * @brief setParData 设置索引下的参数值
 * @param file[in]  参数类别    :ncu\mac\chan等
 * @param sub[in]   子表号     :轴X对应0,轴Y对应1
 * @param index[in] 索引号     :对应配置文件中的索引号
 * @param data[in]  参数单元    :需要改成的值
 * @return
 * 错误码
 */
SHM_EXPORT int setParData(int file, int sub, int index, ParamUnit data);




SHM_EXPORT int getParProp(int file, int sub, int index, int propType, ParamUnit *val);
SHM_EXPORT int setParProp(int file, int sub, int index, int propType, ParamUnit val);



#ifdef __cplusplus
}
#endif

#endif // PARAPI_H
