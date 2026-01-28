#ifndef HMISYSAPI_H
#define HMISYSAPI_H

#include "shmglobal.h"
#include "hmisysdata.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief getHmiSysDataPtr 获取界面数据指针
 * @param[out] hmiSysData
 * @return
 */
SHM_EXPORT int getHmiSysDataPtr(HmiSysData *hmiSysData);

/**
 * @brief setNcFilePath 设置加载的NC程序目录
 * @param[in] ncFilePath
 * @return
 */
SHM_EXPORT int setNcFilePath(char ncFilePath[]);

/**
 * @brief getNcFilePath 获取加载的NC程序目录
 * @param[out] ncFilePath
 * @return
 */
SHM_EXPORT int getNcFilePath(char ncFilePath[]);

/**
 * @brief setMdiContent 设置MDI程序内容
 * @param[in] mdiContent
 * @return
 */
SHM_EXPORT int setMdiContent(char mdiContent[]);

/**
 * @brief getMdiContent 获取MDI程序内容
 * @param[out] mdiContent
 * @return
 */
SHM_EXPORT int getMdiContent(char mdiContent[]);

/**
 * @brief setPitChData 设置全部螺补数据
 * @param[in] pitchData
 * @return
 */
SHM_EXPORT int setAllPitChData(float *pitchData);

/**
 * @brief setPitchValue 设置某个螺补值
 * @param[in] index
 * @param[in] val
 * @return
 */
SHM_EXPORT int setPitchValue(int index, float val);

/**
 * @brief getPitchData 获取指定范围内的螺补数组
 * @param[in] start     获取数组的起点
 * @param[in] num       获取数组的个数
 * @param[out] pitchData    得到数组
 * @return
 */
SHM_EXPORT int getPitchData(int start, int num, float *pitchData);


/**
 * @brief setCrdsValue 设置坐标系数据的值
 * @param[in] type      坐标系类型
 * @param[in] axis      轴号(三轴：0~2)
 * @param[out] value
 * @return
 */
SHM_EXPORT int setCrdsValue(int type, int axis, double value);

/**
 * @brief getCrdsValue 获取坐标系数据的值
 * @param[in] type      坐标系类型
 * @param[in] axis      轴号(三轴:0~2)
 * @param[out] value
 * @return
 */
SHM_EXPORT int getCrdsValue(int type, int axis, double *value);

#ifdef __cplusplus
}
#endif

#endif // HMISYSAPI_H
