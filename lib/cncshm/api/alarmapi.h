#ifndef ALARMAPI_H
#define ALARMAPI_H

#include "shmglobal.h"
#include "alarmdata.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief setAlarm 设置（激活）当前报警
 * @param[in] type
 * @param[in] level
 * @param[in] id
 * @return
 */
SHM_EXPORT int setAlarm(int type, int level, int id);

/**
 * @brief delAlarm 清除当前报警
 * @param[in] type
 * @param[in] level
 * @param[in] id
 * @return
 */
SHM_EXPORT int delAlarm(int type, int level, int id);

/**
 * @brief cleanAllAlarm 清除所有当前报警
 * @param[in] type
 * @param[in] level
 * @return
 */
SHM_EXPORT int cleanAllAlarm(int type, int level);

/**
 * @brief getAlarmCount 获取当前报警数量
 * @param[in] type
 * @param[in] level
 * @param[in] count
 * @return
 */
SHM_EXPORT int getAlarmCount(int type, int level, int *count);

/**
 * @brief getAlarmId 获取索引的当前报警的id号
 * @param[in] type
 * @param[in] level
 * @param[in] index
 * @param[out] id
 * @return
 */
SHM_EXPORT int getAlarmId(int type, int level, int index, int *id);

SHM_EXPORT int getAlarmData(int idx, int *id, int *type, int *level, time_t *start);

SHM_EXPORT int getAlarmHisCount(int *count);

SHM_EXPORT int takeAlarmHisFront(int *id, int *type, int *level, time_t *start, time_t *end);

#ifdef __cplusplus
}
#endif

#endif // ALARMAPI_H
