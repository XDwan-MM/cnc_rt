#ifndef ALARMMANAGER_H
#define ALARMMANAGER_H

#pragma once
#include "alarmdata.h"
#include <QSystemSemaphore>

class AlarmManager {
public:
    AlarmManager(AlarmDataBuffer &buffer);
    ~AlarmManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) {
        _isShmInit = isShmInit;
    }

    // 不支持all
    int setAlarm(int type, int level, int id);
    int delAlarm(int type, int level, int id);

    // 支持all
    int cleanAllAlarm(int type, int level);
    int getAlarmCount(int type, int level, int *count);
    int getAlarmId(int type, int level, int index, int *id);
    int getAlarmData(int index, int *id, int *type, int *level, time_t *start);

    int getAlarmHisCount(int *count);
    int takeAlarmHisFront(int *id, int *type, int *level, time_t *start, time_t *end);

private:
    enum AlarmManagerError {
        ALARM_MANAGER_SUCCESS           = 0,    // 成功
        ALARM_MANAGER_SHM_INIT_FAILED   = -1,   // 共享内存初始化失败
        ALARM_MANAGER_NULL_POINTER      = -2,   // 空指针
        ALARM_MANAGER_NOT_FIND_TARGET   = -3,   // 未找到目标
        ALARM_MANAGER_COUNT_MAX_FAILED  = -4,   // 容量已满
        ALARM_MANAGER_TYPE_FAILED       = -5,   // 报警类型错误
        ALARM_MANAGER_LEVEL_FAILED      = -6,   // 报警等级错误
        ALARM_MANAGER_INDEX_ERROR       = -7,   // 索引值错误
        ALARM_MANAGER_REPEAT_ERROR      = -8,   // 报警已存在
    };

    AlarmDatas &m_alms;
    AlarmHisDatas &m_hiss;
    AlarmDatas m_emptyAlms;
    bool _isShmInit = false;
    QSystemSemaphore _sem;

    inline void emptyData(AlarmDataDef &data);
};


#endif // ALARMMANAGER_H
