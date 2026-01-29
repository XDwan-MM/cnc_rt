#include "alarmapi.h"
#include "alarmmanager.h"
#include "shmmanager.h"

int setAlarm(int type, int level, int id)
{
    return ShmManager::instance().alarmManager()->setAlarm(type, level, id);
}

int delAlarm(int type, int level, int id)
{
    return ShmManager::instance().alarmManager()->delAlarm(type, level, id);
}

int cleanAllAlarm(int type, int level)
{
    return ShmManager::instance().alarmManager()->cleanAllAlarm(type, level);
}

int getAlarmCount(int type, int level, int *count)
{
    return ShmManager::instance().alarmManager()->getAlarmCount(type, level, count);
}

int getAlarmId(int type, int level, int index, int *id)
{
    return ShmManager::instance().alarmManager()->getAlarmId(type, level, index, id);
}

int getAlarmData(int idx, int *id, int *type, int *level, time_t *start)
{
    return ShmManager::instance().alarmManager()->getAlarmData(idx, id, type, level, start);
}

int getAlarmHisCount(int *count)
{
    return ShmManager::instance().alarmManager()->getAlarmHisCount(count);
}

int takeAlarmHisFront(int *id, int *type, int *level, time_t *start, time_t *end)
{
    return ShmManager::instance().alarmManager()->takeAlarmHisFront(id, type, level, start, end);
}
