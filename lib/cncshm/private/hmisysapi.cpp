#include "hmisysapi.h"
#include "shmmanager.h"

int getHmiSysDataPtr(HmiSysData *hmiSysData)
{
    return ShmManager::instance().hmiSysManager()->getHmiSysData(hmiSysData);
}

int setNcFilePath(char ncFilePath[])
{
    return ShmManager::instance().hmiSysManager()->setNcFilePath(ncFilePath);
}

int getNcFilePath(char ncFilePath[])
{
    return ShmManager::instance().hmiSysManager()->getNcFilePath(ncFilePath);
}

int setMdiContent(char mdiContent[])
{
    return ShmManager::instance().hmiSysManager()->setMdiContent(mdiContent);
}

int getMdiContent(char mdiContent[])
{
    return ShmManager::instance().hmiSysManager()->getMdiContent(mdiContent);
}

int setAllPitChData(float *pitchData)
{
    return ShmManager::instance().hmiSysManager()->setAllPitchData(pitchData);
}

int setPitchValue(int index, float val)
{
    return ShmManager::instance().hmiSysManager()->setPitchValue(index, val);
}

int getPitchData(int start, int num, float *pitchData)
{
    return ShmManager::instance().hmiSysManager()->getPitchData(start, num, pitchData);
}

int setCrdsValue(int type, int axis, double value)
{
    return ShmManager::instance().hmiSysManager()->setCrdsValue(type, axis, value);
}

int getCrdsValue(int type, int axis, double *value)
{
    return ShmManager::instance().hmiSysManager()->getCrdsValue(type, axis, value);
}
