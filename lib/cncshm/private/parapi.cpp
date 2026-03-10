#include "parapi.h"
#include "parmanager.h"
#include "shmmanager.h"

SHM_EXPORT ParData* getParDataPtr(){
    return ShmManager::instance().parManager()->getParDataPtr();
}

int getParData(int file, int sub, int index, ParamUnit *data)
{
    return ShmManager::instance().parManager()->getParData(file, sub, index, data);
}

int setParData(int file, int sub, int index, ParamUnit data)
{
    return ShmManager::instance().parManager()->setParData(file, sub, index, data);
}

int getParProp(int file, int sub, int index, int propType, ParamUnit *val)
{
    return ShmManager::instance().parManager()->getParProp(file, sub, index, propType, val);
}

int setParProp(int file, int sub, int index, int propType, ParamUnit val)
{
    return ShmManager::instance().parManager()->setParProp(file, sub, index, propType, val);
}
