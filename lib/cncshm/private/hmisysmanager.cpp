#include "hmisysmanager.h"
#include "semaphorelocker.h"
#include <QDebug>

HmiSysManager::HmiSysManager(HmiSysData &hmiSysData)
    : m_hmiSysData (hmiSysData)
    , m_isShmInit (false)
    , m_sem("HmiSysData", 1, QSystemSemaphore::Open)
{
}

void HmiSysManager::setIsShmInit(bool isShmInit)
{
    m_isShmInit = isShmInit;
}

int HmiSysManager::getHmiSysData(HmiSysData *hmiSysData)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (hmiSysData == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    *hmiSysData = m_hmiSysData;
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::setNcFilePath(char *ncFilePath)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (ncFilePath == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    memcpy(m_hmiSysData.ncFilePath, ncFilePath, sizeof(char) * NCFILEPATH_CHAR_LEN);
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::getNcFilePath(char *ncFilePath)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (ncFilePath == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    memcpy(ncFilePath, m_hmiSysData.ncFilePath, sizeof(char) * NCFILEPATH_CHAR_LEN);
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::setMdiContent(char *mdiContent)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (mdiContent == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    memcpy(m_hmiSysData.mdiContent, mdiContent, sizeof(char) * MDICONTENT_CHAR_LEN);
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::getMdiContent(char *mdiContent)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (mdiContent == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    memcpy(mdiContent, m_hmiSysData.mdiContent, sizeof(char) * MDICONTENT_CHAR_LEN);
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::setAllPitchData(float *pitchData)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (pitchData == nullptr)
    {
        return HMISYS_MANAGER_POINT_ERROR;
    }
    SemaphoreLocker locker(m_sem);
    memcpy(m_hmiSysData.pitchData, pitchData, sizeof(float) * PITCH_COUNT_MAX);
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::setPitchValue(int index, float val)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (index < 0 || index >= PITCH_COUNT_MAX)
    {
        return HMISYS_PITCH_INDEX_ERROR;
    }
    SemaphoreLocker locker(m_sem);

    m_hmiSysData.pitchData[index] = val;

    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::getPitchData(int start, int num, float *pitchData)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (start < 0 || num < 0)
    {
        return HMISYS_PITCH_INDEX_ERROR;
    }
    if (start + num > PITCH_COUNT_MAX)
    {
        return HMISYS_PITCH_CAPACITY_ERROR;
    }
    SemaphoreLocker locker(m_sem);

    memcpy(pitchData, m_hmiSysData.pitchData + start, sizeof(float) * num);

    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::setCrdsValue(int type, int axis, double value)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type >= CRDS_TYPE_COUNT)
    {
        return HMISYS_CRDS_TYPE_ERROR;
    }
    if (axis < 0 || axis >= AXIS_COUNT_MAX)
    {
        return HMISYS_CRDS_AXIS_ERROR;
    }

    m_hmiSysData.coordData[type][axis] = value;
    return HMISYS_MANAGER_SUCCESS;
}

int HmiSysManager::getCrdsValue(int type, int axis, double *value)
{
    if (!m_isShmInit)
    {
        return HMISYS_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type >= CRDS_TYPE_COUNT)
    {
        return HMISYS_CRDS_TYPE_ERROR;
    }
    if (axis < 0 || axis >= AXIS_COUNT_MAX)
    {
        return HMISYS_CRDS_AXIS_ERROR;
    }

    *value = m_hmiSysData.coordData[type][axis];
    return HMISYS_MANAGER_SUCCESS;
}


