#include "parmanager.h"
#include "semaphorelocker.h"

ParManager::ParManager(ParData& parData)
    : _parData(parData) {
}

ParManager::~ParManager() {
}

int ParManager::getParData(int file, int sub, int index, ParamUnit *data)
{
    return getParProp(file, sub, index, PARA_PROP_VALUE, data);
}

int ParManager::setParData(int file, int sub, int index, ParamUnit data)
{
    return setParProp(file, sub, index, PARA_PROP_VALUE, data);
}

int ParManager::getParProp(int file, int sub, int index, int propType, ParamUnit *val)
{
    if (!_isShmInit) {
        return PAR_MANAGER_ERR_SHM_INIT_FAILED;
    }
    if (!val) {
        return PAR_MANAGER_ERR_NULL_POINTER;
    }
    if (index < 0)
    {
        return PAR_MANAGER_ERR_INDEX;
    }
    switch (file)
    {
    case PARAMAN_FILE_NCU:
        if (index >= PARAMAN_FILE_NCU_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        *val = _parData.par[propType].par_ncu[index];
        break;
    case PARAMAN_FILE_MAC:
        if (index >= PARAMAN_FILE_MAC_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        *val = _parData.par[propType].par_mac[index];
        break;
    case PARAMAN_FILE_CHAN:
        if (index >= PARAMAN_FILE_CHAN_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_CHAN_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        *val = _parData.par[propType].par_chan[sub][index];
        break;
    case PARAMAN_FILE_AXIS:
        if (index >= PARAMAN_FILE_AXIS_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_AXES_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        *val = _parData.par[propType].par_axis[sub][index];
        break;
    case PARAMAN_FILE_ACMP:
        if (index >= PARAMAN_FILE_ACMP_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_AXES_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        *val = _parData.par[propType].par_acmp[sub][index];
        break;
    case PARAMAN_FILE_CFG:
        if (index >= PARAMAN_FILE_CFG_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_NCBRD_CFG_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        *val = _parData.par[propType].par_cfg[sub][index];
        break;
    case PARAMAN_FILE_TABLE:
        if (index >= PARAMAN_FILE_TABLE_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        *val = _parData.par[propType].par_table[index];
        break;
    default:
        return PAR_MANAGER_ERR_TYPE;
    }
    return PAR_MANAGER_SUCCESS;
}

int ParManager::setParProp(int file, int sub, int index, int propType, ParamUnit val)
{
    if (!_isShmInit) {
        return PAR_MANAGER_ERR_SHM_INIT_FAILED;
    }
    if (index < 0)
    {
        return PAR_MANAGER_ERR_INDEX;
    }
    switch (file)
    {
    case PARAMAN_FILE_NCU:
        if (index >= PARAMAN_FILE_NCU_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        _parData.par[propType].par_ncu[index] = val;
        break;
    case PARAMAN_FILE_MAC:
        if (index >= PARAMAN_FILE_MAC_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        _parData.par[propType].par_mac[index] = val;
        break;
    case PARAMAN_FILE_CHAN:
        if (index >= PARAMAN_FILE_CHAN_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_CHAN_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        _parData.par[propType].par_chan[sub][index] = val;
        break;
    case PARAMAN_FILE_AXIS:
        if (index >= PARAMAN_FILE_AXIS_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_AXES_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        _parData.par[propType].par_axis[sub][index] = val;
        break;
    case PARAMAN_FILE_ACMP:
        if (index >= PARAMAN_FILE_ACMP_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_AXES_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        _parData.par[propType].par_acmp[sub][index] = val;
        break;
    case PARAMAN_FILE_CFG:
        if (index >= PARAMAN_FILE_CFG_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        if (sub < 0 || sub >= SYS_NCBRD_CFG_NUM)
        {
            return PAR_MANAGER_ERR_SUB;
        }
        _parData.par[propType].par_cfg[sub][index] = val;
        break;
    case PARAMAN_FILE_TABLE:
        if (index >= PARAMAN_FILE_TABLE_MAX)
        {
            return PAR_MANAGER_ERR_INDEX;
        }
        _parData.par[propType].par_table[index] = val;
        break;
    default:
        return PAR_MANAGER_ERR_TYPE;
    }
    return PAR_MANAGER_SUCCESS;
}
