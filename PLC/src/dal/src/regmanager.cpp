#include "regmanager.h"
#include "semaphorelocker.h"
RegManager::RegManager(RegData& regData)
    : _regData(regData)
    , _sem("RegDataSem", 1, QSystemSemaphore::Open) {
}

RegManager::~RegManager() {
}

int RegManager::setRegBit(RegType type, int idx, int bit) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    if (bit < 0) {
        return REG_MANAGER_BIT_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regR[idx] |= (1 << bit);
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regW[idx] |= (1 << bit);
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regD[idx] |= (1 << bit);
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regX[idx] |= (1 << bit);
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regY[idx] |= (1 << bit);
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regF[idx] |= (1 << bit);
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regG[idx] |= (1 << bit);
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regI[idx] |= (1 << bit);
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regQ[idx] |= (1 << bit);
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regB[idx] |= (1 << bit);
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regK[idx] |= (1 << bit);
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::clearRegBit(RegType type, int idx, int bit) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    if (bit < 0) {
        return REG_MANAGER_BIT_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regR[idx] &= ~(1 << bit);
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regW[idx] &= ~(1 << bit);
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regD[idx] &= ~(1 << bit);
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regX[idx] &= ~(1 << bit);
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regY[idx] &= ~(1 << bit);
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regF[idx] &= ~(1 << bit);
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regG[idx] &= ~(1 << bit);
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regI[idx] &= ~(1 << bit);
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regQ[idx] &= ~(1 << bit);
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regB[idx] &= ~(1 << bit);
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regK[idx] &= ~(1 << bit);
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::getRegBit(RegType type, int idx, int bit, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    if (bit < 0) {
        return REG_MANAGER_BIT_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regR[idx] >> bit) & 1;
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regW[idx] >> bit) & 1;
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regD[idx] >> bit) & 1;
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regX[idx] >> bit) & 1;
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regY[idx] >> bit) & 1;
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regF[idx] >> bit) & 1;
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::setRegValue8(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regR[idx] = value;
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regX[idx] = value;
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regY[idx] = value;
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regF[idx] = value;
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regG[idx] = value;
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regI[idx] = value;
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regQ[idx] = value;
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regK[idx] = value;
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::setRegValue16(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regW[idx] = value;
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::setRegValue32(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regD[idx] = value;
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regB[idx] = value;
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
            break;
    }
    return 0;
}

int RegManager::setTimerCounterCurrent(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        _regData.regT[idx].current = value;
    } else {
        _regData.regC[idx].current = value;
    }
    return 0;
}

int RegManager::setTimerCounterPreset(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        _regData.regT[idx].preset = value;
    } else {
        _regData.regC[idx].preset = value;
    }
    return 0;
}

int RegManager::getTimerCounter(RegType type, int idx, RegTimerCounter* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value->current < 0 || value->preset < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx];
    } else {
        *value = _regData.regC[idx];
    }
    return 0;
}

int RegManager::getRegValue8(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regR[idx]);
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regX[idx]);
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regY[idx]);
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regF[idx]);
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regG[idx]);
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regI[idx]);
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regQ[idx]);
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = (_regData.regB[idx] & 0xFF);
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned char>(_regData.regK[idx]);
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::getRegValue16(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = static_cast<unsigned short>(_regData.regW[idx]);
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::getRegValue32(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regD[idx];
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regB[idx];
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return 0;
}

int RegManager::getTimerCounterCurrent(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx].current;
    } else {
        *value = _regData.regC[idx].current;
    }
    return 0;
}

int RegManager::getTimerCounterPreset(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx].preset;
    } else {
        *value = _regData.regC[idx].preset;
    }
    return 0;
}
