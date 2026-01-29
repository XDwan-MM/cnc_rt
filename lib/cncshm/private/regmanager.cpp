#include "regmanager.h"
// #include "semaphorelocker.h"

RegManager::RegManager(RegData& regData)
    : _regData(regData)
    , _sem("RegDataSem", 1, QSystemSemaphore::Open) {
}

RegManager::~RegManager() {
}

int RegManager::getRegNum(RegType type, int* num) {
    switch (type) {
        case REG_R: // 对应 'R' (ASCII 82)
            *num = REG_R_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_W: // 对应 'W' (ASCII 87)
            *num = REG_W_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_D: // 对应 'D' (ASCII 68)
            *num = REG_D_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_X: // 对应 'X' (ASCII 88)
            *num = REG_X_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_Y: // 对应 'Y' (ASCII 89)
            *num = REG_Y_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_F: // 对应 'F' (ASCII 70)
            *num = REG_F_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_G: // 对应 'G' (ASCII 71)
            *num = REG_G_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_I: // 对应 'I' (ASCII 73)
            *num = REG_I_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_Q: // 对应 'Q' (ASCII 81)
            *num = REG_Q_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_B: // 对应 'B' (ASCII 66)
            *num = REG_B_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_K: // 对应 'K' (ASCII 75)
            *num = REG_K_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_T: // 对应 'T' (ASCII 84)
            *num = REG_T_NUM;
            return REG_MANAGER_SUCCESS;
        case REG_C: // 对应 'C' (ASCII 67)
            *num = REG_C_NUM;
            return REG_MANAGER_SUCCESS;
        default:    // 无效类型
            return REG_MANAGER_TYPE_ERROR;
    }
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
    // SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_A:
            if (idx >= REG_A_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regA[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regR[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regW[idx] |= static_cast<unsigned short>(1U << bit);
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regD[idx] |= static_cast<unsigned int>(1U << bit);
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regX[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regY[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regF[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regG[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regI[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regQ[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regB[idx] |= static_cast<unsigned int>(1U << bit);
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regK[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
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
        case REG_A:
            if (idx >= REG_A_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regA[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regR[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regW[idx] &= static_cast<unsigned short>(~(1U << bit));
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regD[idx] &= static_cast<unsigned int>(~(1U << bit));
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regX[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regY[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regF[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regG[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regI[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regQ[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_B:
            if (idx >= REG_B_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regB[idx] &= static_cast<unsigned int>(~(1U << bit));
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            _regData.regK[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getRegBit(RegType type, int idx, int bit, bool* value) {
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
        case REG_A:
            if (idx >= REG_A_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regA[idx] >> bit) & 1U;
            break;
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regR[idx] >> bit) & 1U;
            break;
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 16) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regW[idx] >> bit) & 1U;
            break;
        case REG_D:
            if (idx >= REG_D_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 32) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regD[idx] >> bit) & 1U;
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regX[idx] >> bit) & 1U;
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regY[idx] >> bit) & 1U;
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            if (bit >= 8) {
                return REG_MANAGER_BIT_ERROR;
            }
            *value = (_regData.regF[idx] >> bit) & 1U;
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::setRegValue8(RegType type, int idx,  unsigned char value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_A:
            if (idx >= REG_A_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            _regData.regA[idx] = value;
            break;
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
    return REG_MANAGER_SUCCESS;
}

int RegManager::setRegValue16(RegType type, int idx,  unsigned short value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
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
    return REG_MANAGER_SUCCESS;
}

int RegManager::setRegValue32(RegType type, int idx,  unsigned int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
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
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::setTimerCounterCurrent(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        _regData.regT[idx].current = value;
    } else {
        _regData.regC[idx].current = value;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::setTimerCounterPreset(RegType type, int idx, int value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        _regData.regT[idx].preset = value;
    } else {
        _regData.regC[idx].preset = value;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getTimerCounter(RegType type, int idx, RegTimerCounter* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0 || value->current < 0 || value->preset < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx];
    } else {
        *value = _regData.regC[idx];
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getRegValue8(RegType type, int idx, unsigned char* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_A:
            if (idx >= REG_A_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regA[idx];
            break;
        case REG_R:
            if (idx >= REG_R_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regR[idx];
            break;
        case REG_X:
            if (idx >= REG_X_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regX[idx];
            break;
        case REG_Y:
            if (idx >= REG_Y_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regY[idx];
            break;
        case REG_F:
            if (idx >= REG_F_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regF[idx];
            break;
        case REG_G:
            if (idx >= REG_G_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regG[idx];
            break;
        case REG_I:
            if (idx >= REG_I_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regI[idx];
            break;
        case REG_Q:
            if (idx >= REG_Q_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regQ[idx];
            break;
        case REG_K:
            if (idx >= REG_K_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regK[idx];
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getRegValue16(RegType type, int idx, unsigned short* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    switch (type) {
        case REG_W:
            if (idx >= REG_W_NUM) {
                return REG_MANAGER_IDX_ERROR;
            }
            *value = _regData.regW[idx];
            break;
        default:
            return REG_MANAGER_TYPE_ERROR;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getRegValue32(RegType type, int idx, unsigned int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if (idx < 0) {
        return REG_MANAGER_IDX_ERROR;
    }
    // SemaphoreLocker locker(_sem);
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
    return REG_MANAGER_SUCCESS;
}

int RegManager::getTimerCounterCurrent(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx].current;
    } else {
        *value = _regData.regC[idx].current;
    }
    return REG_MANAGER_SUCCESS;
}

int RegManager::getTimerCounterPreset(RegType type, int idx, int* value) {
    if (!_isShmInit) {
        return REG_MANAGER_SHM_INIT_FAILED;
    }
    if ((type != REG_T && type != REG_C) || idx < 0) {
        return REG_MANAGER_TYPE_ERROR;
    }
    // SemaphoreLocker locker(_sem);
    if (type == REG_T) {
        *value = _regData.regT[idx].preset;
    } else {
        *value = _regData.regC[idx].preset;
    }
    return REG_MANAGER_SUCCESS;
}
