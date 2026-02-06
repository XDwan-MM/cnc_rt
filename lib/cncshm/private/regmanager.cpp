#include "regmanager.h"

RegManager::RegManager(RegData& regData)
    : _regData(regData) {
}

RegManager::~RegManager() {
}

int RegManager::getRegNum(RegType type, int* num) {
    switch (type) {
        case REG_R:
            *num = REG_R_NUM;
            break;
        case REG_W:
            *num = REG_W_NUM;
            break;
        case REG_D:
            *num = REG_D_NUM;
            break;
        case REG_X:
            *num = REG_X_NUM;
            break;
        case REG_Y:
            *num = REG_Y_NUM;
            break;
        case REG_F:
            *num = REG_F_NUM;
            break;
        case REG_G:
            *num = REG_G_NUM;
            break;
        case REG_I:
            *num = REG_I_NUM;
            break;
        case REG_Q:
            *num = REG_Q_NUM;
            break;
        case REG_B:
            *num = REG_B_NUM;
            break;
        case REG_K:
            *num = REG_K_NUM;
            break;
        case REG_T:
            *num = REG_T_NUM;
            break;
        case REG_C:
            *num = REG_C_NUM;
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::setRegBit(RegType type, int idx, int bit) {
    switch (type) {
        case REG_A:
            _regData.regA[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_R:
            _regData.regR[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_W:
            _regData.regW[idx] |= static_cast<unsigned short>(1U << bit);
            break;
        case REG_D:
            _regData.regD[idx] |= static_cast<unsigned int>(1U << bit);
            break;
        case REG_X:
            _regData.regX[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_Y:
            _regData.regY[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_F:
            _regData.regF[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_G:
            _regData.regG[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_I:
            _regData.regI[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_Q:
            _regData.regQ[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        case REG_B:
            _regData.regB[idx] |= static_cast<unsigned int>(1U << bit);
            break;
        case REG_K:
            _regData.regK[idx] |= static_cast<unsigned char>(1U << bit);
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::clearRegBit(RegType type, int idx, int bit) {
    switch (type) {
        case REG_A:
            _regData.regA[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_R:
            _regData.regR[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_W:
            _regData.regW[idx] &= static_cast<unsigned short>(~(1U << bit));
            break;
        case REG_D:
            _regData.regD[idx] &= static_cast<unsigned int>(~(1U << bit));
            break;
        case REG_X:
            _regData.regX[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_Y:
            _regData.regY[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_F:
            _regData.regF[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_G:
            _regData.regG[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_I:
            _regData.regI[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_Q:
            _regData.regQ[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        case REG_B:
            _regData.regB[idx] &= static_cast<unsigned int>(~(1U << bit));
            break;
        case REG_K:
            _regData.regK[idx] &= static_cast<unsigned char>(~(1U << bit));
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::getRegBit(RegType type, int idx, int bit, bool* value) {
    switch (type) {
        case REG_A:
            *value = (_regData.regA[idx] >> bit) & 1U;
            break;
        case REG_R:
            *value = (_regData.regR[idx] >> bit) & 1U;
            break;
        case REG_W:
            *value = (_regData.regW[idx] >> bit) & 1U;
            break;
        case REG_D:
            *value = (_regData.regD[idx] >> bit) & 1U;
            break;
        case REG_X:
            *value = (_regData.regX[idx] >> bit) & 1U;
            break;
        case REG_Y:
            *value = (_regData.regY[idx] >> bit) & 1U;
            break;
        case REG_F:
            *value = (_regData.regF[idx] >> bit) & 1U;
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::setRegValue8(RegType type, int idx,  unsigned char value) {
    switch (type) {
        case REG_A:
            _regData.regA[idx] = value;
            break;
        case REG_R:
            _regData.regR[idx] = value;
            break;
        case REG_X:
            _regData.regX[idx] = value;
            break;
        case REG_Y:
            _regData.regY[idx] = value;
            break;
        case REG_F:
            _regData.regF[idx] = value;
            break;
        case REG_G:
            _regData.regG[idx] = value;
            break;
        case REG_I:
            _regData.regI[idx] = value;
            break;
        case REG_Q:
            _regData.regQ[idx] = value;
            break;
        case REG_K:
            _regData.regK[idx] = value;
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::setRegValue16(RegType type, int idx,  unsigned short value) {
    switch (type) {
        case REG_W:
            _regData.regW[idx] = value;
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::setRegValue32(RegType type, int idx,  unsigned int value) {
    switch (type) {
        case REG_D:
            _regData.regD[idx] = value;
            break;
        case REG_B:
            _regData.regB[idx] = value;
            break;
        case REG_T:
            _regData.regT[idx] = value;
            break;
        case REG_C:
            _regData.regC[idx] = value;
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::getRegValue8(RegType type, int idx, unsigned char* value) {
    switch (type) {
        case REG_A:
            *value = _regData.regA[idx];
            break;
        case REG_R:
            *value = _regData.regR[idx];
            break;
        case REG_X:
            *value = _regData.regX[idx];
            break;
        case REG_Y:
            *value = _regData.regY[idx];
            break;
        case REG_F:
            *value = _regData.regF[idx];
            break;
        case REG_G:
            *value = _regData.regG[idx];
            break;
        case REG_I:
            *value = _regData.regI[idx];
            break;
        case REG_Q:
            *value = _regData.regQ[idx];
            break;
        case REG_K:
            *value = _regData.regK[idx];
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::getRegValue16(RegType type, int idx, unsigned short* value) {
    switch (type) {
        case REG_W:
            *value = _regData.regW[idx];
            break;
        default:
            return -1;
    }
    return 0;
}

int RegManager::getRegValue32(RegType type, int idx, unsigned int* value) {
    switch (type) {
        case REG_D:
            *value = _regData.regD[idx];
            break;
        case REG_B:
            *value = _regData.regB[idx];
            break;
        case REG_T:
            *value = _regData.regT[idx];
            break;
        case REG_C:
            *value = _regData.regC[idx];
            break;
        default:
            return -1;
    }
    return 0;
}
