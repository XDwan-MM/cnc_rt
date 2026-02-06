#include "regapi.h"
#include "shmmanager.h"

int getRegNum(RegType type, int* num) {
    return ShmManager::instance().regManager()->getRegNum(type, num);
}

int setRegBit(RegType type, int idx, int bit) {
    return ShmManager::instance().regManager()->setRegBit(type, idx, bit);
}

int clearRegBit(RegType type, int idx, int bit) {
    return ShmManager::instance().regManager()->clearRegBit(type, idx, bit);
}

int getRegBit(RegType type, int idx, int bit, bool* value) {
    return ShmManager::instance().regManager()->getRegBit(type, idx, bit, value);
}

int setRegValue8(RegType type, int idx, unsigned char value) {
    return ShmManager::instance().regManager()->setRegValue8(type, idx, value);
}

int setRegValue16(RegType type, int idx, unsigned short value) {
    return ShmManager::instance().regManager()->setRegValue16(type, idx, value);
}

int setRegValue32(RegType type, int idx, unsigned int value) {
    return ShmManager::instance().regManager()->setRegValue32(type, idx, value);
}

int getRegValue8(RegType type, int idx, unsigned char* value) {
    return ShmManager::instance().regManager()->getRegValue8(type, idx, value);
}

int getRegValue16(RegType type, int idx, unsigned short* value) {
    return ShmManager::instance().regManager()->getRegValue16(type, idx, value);
}

int getRegValue32(RegType type, int idx, unsigned int* value) {
    return ShmManager::instance().regManager()->getRegValue32(type, idx, value);
}
