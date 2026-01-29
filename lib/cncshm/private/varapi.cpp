#include "varapi.h"
#include "varmanager.h"
#include "shmmanager.h"

static VarUnion VAR_INT(int value) {
    VarUnion var_union = {};
    var_union.type = INT_TYPE;
    var_union.value.i = value;
    return var_union;
}

static VarUnion VAR_DOUBLE(double value) {
    VarUnion var_union = {};
    var_union.type = DOUBLE_TYPE;
    var_union.value.f = value;
    return var_union;
}

static VarUnion VAR_UINT(unsigned int value) {
    VarUnion var_union = {};
    var_union.type = UINT_TYPE;
    var_union.value.n = value;
    return var_union;
}

static VarUnion VAR_CHAR(const char* value) {
    VarUnion var_union = {};
    var_union.type = CHAR_TYPE;
    for(int i = 0; i < 8 && value[i]; i++) {
        var_union.value.s[i] = value[i];
    }
    return var_union;
}

//static VarUnion VAR_PSTR(char* value) {
//    VarUnion var_union = {};
//    var_union.type = PSTR_TYPE;
//    var_union.value.pstr = value;
//    return var_union;
//}

int getVarValue(int type, int no, int index, void* value, int clientNo){
    return ShmManager::instance().varManager()->getVarData(type, no, index, 0, value, clientNo);
}

int setVarValue(int type, int no, int index, const void *value, int clientNo){
    return  ShmManager::instance().varManager()->setVarData(type, no, index, 0, value, clientNo);
}

int getVarChanValue(int no, int index, int sub_index, void* value, int clientNo){
    return ShmManager::instance().varManager()->getVarData(VAR_TYPE_CHANNEL, no, index, sub_index, value, clientNo);
}

int setVarChanValue(int no, int index, int sub_index, void* value, int clientNo){
    return  ShmManager::instance().varManager()->setVarData(VAR_TYPE_CHANNEL, no, index, sub_index, value, clientNo);
}

int getVarAxisValue(int no, int index, void* value, int clientNo){
    return ShmManager::instance().varManager()->getVarData(VAR_TYPE_AXIS, no, index, 0, value, clientNo);
}

int setVarAxisValue(int no, int index, void* value, int clientNo){
    return  ShmManager::instance().varManager()->setVarData(VAR_TYPE_AXIS, no, index, 0, value, clientNo);
}

int getVarSysValue(int index, void* value, int clientNo){
    return ShmManager::instance().varManager()->getVarData(VAR_TYPE_SYSTEM, 0, index, 0, value, clientNo);
}

int setVarSysValue(int index, void* value, int clientNo){
    return  ShmManager::instance().varManager()->setVarData(VAR_TYPE_SYSTEM, 0, index, 0, value, clientNo);
}

int getVarInt(int type, int no, int index, int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(type, no, index, 0, &var_union, clientNo);
    *value = var_union.value.i;
    return res;
}

int setVarInt(int type, int no, int index, int value, int clientNo){
    VarUnion var_union = VAR_INT(value);
    return ShmManager::instance().varManager()->setVarData(type, no, index, 0, &var_union, clientNo);
}

int getVarDouble(int type, int no, int index, double* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(type, no, index, 0, &var_union, clientNo);
    *value = var_union.value.f;
    return res;
}

int setVarDouble(int type, int no, int index, double value, int clientNo){
    VarUnion var_union = VAR_DOUBLE(value);
    return ShmManager::instance().varManager()->setVarData(type, no, index, 0, &var_union, clientNo);
}

int getVarUint(int type, int no, int index, unsigned int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(type, no, index, 0, &var_union, clientNo);
    *value = var_union.value.n;
    return res;
}

int setVarUint(int type, int no, int index, unsigned int value, int clientNo){
    VarUnion var_union = VAR_UINT(value);
    return ShmManager::instance().varManager()->setVarData(type, no, index, 0, &var_union, clientNo);
}

int getVarChar(int type, int no, int index, char* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(type, no, index, 0, &var_union, clientNo);
    for(int i = 0; i < 8; i++) value[i] = var_union.value.s[i];
    return res;
}

int setVarChar(int type, int no, int index, const char* value, int clientNo){
    VarUnion var_union = VAR_CHAR(value);
    return ShmManager::instance().varManager()->setVarData(type, no, index, 0, &var_union, clientNo);
}

int getVarAxisInt(int no, int index, int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
    *value = var_union.value.i;
    return res;
}

int setVarAxisInt(int no, int index, int value, int clientNo){
    VarUnion var_union = VAR_INT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
}

int getVarAxisDouble(int no, int index, double* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
    *value = var_union.value.f;
    return res;
}

int setVarAxisDouble(int no, int index, double value, int clientNo){
    VarUnion var_union = VAR_DOUBLE(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
}

int getVarAxisChar(int no, int index, char* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
    for(int i = 0; i < 8; i++) value[i] = var_union.value.s[i];
    return res;
}

int setVarAxisChar(int no, int index, const char* value, int clientNo){
    VarUnion var_union = VAR_CHAR(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
}

int getVarAxisUint(int no, int index, unsigned int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
    *value = var_union.value.n;
    return res;
}

int setVarAxisUint(int no, int index, unsigned int value, int clientNo){
    VarUnion var_union = VAR_UINT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_AXIS, no, index, 0, &var_union, clientNo);
}

int getVarChanInt(int no, int index, int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
    *value = var_union.value.i;
    return res;
}

int setVarChanInt(int no, int index, int value, int clientNo){
    VarUnion var_union = VAR_INT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
}

int getVarChanDouble(int no, int index, double* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
    *value = var_union.value.f;
    return res;
}

int setVarChanDouble(int no, int index, double value, int clientNo){
    VarUnion var_union = VAR_DOUBLE(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
}

int getVarChanChar(int no, int index, char* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
    for(int i = 0; i < 8; i++) value[i] = var_union.value.s[i];
    return res;
}

int setVarChanChar(int no, int index, const char* value, int clientNo){
    VarUnion var_union = VAR_CHAR(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
}

int getVarChanUint(int no, int index, unsigned int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
    *value = var_union.value.n;
    return res;
}

int setVarChanUint(int no, int index, unsigned int value, int clientNo){
    VarUnion var_union = VAR_UINT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_CHANNEL, no, index, 0, &var_union, clientNo);
}

int getVarSysInt(int index, int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
    *value = var_union.value.i;
    return res;
}

int setVarSysInt(int index, int value, int clientNo){
    VarUnion var_union = VAR_INT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
}

int getVarSysDouble(int index, double* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
    *value = var_union.value.f;
    return res;
}

int setVarSysDouble(int index, double value, int clientNo){
    VarUnion var_union = VAR_DOUBLE(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
}

int getVarSysChar(int index, char* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
    for(int i = 0; i < 8; i++) value[i] = var_union.value.s[i];
    return res;
}

int setVarSysChar(int index, const char* value, int clientNo){
    VarUnion var_union = VAR_CHAR(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
}

int getVarSysUint(int index, unsigned int* value, int clientNo){
    VarUnion var_union = {};
    int res = ShmManager::instance().varManager()->getVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
    *value = var_union.value.n;
    return res;
}

int setVarSysUint(int index, unsigned int value, int clientNo){
    VarUnion var_union = VAR_UINT(value);
    return ShmManager::instance().varManager()->setVarData(VAR_TYPE_SYSTEM, 0, index, 0, &var_union, clientNo);
}


// int VarSetBit(int type, int no, int index, int bit, int clientNo){
    
// }

// int VarClrBit(int type, int no, int index, int bit, int clientNo){
    
// }
