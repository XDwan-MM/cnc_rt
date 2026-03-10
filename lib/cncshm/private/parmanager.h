#pragma once
#include "pardata.h"
#include <QSystemSemaphore>

class ParManager {
public:
    ParManager(ParData& parData);
    ~ParManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) {
        _isShmInit = isShmInit;
    }
    
    ParData* getParDataPtr() {
        Q_ASSERT(_isShmInit);
        return &_parData;
    }

    int getParData(int file, int sub, int index, ParamUnit *data);
    int setParData(int file, int sub, int index, ParamUnit data);

    int getParProp(int file, int sub, int index, int propType, ParamUnit *val);
    int setParProp(int file, int sub, int index, int propType, ParamUnit val);

private:
    enum ParManagerError {
        PAR_MANAGER_SUCCESS = 0,
        PAR_MANAGER_ERR_SHM_INIT_FAILED = -1,
        PAR_MANAGER_ERR_NULL_POINTER = -2,
        PAR_MANAGER_ERR_INDEX = -3,
        PAR_MANAGER_ERR_TYPE = -4,
        PAR_MANAGER_ERR_SUB = -5,
    };
    ParData& _parData;
    bool _isShmInit = false;
};
