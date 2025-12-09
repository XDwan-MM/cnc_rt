#pragma once
#include "laddata.h"

class LadManager {
public:
    LadManager(LadData& ladData);
    ~LadManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) {
        _isShmInit = isShmInit;
    }

    LadData* getLadDataPtr() { return _isShmInit ? &_ladData : nullptr; }

private:
    enum LadManagerError {
        LAD_MANAGER_SUCCESS = 0,
        LAD_MANAGER_SHM_INIT_FAILED = -1,
    };
    LadData& _ladData;
    bool _isShmInit = false;
};
