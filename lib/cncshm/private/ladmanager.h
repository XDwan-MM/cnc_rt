#pragma once
#include "laddata.h"
#include "qglobal.h"

class LadManager {
public:
    LadManager(LadData& ladData);
    ~LadManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) {
        _isShmInit = isShmInit;
    }

    LadData* getLadDataPtr() {
        Q_ASSERT(_isShmInit);
        return &_ladData;
    }

private:
    LadData& _ladData;
    bool _isShmInit = false;
};
