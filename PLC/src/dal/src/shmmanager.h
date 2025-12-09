#pragma once

#include <QSharedMemory>
#include "dalglobal.h"
#include "ncdata.h"
#include "regmanager.h"
#include "ladmanager.h"

class ShmManager {
public:
    ShmManager(const ShmManager&) = delete;
    ShmManager(ShmManager&&) = delete;
    ShmManager& operator=(const ShmManager&) = delete;
    ShmManager& operator=(ShmManager&&) = delete;
    static ShmManager& instance() {
        static ShmManager _instance;
        return _instance;
    }

    bool init();
    RegManager* regManager() { return _regManager; }
    LadManager* ladManager() { return _ladManager; }
    
private:
    ShmManager();
    ~ShmManager();
    void setIsInit(bool isInit);
    bool detach();
    const char* _shmName = "NcData";
    bool _isCreator = false;
    bool _isInit = false;
    NcData* _ncData = nullptr;
    QSharedMemory _shm;
    RegManager* _regManager = nullptr;
    LadManager* _ladManager = nullptr;
};
