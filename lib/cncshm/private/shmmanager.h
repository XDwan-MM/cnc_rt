#pragma once

#include <QSharedMemory>
#include "ncdata.h"
#include "regmanager.h"
#include "ladmanager.h"
#include "hmisysmanager.h"
#include "varmanager.h"
#include "parmanager.h"
#include "alarmmanager.h"

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
    HmiSysManager* hmiSysManager() { return _hmiSysManager; }
    VarManager* varManager() { return _varManager; }
    ParManager* parManager() { return _parManager; }
    AlarmManager* alarmManager() { return _alarmManager; }

private:
    ShmManager();
    ~ShmManager();
    void setIsInit(bool isInit);
    bool isProcessAlive(int pid);
    QSharedMemory _shm;
    bool _isInit = false;
    QString _version = "1.1.2";
    NcData* _ncData = nullptr;
    RegManager* _regManager = nullptr;
    LadManager* _ladManager = nullptr;
    HmiSysManager *_hmiSysManager = nullptr;
    VarManager* _varManager = nullptr;
    ParManager* _parManager = nullptr;
    AlarmManager* _alarmManager = nullptr;
};
