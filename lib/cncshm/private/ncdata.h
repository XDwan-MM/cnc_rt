#pragma once

#include "regdata.h"
#include "laddata.h"
#include "hmisysdata.h"
#include "vardata.h"
#include "pardata.h"
#include "alarmdata.h"

typedef struct {
    int64_t pid1;
    int64_t pid2;
    RegData regData;
    LadData ladData;
    HmiSysData hmiSysData;
    VarData varData;
    ParData parData;
    AlarmDataBuffer alarmDataBuffer;
} NcData;
