#ifndef HMISYSMANAGER_H
#define HMISYSMANAGER_H

#include "hmisysdata.h"

#include <QSystemSemaphore>

class HmiSysManager
{
public:
    HmiSysManager(HmiSysData &hmiSysData);
    void setIsShmInit(bool isShmInit);

    int getHmiSysData(HmiSysData *hmiSysData);

    int setNcFilePath(char *ncFilePath);
    int getNcFilePath(char *ncFilePath);

    int setMdiContent(char *mdiContent);
    int getMdiContent(char *mdiContent);

    int setAllPitchData(float *pitchData);
    int setPitchValue(int index, float val);
    int getPitchData(int start, int num, float *pitchData);

    int setCrdsValue(int type, int axis, double value);
    int getCrdsValue(int type, int axis, double *value);

private:
    enum HmiSysManagerError
    {
        HMISYS_MANAGER_SUCCESS = 0, // 读取数据成功
        HMISYS_MANAGER_SHM_INIT_FAILED = -1, // 共享内存未初始化
        HMISYS_MANAGER_POINT_ERROR = -2,    // 空指针错误

        HMISYS_CRDS_TYPE_ERROR = -3,  // 坐标类型错误
        HMISYS_CRDS_AXIS_ERROR = -4,  // 坐标轴错误

        HMISYS_PITCH_INDEX_ERROR = -5,  // 螺补索引错误
        HMISYS_PITCH_CAPACITY_ERROR = -6,   // 螺补容量错误
    };

    HmiSysData &m_hmiSysData;
    bool m_isShmInit;
    QSystemSemaphore m_sem;
};

#endif // HMISYSMANAGER_H
