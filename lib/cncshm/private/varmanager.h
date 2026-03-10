#pragma once
#include "vardata.h"
#include "qglobal.h"

class VarManager {
public:
    VarManager(VarData& varData);
    ~VarManager();

    // 设置共享内存初始化状态
    void setIsShmInit(bool isShmInit) {
        _isShmInit = isShmInit;
    }

    VarData* getVarDataPtr() {
        Q_ASSERT(_isShmInit);
        return &_varData;
    }

    int getVarData(int type, int no, int index, int sub_index, void* value, int clientNo);

    int setVarData(int type, int no, int index, int sub_index, const void* value, int clientNo);

private:
    enum VarManagerError {
        VAR_MANAGER_SUCCESS = 0,        // 成功
        VAR_MANAGER_TYPE_ERR, 	        // 函数与变量类型不匹配 
        VAR_MANAGER_AXISNO_ERR,			// 轴号超过限制
        VAR_MANAGER_CHANNO_ERR,			// 通道号超过限制
        VAR_MANAGER_AXISINDEX_ERR,		// 轴变量索引号超过限制
        VAR_MANAGER_CHANINDEX_ERR,		// 通道变量索引号超过限制
        VAR_MANAGER_SYSINDEX_ERR,		// 系统变量索引号超过限制
        VAR_MANAGER_BITNO_ERR,			// 变量位号索引号超过限制
        VAR_MANAGER_POINT_NULL_ERR,		// 变量值指针为空
        VAR_MANAGER_SHM_INIT_FAILED,    // 共享内存未初始化
    };
    VarData& _varData;
    bool _isShmInit = false;
};
