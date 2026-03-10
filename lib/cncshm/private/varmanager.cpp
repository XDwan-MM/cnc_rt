#include "varmanager.h"
#include <stdlib.h>
#include <stdio.h>

VarManager::VarManager(VarData& varData)
    : _varData(varData) {
}

VarManager::~VarManager() {
}

int VarManager::getVarData(int type, int no, int index, int sub_index, void* value, int clientNo)
{
    if (!_isShmInit) {
        return VAR_MANAGER_SHM_INIT_FAILED;
    }
    if (!value) {
        return VAR_MANAGER_POINT_NULL_ERR;
    }
    switch (type)
    {
    case VAR_TYPE_AXIS:
        if (no < 0 || no >= SYS_AXES_NUM){
            return VAR_MANAGER_AXISNO_ERR;
        }
        if (index < 0 || index >= VAR_AXIS_NUM)
        {
            return VAR_MANAGER_AXISINDEX_ERR;
        }
        *(VarUnion*)value = _varData.var_axis[no][index];
        break;
    case VAR_TYPE_CHANNEL:
        if (no < 0 || no >= SYS_CHAN_NUM){
            return VAR_MANAGER_CHANNO_ERR;
        }
        if (index < 0 || index >= VAR_CHAN_NUM)
        {
            return VAR_MANAGER_CHANINDEX_ERR;
        }
        if(index == VAR_CHAN_MODAL){
            if(sub_index < 0 || sub_index > MODAL_GROUP) return VAR_MANAGER_CHANINDEX_ERR;
            ((VarUnion*)value)->value.i = _varData.var_modal[sub_index].modeVal;
            return VAR_MANAGER_SUCCESS;
        }   
        else{
            *(VarUnion*)value = _varData.var_channel[no][index];
        }
        break;
    case VAR_TYPE_SYSTEM:
        if (index < 0 || index >= VAR_SYS_NUM)
        {
            return VAR_MANAGER_SYSINDEX_ERR;
        }
        *(VarUnion*)value = _varData.var_system[index];
        break;
    default:
        return VAR_MANAGER_TYPE_ERR;
    }
    return VAR_MANAGER_SUCCESS;
}

int VarManager::setVarData(int type, int no, int index, int sub_index, const void* value, int clientNo)
{
    if (!_isShmInit) {
        return VAR_MANAGER_SHM_INIT_FAILED;
    }
    // 检查value指针是否为空
    if (!value) {
        return VAR_MANAGER_POINT_NULL_ERR;
    }
    switch (type)
    {
    case VAR_TYPE_AXIS:
        if (no < 0 || no >= SYS_AXES_NUM){
            return VAR_MANAGER_AXISNO_ERR;
        }
        if (index < 0 || index >= VAR_AXIS_NUM)
        {
            return VAR_MANAGER_AXISINDEX_ERR;
        }
        _varData.var_axis[no][index] = *(VarUnion*)value;
        break;
    case VAR_TYPE_CHANNEL:
        if (no < 0 || no >= SYS_CHAN_NUM){
            return VAR_MANAGER_CHANNO_ERR;
        }
        if (index < 0 || index >= VAR_CHAN_NUM)
        {
            return VAR_MANAGER_CHANINDEX_ERR;
        }
        if(index == VAR_CHAN_MODAL){
            if(sub_index < 0 || sub_index > MODAL_GROUP) return VAR_MANAGER_CHANINDEX_ERR;
            _varData.var_modal[sub_index].modeVal = ((VarUnion*)value)->value.i;
        }else{
            _varData.var_channel[no][index] = *(VarUnion*)value;
        }
        break;
    case VAR_TYPE_SYSTEM:
        if (index < 0 || index >= VAR_SYS_NUM)
        {
            return VAR_MANAGER_SYSINDEX_ERR;
        }
        _varData.var_system[index] = *(VarUnion*)value;
        break;
    default:
        return VAR_MANAGER_TYPE_ERR;
    }
    return VAR_MANAGER_SUCCESS;
}
