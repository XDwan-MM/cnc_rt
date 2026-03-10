#include "rt_data.h"

extern RT_DATA* rt_data;

GNC_RES rt_data_init(void)
{
    rt_data = (RT_DATA*)calloc(sizeof(RT_DATA), 1);
    if(rt_data == NULL) return GNC_DATA_NULL;
    rt_data->target_mode = MODE_EDIT;
    rt_data->current_mode = MODE_EDIT;
    rt_data->nc_event = SYS_EVENT_NONE;
    pthread_mutex_init(&rt_data->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES rt_data_reset(void){
    if(rt_data == NULL) return GNC_DATA_NULL;
    rt_data->nc_event = SYS_EVENT_NONE;
    pthread_mutex_lock(&rt_data->mutex);
    memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
    for(int i = 0; i < SYS_CHAN_NUM; i++){
        memset(rt_data->chan_state[i], 0, sizeof(rt_data->chan_state[i]));
    }
    pthread_mutex_unlock(&rt_data->mutex);
    return GNC_OPERATOR_SUCCESS;
}

bool rt_get_sys_state(SysStateIndex state){
    if(state >= SYS_STATE_NUM || state < 0){
        return false;
    }
    pthread_mutex_lock(&rt_data->mutex);
    bool value = rt_data->sys_state[state];
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

void rt_set_sys_state(SysStateIndex state, bool value){
    if(state >= SYS_STATE_NUM || state < 0){
        return;
    }
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->sys_state[state] = value;
    pthread_mutex_unlock(&rt_data->mutex);
}

void rt_set_chan_state(int chan_index, ChanStateIndex state, bool value){
    if(chan_index < 0 || chan_index >= SYS_CHAN_NUM || state >= CHAN_STATE_NUM || state < 0){
        return;
    }
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->chan_state[chan_index][state] = value;
    pthread_mutex_unlock(&rt_data->mutex);
}

bool rt_get_chan_state(int chan_index, ChanStateIndex state){
    if(chan_index < 0 || chan_index >= SYS_CHAN_NUM || state >= CHAN_STATE_NUM || state < 0){
        return false;
    }
    pthread_mutex_lock(&rt_data->mutex);
    bool value = rt_data->chan_state[chan_index][state];
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}