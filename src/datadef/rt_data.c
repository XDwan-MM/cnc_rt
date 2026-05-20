#include "rt_data.h"
#include "global.h"

extern RT_DATA* rt_data;

GNC_RES rt_data_init(void)
{
    rt_data = (RT_DATA*)calloc(sizeof(RT_DATA), 1);
    if(rt_data == NULL) return GNC_DATA_NULL;
    rt_data->target_mode = MODE_EDIT;
    rt_data->current_mode = MODE_EDIT;
    rt_data->nc_event = SYS_EVENT_NONE;
    rt_data->parser_error.parser_error_no = -1;
    rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
    rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
    pthread_mutex_init(&rt_data->mutex, NULL);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES rt_data_reset(void){
    if(rt_data == NULL) return GNC_DATA_NULL;
    rt_data->nc_event = SYS_EVENT_NONE;
    pthread_mutex_lock(&rt_data->mutex);
    // memset(rt_data->sys_state, 0, sizeof(rt_data->sys_state));
    rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
    rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
    rt_data->sys_state[SYS_IS_ESTOP_ON] = false;
    rt_data->parser_error.parser_error_no = -1;
    pthread_mutex_unlock(&rt_data->mutex);
    return GNC_OPERATOR_SUCCESS;
}

void rt_set_parser_state(int error_no, int error_line){
    pthread_mutex_lock(&rt_data->mutex);
    rt_data->parser_error.parser_error_no = error_no;
    rt_data->parser_error.parser_error_line = error_line;
    pthread_mutex_unlock(&rt_data->mutex);
}

int rt_get_parser_error_no(){
    pthread_mutex_lock(&rt_data->mutex);
    int value = rt_data->parser_error.parser_error_no;
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
}

int rt_get_parser_error_line(){
    pthread_mutex_lock(&rt_data->mutex);
    int value = rt_data->parser_error.parser_error_line;
    pthread_mutex_unlock(&rt_data->mutex);
    return value;
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

void nc_set_resetting(){
    setVarChanInt(0, VAR_CHAN_IS_RESETTING, 1, 0);
    rt_data->nc_reset.is_resetting = 1;
}

void nc_set_resetted(){
    switch (rt_data->current_mode) {
        case MODE_MDI: // MDI
        case MODE_MEM: // 记忆
        case MODE_DNC:
            if(rt_data->nc_reset.is_resetting){
                if(cycle_tp->mem_exec_state == EXEC_IDLE && mcode_data->mcode_state == MCODE_EXECUTE_IDLE){
                    setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
                    rt_data->nc_reset.is_resetted = 1;
                }
            }
            break;
        case MODE_HOMING: // 回零
            if(rt_data->nc_reset.is_resetting && !var_chan[0][VAR_CHAN_IS_HOMING].value.i){
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
                rt_data->nc_reset.is_resetted = 1;
            }
            break;
        case MODE_JOG: // 点动
        case MODE_MPG: // 手轮
        case MODE_IDLE: // 空闲状态
        case MODE_EDIT: // 编辑
        default: // error
            if(rt_data->nc_reset.is_resetting){
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
                rt_data->nc_reset.is_resetted = 1;
            }
            break;
    }
}