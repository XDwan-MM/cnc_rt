#include "rerun.h"
#include "global.h"
#include <hmisysapi.h>
#include "alarmapi.h"

GNC_RES rerun_data_init(){
    rerun_data = (RERUN_DATA*)calloc(sizeof(RERUN_DATA), 1);
    if(rt_data == NULL) return GNC_DATA_NULL;
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->mem_rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    return GNC_OPERATOR_SUCCESS;
}

/// @brief 复位清除再启动断点信息
void rerun_data_reset(){
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->mem_rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
}

/// @brief 自动加工过程中记录当前行信息
void rerun_data_store(PLAN_BLOCK* break_block){
    switch (rt_data->current_mode)
    {
    case MODE_MEM:
        rerun_data->is_mem_rerun = 1;
        rerun_data->mem_rerun_row = break_block->line_number;
        memcpy(&rerun_data->break_block, break_block, sizeof(PLAN_BLOCK));
    default:
        break;
    } 
}

/// @brief 自动加工过程中记录断点位置
void rerun_store_breakpoint(){
    switch (rt_data->current_mode)
    {
    case MODE_MEM:
        rerun_data->breakpoint.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
        rerun_data->breakpoint.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
        rerun_data->breakpoint.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
    default:
        break;
    }
}

/// @brief 自动加工记录断点程序名称
void rerun_store_parser_file(){
    
}

/// @brief 切换模式
void rerun_update_when_swithing(){
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    switch (rt_data->target_mode)
    {
    case MODE_MDI:
        rerun_data->is_rerun = 0;
        rerun_data->rerun_row = 0;
        break;
    case MODE_MEM:
        rerun_data->is_rerun = rerun_data->is_mem_rerun;
        rerun_data->rerun_row = rerun_data->mem_rerun_row;
        rerun_data->is_first_time_click_rerun = true;
        break;
    default:
        break;
    }
}

/// @brief 判断是否程序再启动
bool check_is_program_rerun(){
    if (rerun_data->is_rerun){
        if (rerun_data->is_first_time_click_rerun) {
            LOG(LOG_LEVEL_WARN, "从程序中间开始，请确保刀具安全后按开始继续");
            setAlarm(ALARM_CH, ALARM_WARNING, 3);
            rerun_data->is_first_time_click_rerun = false;
            return true;
        }
    }
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    rerun_data->is_first_time_click_rerun = true;
    return false;
}

/// @brief 切换程序
void rerun_reset_is_rerun(){
    if(rerun_data->rerun_row > 0) rerun_data->is_rerun = 1;
    else rerun_data->is_rerun = 0;
    rerun_data->rerun_row = 0;
}

/// @brief 界面修改行信息
void rt_update_rerun_row(){
    getVarChanInt(0, VAR_CHAN_RUN_ROW, &rerun_data->rerun_row, 0);
}

/// @brief 构建回断点程序段
void create_rerun_block(PLAN_BLOCK* rerun_block){
    memcpy(rerun_block, &rerun_data->break_block, sizeof(PLAN_BLOCK));
    memcpy(&rerun_block->end_pos, &rerun_data->breakpoint, sizeof(POS));
    // 以空运行速度移动到断点位置
    rerun_block->mode = PMC_MODE_CHOP;
    rerun_block->type = RAPID;
    rerun_block->feed_vel = rt_data->chan_data[0].dryrun_spd;
    LOG(LOG_LEVEL_DEBUG, "再启动行号 %d, 位置 %f %f %f", rerun_block->line_number, rerun_block->end_pos.x, rerun_block->end_pos.y, rerun_block->end_pos.z);
}

/// @brief 检查断点是否可再启动
bool check_rerun_row_is_valid(){
    switch (rerun_data->break_block.mode)
    {
    // 在下列方式在的程序段不可以执行程序再启动
    case PMC_MODE_TAP:
    case PMC_MODE_TAP_RETURE:
        LOG(LOG_LEVEL_ERR, "攻丝不可以执行程序再启动");
        setAlarm(ALARM_CH, ALARM_ERROR, 4);
        return false;
    case PMC_MODE_CHOP:
    case PMC_MODE_EXACT_STOP:
    case PMC_MODE_MEASURE_TOOL:
    case PMC_MODE_ARRIVE_CHECK:
        switch (rerun_data->break_block.type)
        {
        case HELIX_CW:
        case HELIX_CCW:
        case RIGID_TAP:
            LOG(LOG_LEVEL_ERR, "攻丝不可以执行程序再启动");
            setAlarm(ALARM_CH, ALARM_ERROR, 4);
            return false;
        default:
            break;
        }
        break;
    case PMC_MODE_M_CMD:
        switch (rerun_data->break_block.mst_data.m_code)
        {
        case M06:
        case M19:
            LOG(LOG_LEVEL_ERR, "主轴定向/刀具管理不可以执行程序再启动");
            setAlarm(ALARM_CH, ALARM_ERROR, 4);
            return false;
        default:
            break;
        }
        break;
    default:
        break;
    }
    LOG(LOG_LEVEL_DEBUG, "在该程序段可以执行程序再启动");
    delAlarm(ALARM_CH, ALARM_ERROR, 4);
    return true;
}
