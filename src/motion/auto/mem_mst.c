#include "mem_mst.h"
#include "mcode_queue.h"
#include "global.h"
#include <varapi.h>
#include "spl_control.h"
#include "regapi.h"
#include <hmisysapi.h>  
#include <alarmapi.h>
#include <stdlib.h>

static void nc_deal_mcode();
static void nc_output_mf();
static void nc_check_fin();
static void nc_clear_msignal();
static void mcode_data_mend();
static void mstb_lock_on();
static void mstb_fin_alert();
static void mstb_fin_reset();

void auto_execute_mcode(){
    switch (mcode_data->mcode_state)
    {
    case MCODE_EXECUTE_RUNNING:
        if(!mcode_queue_is_empty(mcode_queue)) nc_deal_mcode();
        if(mcode_data->mcode_size == 0) return;
        nc_output_mf();
        nc_check_fin();
        break;
    case MCODE_EXECUTE_PAUSING:
        if(mcode_data->mcode_size == 0) return;
        nc_output_mf();
        nc_check_fin();
        break;
    case MCODE_EXECUTE_RESET: 
        // 无M代码时按复位
        if(mcode_data->mcode_size == 0) {
            mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
            return;
        }
        // todo：HMI的复位和M30均直接重置M代码F变量状态
        if(mcode_data->mcode_block[0].m_code == M02 || mcode_data->mcode_block[0].m_code == M30) {
            LOG(LOG_LEVEL_DEBUG, "3、接收到FIN信号, 执行下一指令");
            // 程序运行行光标回到首行
            setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
            // M02和M30结尾的进行加工计数
            setVarChanInt(0, VAR_CHAN_PART_CNTR, var_chan[0][VAR_CHAN_PART_CNTR].value.i + 1, 0);
            LOG(LOG_LEVEL_INFO, "加工计数：%d", var_chan[0][VAR_CHAN_PART_CNTR].value.i);
            LOG(LOG_LEVEL_INFO, "程序运行结束");
        }
        mcode_handle_reset();
        // nc_output_mf();
        // nc_check_fin();
        // if(mcode_data->mcode_size == 0) mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
        break;
    case MCODE_EXECUTE_IDLE:
    default:
        break;
    }
}

// 1、输出M代码信号，最多3个
static void nc_deal_mcode(){
    if(mcode_queue_is_empty(mcode_queue)) return;
    mstb_lock_on();
    mcode_data->mcode_size = mcode_queue_get_size(mcode_queue);
    for(int i = 0; i < mcode_data->mcode_size; i++){
        mcode_queue_copy_block(mcode_queue, i, &mcode_data->mcode_block[i]);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, mcode_data->mcode_block[i].line_number, 0);
        LOG(LOG_LEVEL_INFO, "mline: %d", mcode_data->mcode_block[i].line_number);
        // nc侧输出代码信号M00~M31
        switch (mcode_data->mcode_block[i].m_code)
        {
        case M00: // 暂停
            setRegValue8(REG_F, 10, 0x00); 
            setRegBit(REG_F, 9, 7);
            break;
        case M01: // 选择停
            setRegValue8(REG_F, 10, 0x01); 
            setRegBit(REG_F, 9, 6); 
            break;
        case M02: // 复位
            setRegValue8(REG_F, 10, 0x02); 
            setRegBit(REG_F, 9, 5);  
            break;
        case M30: // 复位
            setRegValue8(REG_F, 10, 0x1e); 
            setRegBit(REG_F, 9, 4);  
            break;
        case M03: // 主轴正转
            setRegValue8(REG_F, 10, 0x03); 
            spl_data.mcp_target_speed = mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcode_feedback = false;
            break;
        case M04: // 主轴反转
            setRegValue8(REG_F, 10, 0x04); 
            spl_data.mcp_target_speed = mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcode_feedback = false;
            break;
        case M05: // 主轴停止
            setRegValue8(REG_F, 10, 0x05);
            spl_data.mcode_feedback = false;
            break;
        case M19: // 主轴定位
            setRegValue8(REG_F, 10, 0x13); 
            spl_data.spl_event = SPINDLE_EVENT_ORIENT;
            break;
        default:
            // 判断Ｍ代码大小
            if(mcode_data->mcode_block[i].m_code > 300){
                rt_set_parser_state(3002, mcode_data->mcode_block[i].line_number);
                rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
                mcode_queue_reset(mcode_queue);
                mcode_handle_reset();
                return;
            }
            if(mcode_data->mcode_block[i].m_code < REG_F10){
                setRegValue8(REG_F, 10, mcode_data->mcode_block[i].m_code);
                setRegValue8(REG_F, 11, 0);
                setRegValue8(REG_F, 12, 0);
                setRegValue8(REG_F, 13, 0);
            } 
            else if(mcode_data->mcode_block[i].m_code < REG_F11){
                setRegValue8(REG_F, 10, mcode_data->mcode_block[i].m_code & 0xFF);          // 低 8 位
                setRegValue8(REG_F, 11, (mcode_data->mcode_block[i].m_code >> 8) & 0xFF);   // 高 8 位
                setRegValue8(REG_F, 12, 0);
                setRegValue8(REG_F, 13, 0);
            }
            else if(mcode_data->mcode_block[i].m_code < REG_F12){
                setRegValue8(REG_F, 10, mcode_data->mcode_block[i].m_code & 0xFF);
                setRegValue8(REG_F, 11, (mcode_data->mcode_block[i].m_code >> 8) & 0xFF);
                setRegValue8(REG_F, 12, (mcode_data->mcode_block[i].m_code >> 16) & 0xFF);
                setRegValue8(REG_F, 13, 0);
            }
            else{
                setRegValue8(REG_F, 10, mcode_data->mcode_block[i].m_code & 0xFF);
                setRegValue8(REG_F, 11, (mcode_data->mcode_block[i].m_code >> 8) & 0xFF);
                setRegValue8(REG_F, 12, (mcode_data->mcode_block[i].m_code >> 16) & 0xFF);
                setRegValue8(REG_F, 13, (mcode_data->mcode_block[i].m_code >> 24) & 0xFF);
            }
            break;
        }
        LOG(LOG_LEVEL_DEBUG, "1、nc侧输出代码信号M%d", mcode_data->mcode_block[i].m_code);
        // 分配信号 DEN信号 F1.3
        setRegBit(REG_F, 1, 3);   
        mcode_data->signal_den = true; 
    }
    mcode_queue_reset(mcode_queue);
    mcode_data->pmc_end = false;
    mcode_data->signal_mf = false;
}

// 2、输出MF信号
static void nc_output_mf(){
    // 设定TMF后，将MF信号置为1
    switch (mcode_data->mcode_size)
    {
    case 1:
        // MF信号 F7.0
        if(mcode_data->signal_den){
            // 脉冲选通信号延时时间，缺省值2ms
            if(mcode_data->tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int){
                if(! mcode_data->signal_mf){
                    setRegBit(REG_F, 7, 0);
                    mcode_data->signal_mf = true;
                    LOG(LOG_LEVEL_DEBUG, "2、延时%dms后输出MF信号", par_chan[0][PAR_CH_TMF_DELAY].value.val_int);
                    mstb_fin_alert();
                }
            }
            else mcode_data->tmf_count ++;
        }
        break;
    case 2:
        if(mcode_data->signal_den){
            if(mcode_data->tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int){
                if(! mcode_data->signal_mf){
                    setRegBit(REG_F, 7, 0);
                    setRegBit(REG_F, 8, 4);
                    mcode_data->signal_mf = true;
                    LOG(LOG_LEVEL_DEBUG, "2、延时%dms后输出MF信号", par_chan[0][PAR_CH_TMF_DELAY].value.val_int);
                    mstb_fin_alert();
                }
            }
            else mcode_data->tmf_count ++;
        }
        break;
    case 3:
        if(mcode_data->signal_den){
            if(mcode_data->tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int){
                if(! mcode_data->signal_mf){
                    setRegBit(REG_F, 7, 0);
                    setRegBit(REG_F, 8, 4);
                    setRegBit(REG_F, 8, 5);
                    mcode_data->signal_mf = true;
                    LOG(LOG_LEVEL_DEBUG, "2、延时%dms后输出MF信号", par_chan[0][PAR_CH_TMF_DELAY].value.val_int);
                    mstb_fin_alert();
                }
            }
            else mcode_data->tmf_count ++;
        }
    default:
        break;
    }
}

// 3、接受FIN信号
static void nc_check_fin(){
    // ===========调试=========
    // nc_clear_msignal();
    // mcode_data_mend();
    // LOG(LOG_LEVEL_DEBUG, "3 接收到FIN信号");
    // return;
    // ===========调试=========
    // FIN信号 G4.3
    getRegBit(REG_G, 4, 3, &mcode_data->signal_fin);
    // FIN为1时，nc侧将MF选通信号置为0
    if(mcode_data->signal_fin){
        // 完成信号宽幅时间，缺省值2ms
        if(mcode_data->tfin_count >= par_chan[0][PAR_CH_TFIN_DELEY].value.val_int){
            if(mcode_data->mcode_block[0].m_code == M02 || mcode_data->mcode_block[0].m_code == M30) {
                // 程序运行行光标回到首行
                setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                // M02和M30结尾的进行加工计数
                setVarChanInt(0, VAR_CHAN_PART_CNTR, var_chan[0][VAR_CHAN_PART_CNTR].value.i + 1, 0);
                LOG(LOG_LEVEL_INFO, "加工计数：%d", var_chan[0][VAR_CHAN_PART_CNTR].value.i);
                LOG(LOG_LEVEL_INFO, "程序运行结束");
            }
            mstb_fin_reset();
            // FIN为1，MF为0后，nc侧将代码信号置为0
            nc_clear_msignal();
            mcode_data_mend();
            LOG(LOG_LEVEL_DEBUG, "3、接收到FIN信号, 执行下一指令");
            if(rt_get_sys_state(SYS_IS_SINGLE_BLOCK)) {
                mcode_data->mcode_state = MCODE_EXECUTE_PAUSING;
                cycle_tp->mem_exec_state = EXEC_PAUSING;
                LOG(LOG_LEVEL_DEBUG, "单段下停止在%d段", mcode_data->mcode_block->line_number);
            }  
        }
        else mcode_data->tfin_count ++;
    }
}

// 等待FIN信号的提示
static void mstb_fin_alert(){
    var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i = 1;
    setAlarm(ALARM_CH, ALARM_WARNING, 0);
    LOG(LOG_LEVEL_DEBUG, "M%d获取不到FIN信号,等待中...", mcode_data->mcode_block[0].m_code);
}

static void mstb_fin_reset(){
    var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i = 0;
    delAlarm(ALARM_CH, ALARM_WARNING, 0);
}

// 开启辅助功能锁住
static void mstb_lock_on(){
    if(rt_data->sys_state[SYS_IS_MSBT_LOCK]) {
        int mcode_size =  mcode_queue_get_size(mcode_queue);
        for(int i = 0; i < mcode_size; i++){
            switch (mcode_queue->block->m_code)
            {
            // 依旧可执行预定义M代码
            case M00:
            case M01:
            case M02:
            case M30:
                break;
            // 不执行自定义M代码
            default:
                mcode_queue_pop(mcode_queue);
                rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
                break;
            }
        } 
    }
}

// 当FIN信号后，nc侧将代码信号置为0
static void nc_clear_msignal(){
    setRegValue8(REG_F, 10, 0x00);     // 代码信号
    setRegValue8(REG_F, 11, 0x00);
    setRegValue8(REG_F, 12, 0x00);
    setRegValue8(REG_F, 13, 0x00);
    clearRegBit(REG_F, 9, 7);          // 解码信号
    clearRegBit(REG_F, 9, 6);
    clearRegBit(REG_F, 9, 5);
    clearRegBit(REG_F, 9, 4);
    clearRegBit(REG_F, 1, 3);          // 分配结束信号
    clearRegBit(REG_F, 7, 0);
    clearRegBit(REG_F, 8, 4);
    clearRegBit(REG_F, 8, 5);
}

GNC_RES mcode_data_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    mcode_data_delete();
    mcode_queue = (MCODE_QUEUE*)calloc(sizeof(MCODE_QUEUE), 1);
    if(mcode_queue == NULL) return GNC_DATA_NULL;
    res = mcode_queue_init(mcode_queue);
    if(res != GNC_OPERATOR_SUCCESS){
        mcode_data_delete();
        return res;
    }
    mcode_data = (MCODE_DATA*)calloc(sizeof(MCODE_DATA), 1);
    if(mcode_data == NULL){
        mcode_data_delete();
        return GNC_DATA_NULL;
    }
    mcode_data->mcode_size = 0;
    mcode_data->pmc_end = true;
    mcode_data->tfin_count = 0;
    return res;
}

void mcode_data_delete(){
    if(mcode_queue != NULL){
        mcode_queue_delete(mcode_queue);
        free(mcode_queue);
        mcode_queue = NULL;
    }
    free(mcode_data);
    mcode_data = NULL;
}

static void mcode_data_mend(){
    mcode_queue_reset(mcode_queue);
    mcode_data->mcode_size = 0;
    mcode_data->signal_mf = false;
    mcode_data->signal_den = false;
    mcode_data->pmc_end = true;
    mcode_data->tfin_count = 0;
    mcode_data->tmf_count = 0;
    rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
}

void mcode_handle_reset(){
    nc_clear_msignal();
    mcode_data_mend();
    mstb_fin_reset();
    mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
}

void mcode_handle_estop(){
    nc_clear_msignal();
    mcode_data_mend();
    mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
}
