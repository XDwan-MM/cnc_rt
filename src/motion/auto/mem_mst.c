#include "mem_mst.h"
#include "mcode_queue.h"
#include "global.h"
#include <varapi.h>
#include "spl_control.h"
#include "regapi.h"
#include <hmisysapi.h>  

static void mcode_set_state();
static void check_pmc_mend();
static void check_nc_mend();
static void nc_deal_new_mcode();
static void nc_clear_msignal();
static void nc_output_mend();
static void mcode_data_mend();
static void mcode_handle_reset();

static void mcode_set_state(){
    switch(rt_data->nc_event){
        case SYS_EVENT_RESET:
            mcode_data->mcode_state = MCODE_EXECUTE_RESET;
            break;
        case SYS_EVENT_RERUN:
            mcode_data->mcode_state = MCODE_EXECUTE_RUNNING;
            break;
        case SYS_EVENT_PAUSE:
            mcode_data->mcode_state = MCODE_EXECUTE_PAUSING;
            break;
        default:
            break;
    }
}

void execute_mcode(){
    mcode_set_state();
    switch (mcode_data->mcode_state)
    {
    case MCODE_EXECUTE_RUNNING:
        if(!mcode_queue_is_empty(mcode_queue)) nc_deal_new_mcode();
        if(mcode_data->mcode_size == 0) return;
        check_pmc_mend();
        check_nc_mend();
        nc_output_mend();
        break;
    case MCODE_EXECUTE_PAUSING:
        if(mcode_data->mcode_size == 0) return;
        check_pmc_mend();
        check_nc_mend();
        nc_output_mend();
        break;
    case MCODE_EXECUTE_RESET:
        // check_pmc_mend();
        // check_nc_mend();
        // nc_output_mend();
        // if(mcode_data->mcode_size == 0) mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
        nc_clear_msignal();
        mcode_data_mend();
        break;
    case MCODE_EXECUTE_IDLE:
    default:
        break;
    }
}

// 处理新的mcode，最多3个
static void nc_deal_new_mcode(){
    if(mcode_queue_is_empty(mcode_queue)) return;
    mcode_data->mcode_size = mcode_queue_get_size(mcode_queue);
    for(int i = 0; i < mcode_data->mcode_size; i++){
        mcode_queue_copy_block(mcode_queue, i, &mcode_data->mcode_block[i]);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, mcode_data->mcode_block[i].line_number, 0);
        // nc侧输出代码信号M00~M31
        switch (mcode_data->mcode_block[i].m_code)
        {
        case M00: // 暂停
            setRegValue8(REG_F, 10, 0x00); 
            setRegBit(REG_F, 9, 7);
            rt_data->nc_event = SYS_EVENT_PAUSE;
            mcode_data->mcode_state = MCODE_EXECUTE_PAUSING;
            cycle_tp->mcode_feedback = false;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M00");
            break;
        case M01: // 选择停
            setRegValue8(REG_F, 10, 0x01); 
            setRegBit(REG_F, 9, 6); 
            rt_data->nc_event = SYS_EVENT_PAUSE;
            mcode_data->mcode_state = MCODE_EXECUTE_PAUSING;
            cycle_tp->mcode_feedback = false;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M01");
            break;
        case M02: // 复位
            setRegValue8(REG_F, 10, 0x02); 
            setRegBit(REG_F, 9, 5);  
            rt_data->nc_event = SYS_EVENT_RESET;
            mcode_data->mcode_state = MCODE_EXECUTE_RESET;
            cycle_tp->mcode_feedback = false;
            cycle_tp->mcode = M02;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M02");
            break;
        case M30: // 复位
            setRegValue8(REG_F, 10, 0x1e); 
            setRegBit(REG_F, 9, 4);  
            rt_data->nc_event = SYS_EVENT_RESET;
            mcode_data->mcode_state = MCODE_EXECUTE_RESET;
            cycle_tp->mcode_feedback = false;
            cycle_tp->mcode = M30;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M30");
            break;
        case M03: // 主轴正转
            setRegValue8(REG_F, 10, 0x03); 
            spl_data.spl_event = SPINDLE_EVENT_CW;
            spl_data.emot_speed = mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcp_target_speed = mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcode_feedback = false;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M03");
            break;
        case M04: // 主轴反转
            setRegValue8(REG_F, 10, 0x04); 
            spl_data.spl_event = SPINDLE_EVENT_CCW;
            spl_data.emot_speed = -1.0 * mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcp_target_speed = mcode_data->mcode_block[i].spindle.speed;
            spl_data.mcode_feedback = false;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M04");
            break;
        case M05: // 主轴停止
            setRegValue8(REG_F, 10, 0x05); 
            spl_data.spl_event = SPINDLE_EVENT_STOP;
            spl_data.emot_speed = 0;
            spl_data.mcode_feedback = false;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M05");
            break;
        case M19: // 主轴定位
            setRegValue8(REG_F, 10, 0x13); 
            spl_data.spl_event = SPINDLE_EVENT_ORIENT;
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M19");
            break;
        default:
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
            LOG(LOG_LEVEL_DEBUG, "1 nc侧输出代码信号M%d", mcode_data->mcode_block[i].m_code);
            break;
        }
        // 分配信号 DEN信号 F1.3
        setRegBit(REG_F, 1, 3);   
        mcode_data->signal_den = true; 
        mcode_data->mcode_block[i].mcode_end = false;
    }
    mcode_queue_reset(mcode_queue);
    mcode_data->pmc_end = false;
}

// 执行控制时序
static void check_pmc_mend(){
    // FIN信号 G4.3
    getRegBit(REG_G, 4, 3, &mcode_data->signal_fin);
    // 设定TMF后，将MF信号置为1
    switch (mcode_data->mcode_size)
    {
    case 1:
        // MF信号 F7.0
        if(mcode_data->signal_den){
            // 脉冲选通信号延时时间，缺省值2ms
            if(mcode_data->mcode_block[0].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int){
                setRegBit(REG_F, 7, 0);
                mcode_data->mcode_block[0].signal_mf = true;
                // LOG(LOG_LEVEL_DEBUG, "2 nc侧mf信号完成");
            }
            else mcode_data->mcode_block[0].tmf_count ++;
        }
        break;
    case 2:
        if(mcode_data->signal_den){
            if(mcode_data->mcode_block[0].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int) {
                setRegBit(REG_F, 7, 0);
                mcode_data->mcode_block[0].signal_mf = true;
            }
            else mcode_data->mcode_block[0].tmf_count ++;
            if(mcode_data->mcode_block[1].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int) {
                setRegBit(REG_F, 8, 4);
                mcode_data->mcode_block[1].signal_mf = true;
            }
            else mcode_data->mcode_block[1].tmf_count ++;
        }
        break;
    case 3:
        if(mcode_data->signal_den){
            if(mcode_data->mcode_block[0].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int) {
                setRegBit(REG_F, 7, 0);
                mcode_data->mcode_block[0].signal_mf = true;
            }
            else mcode_data->mcode_block[0].tmf_count ++;
            if(mcode_data->mcode_block[1].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int) {
                setRegBit(REG_F, 8, 4);
                mcode_data->mcode_block[1].signal_mf = true;
            }
            else mcode_data->mcode_block[1].tmf_count ++;
            if(mcode_data->mcode_block[2].tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int) {
                setRegBit(REG_F, 8, 5);
                mcode_data->mcode_block[2].signal_mf = true;
            }
            else mcode_data->mcode_block[2].tmf_count ++;
        }
    default:
        break;
    }
    // *******调试*******
    // if(mcode_data->mcode_block[0].m_code == M00 || mcode_data->mcode_block[0].m_code == M01 || mcode_data->mcode_block[0].m_code == M02 || mcode_data->mcode_block[0].m_code == M30){
    //     nc_clear_msignal();
    //     clearRegBit(REG_F, 7, 0);
    //     clearRegBit(REG_F, 8, 4);
    //     clearRegBit(REG_F, 8, 5);
    //     mcode_data->mcode_block[0].signal_mf = false;
    //     mcode_data->mcode_block[1].signal_mf = false;
    //     mcode_data->mcode_block[2].signal_mf = false;
    //     mcode_data->pmc_end = true;
    //     return;
    // }
    // *******调试*******
    // FIN为1时，nc侧将MF选通信号置为0
    if(mcode_data->signal_fin){
        // 完成信号宽幅时间，缺省值2ms
        if(mcode_data->tfin_count >= par_chan[0][PAR_CH_TFIN_DELEY].value.val_int){
            mcode_data->signal_fin_delay = true;
            clearRegBit(REG_F, 7, 0);
            clearRegBit(REG_F, 8, 4);
            clearRegBit(REG_F, 8, 5);
            mcode_data->mcode_block[0].signal_mf = false;
            mcode_data->mcode_block[1].signal_mf = false;
            mcode_data->mcode_block[2].signal_mf = false;
            LOG(LOG_LEVEL_DEBUG, "3 pmc侧fin信号完成");
        }
        else mcode_data->tfin_count ++;
    }
    // FIN为1，MF为0后，nc侧将代码信号置为0
    switch (mcode_data->mcode_size)
    {
    case 1:
        if(!mcode_data->mcode_block[0].signal_mf && mcode_data->signal_fin_delay) {
            LOG(LOG_LEVEL_DEBUG, "4 pmc侧m指令完成");
            nc_clear_msignal();
            mcode_data->pmc_end = true;
        }
        break;
    case 2:
        if(!mcode_data->mcode_block[0].signal_mf && !mcode_data->mcode_block[1].signal_mf && mcode_data->signal_fin_delay) {
            LOG(LOG_LEVEL_DEBUG, "4 pmc侧m指令完成");
            nc_clear_msignal();
            mcode_data->pmc_end = true;
            
        }
        break;
    case 3:
        if(!mcode_data->mcode_block[0].signal_mf && !mcode_data->mcode_block[1].signal_mf && !mcode_data->mcode_block[2].signal_mf && mcode_data->signal_fin_delay) {
            LOG(LOG_LEVEL_DEBUG, "4 pmc侧m指令完成");
            nc_clear_msignal();
            mcode_data->pmc_end = true;
        }
        break;
    default:
        break;
    }
}

// nc侧检查是否有m指令，如有m指令判断是否完成
static void check_nc_mend(){
    for(int i = 0; i < mcode_data->mcode_size; i++){
        switch (mcode_data->mcode_block[i].m_code)
        {
        case M00: // 暂停
        case M01: // 选择停
            if(cycle_tp->mcode_feedback) {
                mcode_data->mcode_block[i].mcode_end = true;
                // LOG(LOG_LEVEL_DEBUG, "5 nc侧m指令完成");
            }
            break;
        case M02: // 复位
        case M30: // 复位
            if(cycle_tp->mcode_feedback) {
                mcode_data->mcode_block[i].mcode_end = true;
                // LOG(LOG_LEVEL_DEBUG, "5 nc侧m指令完成");
            }
            break;
        case M03: // 主轴正转
        case M04: // 主轴反转
        case M05: // 主轴停止
        case M19: // 主轴定位
            if(spl_data.mcode_feedback) {
                mcode_data->mcode_block[i].mcode_end = true;
                // LOG(LOG_LEVEL_DEBUG, "5 nc侧m指令完成");
            }
            break;
        default:
            mcode_data->mcode_block[0].mcode_end = true;
            // LOG(LOG_LEVEL_DEBUG, "5 nc侧m指令完成");
            break;
        }
    }
}

// 当nc和pmc都完成，反馈mcode完成信号给译码器，进入下一程序段
static void nc_output_mend(){
    switch (mcode_data->mcode_size)
    {
    case 1:
        if(mcode_data->mcode_block[0].mcode_end && mcode_data->pmc_end) {
            mcode_data_mend();
            LOG(LOG_LEVEL_DEBUG, "6 nc和pmc都完成");
        }
        break;
    case 2:
        if(mcode_data->mcode_block[0].mcode_end && mcode_data->mcode_block[1].mcode_end && mcode_data->pmc_end) {
            mcode_data_mend();
        }
        break;
    case 3:
        if(mcode_data->mcode_block[0].mcode_end && mcode_data->mcode_block[1].mcode_end && mcode_data->mcode_block[2].mcode_end && mcode_data->pmc_end) {
            mcode_data_mend();
        }
        break;
    default:
        break;
    }
}

// 当FIN信号后，nc侧将代码信号置为0
static void nc_clear_msignal(){
    // LOG(LOG_LEVEL_DEBUG, "当FIN信号后, nc侧将代码信号置为0");
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
    mcode_data->signal_den = false;
    mcode_data->signal_fin_delay = false;
}

GNC_RES mcode_data_init(){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    mcode_queue = (MCODE_QUEUE*)malloc(sizeof(MCODE_QUEUE));
    res = mcode_queue_init(mcode_queue);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    mcode_data = (MCODE_DATA*)malloc(sizeof(MCODE_DATA));
    if(mcode_data == NULL) return GNC_DATA_NULL;
    mcode_data->mcode_size = 0;
    mcode_data->pmc_end = true;
    mcode_data->tfin_count = 0;
    return res;
}

static void mcode_data_mend(){
    mcode_queue_reset(mcode_queue);
    mcode_data->mcode_size = 0;
    mcode_data->pmc_end = true;
    mcode_data->tfin_count = 0;
    rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
}

static void mcode_handle_reset(){
    nc_clear_msignal();
    mcode_data_mend();
    mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
}

void mcode_handle_estop(){
    nc_clear_msignal();
    mcode_data_mend();
    mcode_data->mcode_state = MCODE_EXECUTE_IDLE;
}
