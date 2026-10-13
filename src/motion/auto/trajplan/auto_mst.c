#include "auto_mst.h"
#include "plan_queue.h"
#include "global.h"
#include <varapi.h>
#include "spl_control.h"
#include "regapi.h"
#include <hmisysapi.h>  
#include <alarmapi.h>
#include <stdlib.h>
#include <string.h>

static void auto_deal_mstb();
static void auto_check_mstb_lock_on();
static void auto_deal_sub_program();
static void auto_clear_mstb_signal();
static void auto_mstb_fin_alert();
static void auto_mstb_fin_reset();
static void auto_set_mcode_signal(int index);
static void auto_set_scode_signal(int index);
static void auto_set_tcode_signal(int index);
static void auto_set_bcode_signal(int index);
static bool auto_m29_handshake_complete(bool selected_before_cycle);

void auto_get_mstb_cmd(){
    cycle_tp->exec_cmd = DO_MSTB_CMD;
    cycle_tp->target_stage = BLOCK_STOP_2_END;
    cycle_tp_mstb_cmd_data_reset();
    cycle_tp->mstb_cmd_data.mcode_size = cycle_tp->plan_block->mst_data.size;
    cycle_tp->mstb_cmd_data.sub_program = cycle_tp->plan_block->mst_data.sub_program;
    auto_check_mstb_lock_on();
    auto_deal_mstb();
}  

void auto_update_mstb_cmd(){
    switch (cycle_tp->exec_state) {
    case EXEC_RESET:
        cycle_tp->target_stage = PROGRAM_END;
        cycle_tp->exec_cmd = DO_NULL_CMD;
        cycle_tp_mstb_cmd_data_reset();
        break;
    case EXEC_PAUSING:
        break;
    default:
        auto_get_mstb_cmd();
        break;
    }
}

bool auto_mstb_cmd_is_finished(){
    return cycle_tp->mstb_cmd_data.finished;
}

void auto_execute_mstb_cmd(){
    if (cycle_tp->mstb_cmd_data.finished)
        return;
    bool selected_before_cycle = false;
    getRegBit(REG_F, 7, 0, &selected_before_cycle);
    // MF、SF、TF选通信号
    if(cycle_tp->mstb_cmd_data.signal_den){
        // 脉冲选通信号延时时间，缺省值2ms
        if(cycle_tp->mstb_cmd_data.tmf_count >= par_chan[0][PAR_CH_TMF_DELAY].value.val_int){
            if(! cycle_tp->mstb_cmd_data.signal_mf){
                switch (cycle_tp->plan_block->mst_data.m_code)
                {
                case M06:
                    // TF刀具选通信号
                    // 不输出MF：1、M99，M98，M198；2、调用子程序的M代码；3、调用用户宏程序的M代码
                    setRegBit(REG_F, 7, 3);
                    cycle_tp->mstb_cmd_data.signal_tf = true;
                    break;
                default:
                    // MF辅助功能选通信号
                    setRegBit(REG_F, 7, 0);
                    break;
                }
                cycle_tp->mstb_cmd_data.signal_mf = true;
                LOG(LOG_LEVEL_DEBUG, "2、延时%dms后输出M/S/TF信号", par_chan[0][PAR_CH_TMF_DELAY].value.val_int);
                auto_mstb_fin_alert();
            }
        }
        else cycle_tp->mstb_cmd_data.tmf_count ++;
    }
    // FIN信号 G4.3
    getRegBit(REG_G, 4, 3, &cycle_tp->mstb_cmd_data.signal_fin);
    if (cycle_tp->plan_block->mst_data.m_code == M29)
    {
        if (auto_m29_handshake_complete(selected_before_cycle))
        {
            spindle_complete_m29();
            auto_mstb_fin_reset();
            auto_clear_mstb_signal();
            cycle_tp->mstb_cmd_data.finished = true;
            LOG(LOG_LEVEL_INFO, "M29 PLC FIN及NC CSP均完成，推进程序");
        }
        return;
    }
    // FIN为1时，nc侧将MF选通信号置为0
    if(cycle_tp->mstb_cmd_data.signal_fin){
        // 完成信号宽幅时间，缺省值2ms
        if(cycle_tp->mstb_cmd_data.tfin_count >= par_chan[0][PAR_CH_TFIN_DELEY].value.val_int){
            cycle_tp->mstb_cmd_data.finished = true;
            LOG(LOG_LEVEL_DEBUG, "4、接收到FIN信号, 执行下一指令"); 
            auto_mstb_fin_reset();
            auto_clear_mstb_signal();
            if(cycle_tp->plan_block->mst_data.m_code == M02 || cycle_tp->plan_block->mst_data.m_code == M30) {
                // 程序运行行光标回到首行
                setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
                // M02和M30结尾的进行加工计数
                setVarChanInt(0, VAR_CHAN_PART_CNTR, var_chan[0][VAR_CHAN_PART_CNTR].value.i + 1, 0);
                LOG(LOG_LEVEL_INFO, "加工计数：%d", var_chan[0][VAR_CHAN_PART_CNTR].value.i);
            }
        }
        else cycle_tp->mstb_cmd_data.tfin_count ++;
    } else cycle_tp->mstb_cmd_data.tfin_count = 0;
}

void cycle_tp_mstb_cmd_data_reset(){
    cycle_tp->mstb_cmd_data.mcode_size = 0;
    cycle_tp->mstb_cmd_data.signal_mf = false;
    cycle_tp->mstb_cmd_data.signal_sf = false;
    cycle_tp->mstb_cmd_data.signal_tf = false;
    cycle_tp->mstb_cmd_data.signal_den = false;
    cycle_tp->mstb_cmd_data.pmc_end = true;
    cycle_tp->mstb_cmd_data.tfin_count = 0;
    cycle_tp->mstb_cmd_data.tmf_count = 0;
    cycle_tp->mstb_cmd_data.finished = false;
    cycle_tp->mstb_cmd_data.signal_fin = false;
    cycle_tp->mstb_cmd_data.m29_fin_low_seen = false;
    cycle_tp->mstb_cmd_data.m29_fin_latched = false;
    auto_mstb_fin_reset();
    auto_clear_mstb_signal();
}

/* PLC 的 FIN 与 NC 的动作完成可按任意顺序到达。锁存只属于当前辅助指令。 */
static bool auto_m29_handshake_complete(bool selected_before_cycle)
{
    MSTB_CMD_DATA *cmd = &cycle_tp->mstb_cmd_data;
    if (!spl_context.m29_active || spl_context.m29_phase == SPINDLE_M29_ERROR)
    {
        cmd->m29_fin_latched = false;
        cmd->m29_fin_low_seen = false;
        cmd->tfin_count = 0;
        return false;
    }
    if (!cmd->m29_fin_latched)
    {
        if (!cmd->signal_fin)
        {
            cmd->m29_fin_low_seen = true;
            cmd->tfin_count = 0;
        }
        else if (!selected_before_cycle || !cmd->signal_mf)
        {
            /* 未选通时的高电平不能成为当前指令的确认。 */
            cmd->m29_fin_low_seen = false;
            cmd->tfin_count = 0;
        }
        else if (cmd->m29_fin_low_seen)
        {
            if (cmd->tfin_count >= par_chan[0][PAR_CH_TFIN_DELEY].value.val_int)
            {
                cmd->m29_fin_latched = true;
                LOG(LOG_LEVEL_INFO,
                    "M29锁存PLC FIN：F7.0=%d G4.3=%d width_cycles=%d phase=%d ready=%d",
                    selected_before_cycle, cmd->signal_fin, cmd->tfin_count,
                    spl_context.m29_phase, spindle_m29_is_ready());
            }
            else
                ++cmd->tfin_count;
        }
    }
    return cmd->m29_fin_latched && spindle_m29_is_ready();
}

// 1、输出M代码信号，最多3个
static void auto_deal_mstb(){
    for(int i = 0; i < cycle_tp->mstb_cmd_data.mcode_size; i++){
        // nc侧输出代码信号M00~M31
        switch (cycle_tp->plan_block->mst_data.m_code)
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
            cycle_tp->target_stage = PROGRAM_END; 
            break;
        case M30: // 复位
            setRegValue8(REG_F, 10, 0x1e); 
            setRegBit(REG_F, 9, 4);  
            cycle_tp->target_stage = PROGRAM_END;
            break;
        case M03: // 主轴正转
            setRegValue8(REG_F, 10, 0x03); 
            spl_context.command_speed = cycle_tp->plan_block->mst_data.spindle.speed;
            spl_context.mcp_target_speed = cycle_tp->plan_block->mst_data.spindle.speed;
            spindle_clear_velocity_arrival();
            spl_context.m19_active = false;
            spl_context.orient_hold_stop = false;
            spl_context.command_state = SPINDLE_CW;
            spl_context.start_request_pending = true;
            spl_context.deferred_start_pending = false;
            auto_set_scode_signal(i);
            break;
        case M04: // 主轴反转
            setRegValue8(REG_F, 10, 0x04); 
            spl_context.command_speed = cycle_tp->plan_block->mst_data.spindle.speed;
            spl_context.mcp_target_speed = cycle_tp->plan_block->mst_data.spindle.speed;
            spindle_clear_velocity_arrival();
            spl_context.m19_active = false;
            spl_context.orient_hold_stop = false;
            spl_context.command_state = SPINDLE_CCW;
            spl_context.start_request_pending = true;
            spl_context.deferred_start_pending = false;
            auto_set_scode_signal(i);
            break;
        case M05: // 主轴停止
            setRegValue8(REG_F, 10, 0x05);
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
            spindle_clear_velocity_arrival();
            spl_context.m19_active = false;
            spl_context.orient_hold_stop = false;
            spl_context.start_request_pending = false;
            spl_context.deferred_start_pending = false;
            auto_set_scode_signal(i);
            break;
        case M06: // 不输出M代码信号：1、M99，M98，M198；2、调用子程序的M代码；3、调用用户宏程序的M代码
            spl_context.mcode_feedback = false;
            auto_set_tcode_signal(i);
            break;    
        case M19: // 主轴定向
            setRegValue8(REG_F, 10, 0x13); 
            spindle_clear_velocity_arrival();
            spl_context.m19_active = true;
            spl_context.orient_hold_stop = true;
            spl_context.command_speed = 0.0;
            spl_context.command_state = SPINDLE_STOP;
            spl_context.start_request_pending = false;
            spl_context.deferred_start_pending = false;
            /* G70.6 上升沿由 PLC 扫描器统一转换为 SPINDLE_EVENT_ORIENT。 */
            break;
        case M29: // 将主轴定向，并且到CSP模式上使能
            setRegValue8(REG_F, 10, 0x1d);
            spindle_request_m29();
            break;
        default:
            // 判断M代码大小
            if(cycle_tp->plan_block->mst_data.m_code > 300){
                // rt_set_parser_state(3002, cycle_tp->plan_block->line_number);
                setAlarm(ALARM_PS, ALARM_ERROR, 3002);
                rt_set_chan_state(0, CHAN_IS_PARSER_RESET, true);
                cycle_tp->mstb_cmd_data.finished = true;
                return;
            }
            auto_set_mcode_signal(i);
            break;
        }
        // 调试：打印信息
        switch (cycle_tp->plan_block->mst_data.m_code)
        {
        case M03:
        case M04:
            LOG(LOG_LEVEL_DEBUG, "1、nc侧输出代码信号M%dS%d", cycle_tp->plan_block->mst_data.m_code, cycle_tp->plan_block->mst_data.spindle.speed);
            break;
        case M06:
            LOG(LOG_LEVEL_DEBUG, "1、nc侧输出代码信号T%d", cycle_tp->plan_block->mst_data.tool_number);
            break;
        default:
            LOG(LOG_LEVEL_DEBUG, "1、nc侧输出代码信号M%d", cycle_tp->plan_block->mst_data.m_code);
            break;
        } 
        // 分配信号 DEN信号 F1.3
        setRegBit(REG_F, 1, 3);   
        cycle_tp->mstb_cmd_data.signal_den = true; 
    }
    cycle_tp->mstb_cmd_data.pmc_end = false;
    cycle_tp->mstb_cmd_data.signal_mf = false;
}

// 等待FIN信号的提示
static void auto_mstb_fin_alert(){
    var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i = 1;
    setAlarm(ALARM_CH, ALARM_WARNING, 0);
    // 调试：打印信息
    switch (cycle_tp->plan_block->mst_data.m_code)
    {
    case M03:
    case M04:
        LOG(LOG_LEVEL_DEBUG, "3、M%dS%d等待FIN信号中", cycle_tp->plan_block->mst_data.m_code, cycle_tp->plan_block->mst_data.spindle.speed);
        break;
    case M06:
        LOG(LOG_LEVEL_DEBUG, "3、T%d等待FIN信号中", cycle_tp->plan_block->mst_data.tool_number);
        break;
    default:
        LOG(LOG_LEVEL_DEBUG, "3、M%d等待FIN信号中", cycle_tp->plan_block->mst_data.m_code);
        break;
    } 
}

static void auto_mstb_fin_reset(){
    var_chan[0][VAR_CHAN_FIN_SIGNAL].value.i = 0;
    delAlarm(ALARM_CH, ALARM_WARNING, 0);
}

// 开启辅助功能锁住
static void auto_check_mstb_lock_on(){
    if(rt_data->sys_state[SYS_IS_MSBT_LOCK]) {
        switch (cycle_tp->plan_block->mst_data.m_code)
        {
        // 依旧可执行预定义M代码
        case M00:
        case M01:
        case M02:
        case M30:
            break;
        // 不执行自定义M代码
        default:
            cycle_tp->mstb_cmd_data.mcode_size = 0;
            cycle_tp->mstb_cmd_data.finished = true;
            break;
        }
    }
}

static void auto_deal_sub_program(){
    // 不输出MF：1、M99，M98，M198；2、调用子程序的M代码；3、调用用户宏程序的M代码
    if(cycle_tp->mstb_cmd_data.sub_program) {
        cycle_tp->mstb_cmd_data.mcode_size = 0;
        cycle_tp->mstb_cmd_data.finished = true;
    }
}

// 当FIN信号后，nc侧将代码信号置为0
static void auto_clear_mstb_signal(){
    // M代码信号
    setRegValue8(REG_F, 10, 0x00);     
    setRegValue8(REG_F, 11, 0x00);
    setRegValue8(REG_F, 12, 0x00);
    setRegValue8(REG_F, 13, 0x00);
    // S代码信号
    setRegValue8(REG_F, 22, 0x00);     
    setRegValue8(REG_F, 23, 0x00);
    setRegValue8(REG_F, 24, 0x00);
    setRegValue8(REG_F, 25, 0x00);
    // T代码信号
    setRegValue8(REG_F, 26, 0x00);     
    setRegValue8(REG_F, 27, 0x00);
    setRegValue8(REG_F, 28, 0x00);
    setRegValue8(REG_F, 29, 0x00);
    // 解码信号
    clearRegBit(REG_F, 9, 7);          
    clearRegBit(REG_F, 9, 6);
    clearRegBit(REG_F, 9, 5);
    clearRegBit(REG_F, 9, 4);
    // 分配结束信号
    clearRegBit(REG_F, 1, 3); 
    // 辅助功能选通信号         
    clearRegBit(REG_F, 7, 0);          
    clearRegBit(REG_F, 8, 4);
    clearRegBit(REG_F, 8, 5);
    // 主轴速度选通信号
    clearRegBit(REG_F, 7, 2);
    // 刀具功能选通信号
    clearRegBit(REG_F, 7, 3);
}

static void auto_set_mcode_signal(int index){
    if(cycle_tp->plan_block->mst_data.m_code < REG_8BITE){
        setRegValue8(REG_F, 10, cycle_tp->plan_block->mst_data.m_code);
        setRegValue8(REG_F, 11, 0);
        setRegValue8(REG_F, 12, 0);
        setRegValue8(REG_F, 13, 0);
    } 
    else if(cycle_tp->plan_block->mst_data.m_code < REG_16BITE){
        setRegValue8(REG_F, 10, cycle_tp->plan_block->mst_data.m_code & 0xFF);          // 低 8 位
        setRegValue8(REG_F, 11, (cycle_tp->plan_block->mst_data.m_code >> 8) & 0xFF);   // 高 8 位
        setRegValue8(REG_F, 12, 0);
        setRegValue8(REG_F, 13, 0);
    }
    else if(cycle_tp->plan_block->mst_data.m_code < REG_24BITE){
        setRegValue8(REG_F, 10, cycle_tp->plan_block->mst_data.m_code & 0xFF);
        setRegValue8(REG_F, 11, (cycle_tp->plan_block->mst_data.m_code >> 8) & 0xFF);
        setRegValue8(REG_F, 12, (cycle_tp->plan_block->mst_data.m_code >> 16) & 0xFF);
        setRegValue8(REG_F, 13, 0);
    }
    else{
        setRegValue8(REG_F, 10, cycle_tp->plan_block->mst_data.m_code & 0xFF);
        setRegValue8(REG_F, 11, (cycle_tp->plan_block->mst_data.m_code >> 8) & 0xFF);
        setRegValue8(REG_F, 12, (cycle_tp->plan_block->mst_data.m_code >> 16) & 0xFF);
        setRegValue8(REG_F, 13, (cycle_tp->plan_block->mst_data.m_code >> 24) & 0xFF);
    }
}

static void auto_set_scode_signal(int index){
    if(cycle_tp->plan_block->mst_data.spindle.speed < REG_8BITE){
        setRegValue8(REG_F, 22, cycle_tp->plan_block->mst_data.spindle.speed);
        setRegValue8(REG_F, 23, 0);
        setRegValue8(REG_F, 24, 0);
        setRegValue8(REG_F, 25, 0);
    } 
    else if(cycle_tp->plan_block->mst_data.spindle.speed < REG_16BITE){
        setRegValue8(REG_F, 22, cycle_tp->plan_block->mst_data.spindle.speed & 0xFF);          // 低 8 位
        setRegValue8(REG_F, 23, (cycle_tp->plan_block->mst_data.spindle.speed >> 8) & 0xFF);   // 高 8 位
        setRegValue8(REG_F, 24, 0);
        setRegValue8(REG_F, 25, 0);
    }
    else if(cycle_tp->plan_block->mst_data.spindle.speed < REG_24BITE){
        setRegValue8(REG_F, 22, cycle_tp->plan_block->mst_data.spindle.speed & 0xFF);
        setRegValue8(REG_F, 23, (cycle_tp->plan_block->mst_data.spindle.speed >> 8) & 0xFF);
        setRegValue8(REG_F, 24, (cycle_tp->plan_block->mst_data.spindle.speed >> 16) & 0xFF);
        setRegValue8(REG_F, 25, 0);
    }
    else{
        setRegValue8(REG_F, 22, cycle_tp->plan_block->mst_data.spindle.speed & 0xFF);
        setRegValue8(REG_F, 23, (cycle_tp->plan_block->mst_data.spindle.speed >> 8) & 0xFF);
        setRegValue8(REG_F, 24, (cycle_tp->plan_block->mst_data.spindle.speed >> 16) & 0xFF);
        setRegValue8(REG_F, 25, (cycle_tp->plan_block->mst_data.spindle.speed >> 24) & 0xFF);
    }
    // LOG(LOG_LEVEL_DEBUG, "主轴转速 %d", cycle_tp->plan_block->mst_data.spindle.speed);
}

static void auto_set_tcode_signal(int index){
    if(cycle_tp->plan_block->mst_data.tool_number < REG_8BITE){
        setRegValue8(REG_F, 26, cycle_tp->plan_block->mst_data.tool_number);
        setRegValue8(REG_F, 27, 0);
        setRegValue8(REG_F, 28, 0);
        setRegValue8(REG_F, 29, 0);
    } 
    else if(cycle_tp->plan_block->mst_data.tool_number < REG_16BITE){
        setRegValue8(REG_F, 26, cycle_tp->plan_block->mst_data.tool_number & 0xFF);          // 低 8 位
        setRegValue8(REG_F, 27, (cycle_tp->plan_block->mst_data.tool_number >> 8) & 0xFF);   // 高 8 位
        setRegValue8(REG_F, 28, 0);
        setRegValue8(REG_F, 29, 0);
    }
    else if(cycle_tp->plan_block->mst_data.tool_number < REG_24BITE){
        setRegValue8(REG_F, 26, cycle_tp->plan_block->mst_data.tool_number & 0xFF);
        setRegValue8(REG_F, 27, (cycle_tp->plan_block->mst_data.tool_number >> 8) & 0xFF);
        setRegValue8(REG_F, 28, (cycle_tp->plan_block->mst_data.tool_number >> 16) & 0xFF);
        setRegValue8(REG_F, 29, 0);
    }
    else{
        setRegValue8(REG_F, 26, cycle_tp->plan_block->mst_data.tool_number & 0xFF);
        setRegValue8(REG_F, 27, (cycle_tp->plan_block->mst_data.tool_number >> 8) & 0xFF);
        setRegValue8(REG_F, 28, (cycle_tp->plan_block->mst_data.tool_number >> 16) & 0xFF);
        setRegValue8(REG_F, 29, (cycle_tp->plan_block->mst_data.tool_number >> 24) & 0xFF);
    }
    // LOG(LOG_LEVEL_DEBUG, "刀具编号 %d", cycle_tp->plan_block->mst_data.tool_number);
}
