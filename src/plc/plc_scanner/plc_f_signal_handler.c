#include "plc_f_signal_handler.h"
#include "alarmapi.h"
#include "global.h"
#include "regapi.h"
#include "regdata.h"
#include "spl_control.h"
#include <hmisysapi.h>
#include "alarmapi.h"
#include "reset.h"
#include "ref.h"

static void plc_output_alarm();
static void plc_output_mode();
static void plc_output_home();
static void plc_output_auto();
static void plc_output_reset();
static void plc_output_axis_status();
static void plc_output_spl_vel_status();
static void plc_output_spl_vel();

void plc_f_signal_handler_cycle()
{
    plc_output_alarm();
    plc_output_mode();
    plc_output_auto();
    plc_output_home();
    plc_output_reset();
    plc_output_axis_status();
    plc_output_spl_vel_status();
    plc_output_spl_vel();
}

static void plc_output_alarm()
{
    // F1.0 报警中：TH报警、TV报警、PS报警、超程报警、过热报警、伺服报警，不包括急停报警
    int ax_cnt, dv_cnt = 0;
    getAlarmCount(ALARM_AX, ALARM_ERROR, &ax_cnt);
    getAlarmCount(ALARM_SV, ALARM_ERROR, &dv_cnt);
    if (ax_cnt || dv_cnt || rt_get_sys_state(SYS_ESTOP_INPUT))
        setRegBit(REG_F, 1, 0);
    else
        clearRegBit(REG_F, 1, 0);
    // 准备就绪
    if (rt_get_sys_state(SYS_IS_READY))
        setRegBit(REG_F, 1, 7);
    else
        clearRegBit(REG_F, 1, 7);
}

// 自动运行启动中、运行中信号、休止中信号
static void plc_output_auto()
{
    // OP：运行中信号
    if (var_chan[0][VAR_CHAN_CYCLE].value.i || var_chan[0][VAR_CHAN_HOLD].value.i)
        setRegBit(REG_F, 0, 7);
    else
        clearRegBit(REG_F, 0, 7);
    // STL：启动中
    if (var_chan[0][VAR_CHAN_CYCLE].value.i)
        setRegBit(REG_F, 0, 5);
    else
        clearRegBit(REG_F, 0, 5);
    // SPL：休止中
    if (var_chan[0][VAR_CHAN_HOLD].value.i)
        setRegBit(REG_F, 0, 4);
    else
        clearRegBit(REG_F, 0, 4);
    // 切削进给中
    if(var_chan[0][VAR_CHAN_CYCLE].value.i && cycle_tp->plan_block->mode == PMC_MODE_CHOP){
        setRegBit(REG_F, 2, 6);
    }
    else
        clearRegBit(REG_F, 2, 6);
    // 可选程序段跳过
    if (rt_get_sys_state(SYS_IS_BLOCK_SKIP))
    {
        setRegBit(REG_F, 4, 0);
    }
    else
    {
        clearRegBit(REG_F, 4, 0);
    }
    // 辅助功能锁住
    if (rt_data->sys_state[SYS_IS_MSBT_LOCK])
        setRegBit(REG_F, 4, 4);
    else
        clearRegBit(REG_F, 4, 4);
    // 空运行
    if (rt_data->sys_state[SYS_IS_DRY_RUN])
        setRegBit(REG_F, 2, 7);
    else
        clearRegBit(REG_F, 2, 7);
    // 机械锁住
    if (rt_data->sys_state[SYS_IS_MAC_LOCK_READY])
        setRegBit(REG_F, 4, 1);
    else
        clearRegBit(REG_F, 4, 1);
    // 单程序段
    if (rt_data->sys_state[SYS_IS_SINGLE_BLOCK])
        setRegBit(REG_F, 4, 3);
    else
        clearRegBit(REG_F, 4, 3);
    // 程序再启动
    if(rt_data->sys_state[SYS_IS_PROG_RESTAR])
        setRegBit(REG_F, 2, 4);
    else
        clearRegBit(REG_F, 2, 4);
}

// 各轴返回零点完成、参考点建立
static void plc_output_home()
{
    update_ref_state();
    for (int i = 0; i < SYS_AXES_NUM; i++)
    {
        if(rt_data->axis_data[i].ref_status.ref_setup) setRegBit(REG_F, 120, i);
        else clearRegBit(REG_F, 120, i);
        if(rt_data->axis_data[i].ref_status.ref_on) setRegBit(REG_F, 94, i);
        else clearRegBit(REG_F, 94, i);
        if(rt_data->axis_data[i].ref_status.second_ref_on) setRegBit(REG_F, 96, i);
        else clearRegBit(REG_F, 96, i);
        if(rt_data->axis_data[i].ref_status.third_ref_on) setRegBit(REG_F, 98, i);
        else clearRegBit(REG_F, 98, i);
        if(rt_data->axis_data[i].ref_status.forth_ref_on) setRegBit(REG_F, 100, i);
        else clearRegBit(REG_F, 100, i);
    }
}

static void plc_output_reset()
{
    if (nc_reset->is_resetting)
    {
        switch (nc_reset->reset_trigger)
        {
        // 基于MDI的复位确认信号
        case HMI_RESET:
            if(nc_reset->is_mdi_reset) {
                setRegBit(REG_F, 6, 1);
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 1, 0);
            }
            else
            {
                clearRegBit(REG_F, 6, 1);
                setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
            }
            if(rt_data->sys_state[SYS_ESTOP_INPUT]){
                if(nc_reset->mdi_reset_tic < 24){
                    nc_reset->mdi_reset_tic++;
                }else{
                    nc_reset->is_mdi_reset = 0;
                } 
            }
            break;
        default:
            break;
        }
        // 复位中信号
        setRegBit(REG_F, 1, 1);
    }
    if (nc_reset->is_resetted)
    {
        nc_reset->is_resetting = 0;
        nc_reset->is_resetted = 0;
        nc_reset->is_reset_delaying = 1;
    }
    // 复位减速后延时
    if (nc_reset->is_reset_delaying)
    {
        if (nc_reset->reset_tic < par_mac[PAR_MAC_HARD_RESET_DELAY].value.val_int)
        {
            nc_reset->reset_tic++;
            // LOG(LOG_LEVEL_DEBUG, "复位减速后延时 %d", nc_reset->reset_tic);
        }
        else
        {
            clearRegBit(REG_F, 1, 1);
            clearRegBit(REG_F, 6, 1);
            setVarChanInt(0, VAR_CHAN_IS_RESETTING, 0, 0);
            nc_reset->is_reset_delaying = false;
            nc_reset->reset_tic = 0;
            LOG(LOG_LEVEL_DEBUG, "完成复位");
        }
    }
}

// 方式选择
static void plc_output_mode()
{
    if (rt_data->current_mode == MODE_IDLE)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
    }

    if (rt_data->current_mode == MODE_EDIT)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 6);
    }

    if (rt_data->current_mode == MODE_MEM)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 5);
    }

    // 自动运行选择检测 F3.5
    if (rt_data->current_mode == MODE_MDI)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 3);
    }

    if (rt_data->current_mode == MODE_DNC)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 4);
    }

    if (rt_data->current_mode == MODE_MPG)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        clearRegBit(REG_F, 4, 5);
        setRegBit(REG_F, 3, 1);
    }

    if (rt_data->current_mode == MODE_JOG)
    {
        clearRegBit(REG_F, 4, 5);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 6);
        setRegBit(REG_F, 3, 2);
    }

    if (rt_data->current_mode == MODE_HOMING)
    {
        clearRegBit(REG_F, 2, 1);
        clearRegBit(REG_F, 3, 1);
        clearRegBit(REG_F, 3, 2);
        clearRegBit(REG_F, 3, 3);
        clearRegBit(REG_F, 3, 4);
        clearRegBit(REG_F, 3, 5);
        clearRegBit(REG_F, 3, 6);
        setRegBit(REG_F, 4, 5);
    }
}

// 轴移动状态输出
static void plc_output_axis_status()
{
    for (int i = 0; i < 3; i++)
    {
        if (rt_data->axis_data[i].status.direction)
        {
            clearRegBit(REG_F, 106, i);
        }
        else
        {
            setRegBit(REG_F, 106, i);
        }
        if (rt_data->axis_data[i].status.is_moving)
        {
            setRegBit(REG_F, 102, i);
        }
        else
        {
            clearRegBit(REG_F, 102, i);
        }
    }
    for (int i = 0; i < 5; ++i)
    {
        if (rt_data->axis_data[i].status.is_disable)
        {
            setRegBit(REG_F, 110, i);
        }
        else
        {
            clearRegBit(REG_F, 110, i);
        }
    }
    if (rt_data->axis_data[0].status.is_error || rt_data->axis_data[1].status.is_error ||
        rt_data->axis_data[2].status.is_error || rt_data->axis_data[3].status.is_error)
    {
        clearRegBit(REG_F, 0, 6);
    }
    else
    {
        setRegBit(REG_F, 0, 6);
    }
    if (rt_data->axis_data[3].status.is_error)
    {
        setRegBit(REG_F, 45, 0);
    }
    else
    {
        clearRegBit(REG_F, 45, 0);
    }
}

static void plc_output_spl_vel_status()
{
    if (spl_data.spl_vel_state == SPINDLE_VEL_ARRIVED)
    {
        setRegBit(REG_F, 45, 3);
    }
    else
    {
        clearRegBit(REG_F, 45, 3);
    }
//    if(spl_data.spl_vel_state == SPINDLE_VEL_ZERO)
//    {
//        setRegBit(REG_F, 45, 1);
//    }
//    else
//    {
//        clearRegBit(REG_F, 45, 1);
//    }
    if(abs(spindle.actual_vel) < 5)
    {
        setRegBit(REG_F, 45, 1);
        spl_data.spl_vel_state = SPINDLE_VEL_ZERO;
    }
    else
    {
        clearRegBit(REG_F, 45, 1);
    }
}

static void plc_output_spl_vel()
{
    uint8_t bytes[4];
    uint32_t vel = (uint32_t)spl_data.mcp_target_speed;

    bytes[0] = (uint8_t)(vel & 0xff);
    bytes[1] = (uint8_t)((vel >> 8) & 0xff);
    bytes[2] = (uint8_t)((vel >> 16) & 0xff);
    bytes[3] = (uint8_t)((vel >> 24) & 0xff);

    setRegValue8(REG_F, 22, bytes[0]);
    setRegValue8(REG_F, 23, bytes[1]);
    setRegValue8(REG_F, 24, bytes[2]);
    setRegValue8(REG_F, 25, bytes[3]);
}
