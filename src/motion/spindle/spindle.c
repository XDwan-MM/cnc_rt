#include "spindle.h"
#include "global.h"
#include "mcode_queue.h"
#include "regapi.h"
#include <hmisysapi.h>
#include <alarmapi.h>

int sp_sign;
bool sp_stop_sem = false;

static double emot_speed = 0;
static double speed = 0;
static double mcp_sp_speed = 0;
static bool stop_current_state = false;
static bool stop_last_state = false;
static int32_t current_vel = 0;
static int32_t vel = 0;

// 主轴正转，输入参数d_speed 主轴转速 unit 转/分钟
int32_t Spindle_Cw()
{
    //    double plc_max_speed = 10000;
    double sys_max_speed = par_axis[AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real;
    double para_acc = par_axis[AXIS_SP][PAR_AX_FEED_ACC].value.val_real * 10;
    double diff = para_acc * 0.001;
    double target_speed;


    //    target_speed = fmin(plc_max_speed, sys_max_speed);
    //    target_speed = fmin(target_speed, emot_speed);
    target_speed = fmin(sys_max_speed, emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100);

    if(speed + diff < target_speed)
    {
        speed = speed + diff;
    }
    else
    {
        rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
        speed = target_speed;
    }
    return (int32_t)speed;
}
int32_t Spindle_Ccw(void)
{
    //    double plc_max_speed = -10000;
    double sys_max_speed = -1.0 * par_axis[AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real;
    double para_acc = par_axis[AXIS_SP][PAR_AX_FEED_ACC].value.val_real * 10;
    double diff = para_acc * 0.001;
    double target_speed;

    //    target_speed = fmax(plc_max_speed, sys_max_speed);
    //    target_speed = fmax(target_speed, emot_speed);
    target_speed = fmax(sys_max_speed, emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100);

    if((speed - diff) > target_speed)
    {
        speed = speed - diff;
    }
    else
    {
        speed = target_speed;
        rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
    }
    return (int32_t)speed;
}
void Spindle_Stop()
{
    emot_speed = 0;
    Hal_Driver_Get_Actual_Vel(&spindle,&current_vel);
    if(current_vel < 0)
    {
        vel = Spindle_Cw();
        Hal_Driver_Set_Target_Vel(&spindle,vel);
    }
    else if(current_vel > 0)
    {
        vel = Spindle_Ccw();
        Hal_Driver_Set_Target_Vel(&spindle,vel);
    }
    if(emot_speed == vel )
    {
        stop_current_state = true;
    }
    if(stop_current_state && !stop_last_state)
    {
        sp_stop_sem = true;
        spindle.mode = DRV_MODE_DISABLED;
        clearRegBit(REG_F, 10, 3);
        clearRegBit(REG_F, 10, 4);
        setRegBit(REG_F, 10, 5);
    }
    stop_last_state = stop_current_state;
    setVarChanInt(0, VAR_CHAN_S_CODE, vel, 0);
}

void emcSpindleControl()
{
    if(sp_sign == 0)
    {
        if(!sp_stop_sem)
            Spindle_Stop();
        return;
    }
    if(!mcp_sp_speed)
    {
        int res = setAlarm(ALARM_AX, ALARM_ERROR, 123);
        if(res == -8) return ;
        LOG(LOG_LEVEL_ERR, "主轴速度值缺省");
        sp_sign = 0;
        // mcp_sp_speed = par_axis[3][PAR_AX_DEFAULT_S].value.val_real;
    }
    //control logic
    if(sp_sign == 2 && spindle.state == DRV_STATE_OPERATION_ENABLED)
    {
        spindle.mode = DRV_MODE_CSV;

        emot_speed = -1.0 * mcp_sp_speed ;
        vel = Spindle_Ccw() ;
        sp_stop_sem = false;
        stop_current_state = false;
        stop_last_state = false;
        Hal_Driver_Set_Target_Vel(&spindle,vel);
    }
    else if(sp_sign == 1 && spindle.state == DRV_STATE_OPERATION_ENABLED)
    {
        spindle.mode = DRV_MODE_CSV;

        emot_speed = 1.0 * mcp_sp_speed ;
        vel = Spindle_Cw() ;
        sp_stop_sem = false;
        stop_current_state = false;
        stop_last_state = false;
        Hal_Driver_Set_Target_Vel(&spindle,vel);
    }
    else if(sp_sign == 3)
    {
        printf("dx start \n");
    }
    setVarChanInt(0, VAR_CHAN_S_CODE, vel, 0);

}


void emcmcodedeal()
{
    if(!mcode_queue_is_empty(mcode_queue))
    {
        MCODE_BLOCK* mcode_block = mcode_queue_peak(mcode_queue);
        setVarChanInt(0, VAR_CHAN_RUN_ROW, mcode_block->line_number, 0);
        if (mcode_block == NULL) {
            printf("Error: mcode_queue_pop returned NULL!\n");
            return; // 或其他错误处理
        }
        if(mcode_block->m_code == M03)
        {
            clearRegBit(REG_F, 10, 4);
            clearRegBit(REG_F, 10, 5);
            setRegBit(REG_F, 10, 3);

            spindle.mode = DRV_MODE_CSV;
            sp_sign = 1;
            emot_speed = 1.0 * mcode_block->spindle.speed ;
            mcp_sp_speed = mcode_block->spindle.speed ;
        }
        else if(mcode_block->m_code == M04)
        {
            clearRegBit(REG_F, 10, 3);
            clearRegBit(REG_F, 10, 5);
            setRegBit(REG_F, 10, 4);

            spindle.mode = DRV_MODE_CSV;
            sp_sign = 2;
            emot_speed = -1.0 * mcode_block->spindle.speed ;
            mcp_sp_speed = mcode_block->spindle.speed ;
        }
        else if(mcode_block->m_code == M05 || mcode_block->m_code == M30)
        {
            clearRegBit(REG_F, 10, 3);
            clearRegBit(REG_F, 10, 4);
            setRegBit(REG_F, 10, 5);

            sp_sign = 0;
        }
        mcode_queue_pop(mcode_queue);
        // setRegBit(REG_F, 7, 0); // MF =1;
    }
    if(sp_sign == 0)
    {
        if(!sp_stop_sem)
            Spindle_Stop();
        return;
    }

    if(spindle.state == DRV_STATE_OPERATION_ENABLED)
    {
        if(sp_sign == 2)
        {
            vel = Spindle_Ccw() ;
            sp_stop_sem = false;
            stop_current_state = false;
            stop_last_state = false;
            Hal_Driver_Set_Target_Vel(&spindle,vel);
        }
        else if(sp_sign == 1)
        {
            vel = Spindle_Cw();
            sp_stop_sem = false;
            stop_current_state = false;
            stop_last_state = false;
            Hal_Driver_Set_Target_Vel(&spindle,vel);
        }
        setVarChanInt(0, VAR_CHAN_S_CODE, vel, 0);
    }
}
