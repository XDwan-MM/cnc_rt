#include "spindle.h"
#include "global.h"
#include "mcode_queue.h"

double emot_speed;
double speed;
int sp_sign;
bool sp_stop_sem;
double emot_speed = 0;
double speed = 0;
bool sp_stop_sem = false;
bool stop_current_state = false;
bool stop_last_state = false;
int32_t current_vel = 0;
int32_t vel = 0;
bool speedArrived;


// 主轴正转，输入参数d_speed 主轴转速 unit 转/分钟
int32_t Spindle_Cw()
{
    //    double plc_max_speed = 10000;
    double sys_max_speed = par_axis[AXIS_SP].spdl_max_speed;
    double para_acc = par_axis[AXIS_SP].feed_acc * 10;
    double diff = para_acc * 0.001;
    double target_speed;

    //    target_speed = fmin(plc_max_speed, sys_max_speed);
    //    target_speed = fmin(target_speed, emot_speed);
    target_speed = fmin(sys_max_speed, emot_speed * var_chan[0].spdl_override);

    if(speed + diff < target_speed)
    {
        speed = speed + diff;
        // sys->is_speedArrived = false;
    }
    else
    {
        var_chan[0].is_mcode_ending = true;
        speed = target_speed;
    }
    return (int32_t)speed;
}
int32_t Spindle_Ccw(void)
{
    //    double plc_max_speed = -10000;
    double sys_max_speed = -1.0 * par_axis[AXIS_SP].spdl_max_speed;
    double para_acc = par_axis[AXIS_SP].feed_acc * 10;
    double diff = para_acc * 0.001;
    double target_speed;

    //    target_speed = fmax(plc_max_speed, sys_max_speed);
    //    target_speed = fmax(target_speed, emot_speed);
    target_speed = fmax(sys_max_speed, emot_speed * var_chan[0].spdl_override);

    if((speed - diff) > target_speed)
    {
        speed = speed - diff;
        // sys->is_speedArrived = false;
    }
    else
    {
        speed = target_speed;
        var_chan[0].is_mcode_ending = true;
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
        // printf("var_chan[0].is_mcode_ending:%d \n",var_chan[0].is_mcode_ending);
        // var_chan[0].is_mcode_ending = true;
    }
    stop_last_state = stop_current_state;
    var_chan[0].cmd_spdl_speed = vel;
}

void emcSpindleControl()
{
    //control logic
    if(sp_sign == 2 && spindle.state == DRV_STATE_OPERATION_ENABLED)
    {
        emot_speed = -1.0 * par_axis[AXIS_SP].default_s ;
        vel = Spindle_Ccw() ;
        sp_stop_sem = false;
        stop_current_state = false;
        stop_last_state = false;
        Hal_Driver_Set_Target_Vel(&spindle,vel);
    }
    else if(sp_sign == 1 && spindle.state == DRV_STATE_OPERATION_ENABLED)
    {
        emot_speed = 1.0 * par_axis[AXIS_SP].default_s ;
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
    if(sp_sign == 0 && !sp_stop_sem)
    {
        Spindle_Stop();
    }
    var_chan[0].cmd_spdl_speed = vel;
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
            spindle.mode = DRV_MODE_CSV;
            sp_sign = 1;
            emot_speed = 1 * mcode_block->spindle.speed ;
        }
        else if(mcode_block->m_code == M04)
        {
            spindle.mode = DRV_MODE_CSV;
            sp_sign = 2;
            emot_speed = -1 * mcode_block->spindle.speed ;
        }
        else if(mcode_block->m_code == M05 || mcode_block->m_code == M30)
        {
            sp_sign = 0;
        }
        mcode_queue_pop(mcode_queue);
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
        var_chan[0].cmd_spdl_speed = vel;
    }
    if(sp_sign == 0)
    {
        if(sp_sign == 0 && !sp_stop_sem)
        {
            Spindle_Stop();
        }
        var_chan[0].cmd_spdl_speed = vel;
    }
}
