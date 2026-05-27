#include <parapi.h>
#include "cnc_math.h"
#include "rt_data.h"
#include "spl_control.h"
#include <hmisysapi.h>
#include <alarmapi.h>

SPINDLE_DATA spl_data;

static void Spindle_Stop(SPINDLE_DATA *spl_data);
static void Spindle_Cw(SPINDLE_DATA *spl_data);
static void Spindle_Ccw(SPINDLE_DATA *spl_data);
static void spindle_mcode_feedback();
static bool is_spd_vel(SPINDLE_DATA *spl_data);

static double sys_max_speed;
static double para_acc;
static double diff;
static double target_speed;

void spindle_set_state(){
    // 如果急停或按下复位
    if(rt_data->sys_state[SYS_IS_ESTOP_ON] || rt_data->nc_event == SYS_EVENT_RESET)
    {
        spl_data.spl_event = SPINDLE_EVENT_STOP;
        spl_data.emot_speed = 0;
    }
    // 根据主轴事件设置主轴状态（事件：面板按键和M代码）
    switch (spl_data.spl_event)
    {
    case SPINDLE_EVENT_STOP:
        spl_data.spl_state = SPINDLE_STOP;
        break;
    case SPINDLE_EVENT_CW:
        if(is_spd_vel(&spl_data))
        {
            spl_data.spl_state = SPINDLE_CW;
            spindle.mode = DRV_MODE_CSV;
        }
        break;
    case SPINDLE_EVENT_CCW:
        if(is_spd_vel(&spl_data))
        {
            spl_data.spl_state = SPINDLE_CCW;
            spindle.mode = DRV_MODE_CSV;
        }
        break;
    case SPINDLE_EVENT_ORIENT:
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }
    spl_data.spl_event = SPINDLE_EVENT_NONE;
}

void spindle_cal_speed(){
    sys_max_speed = par_axis[AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real;
    para_acc = par_axis[AXIS_SP][PAR_AX_FEED_ACC].value.val_real * 10;
    diff = para_acc * 0.001;
    switch (spl_data.spl_state)
    {
    case SPINDLE_STOP:
        Spindle_Stop(&spl_data);
        break;
    case SPINDLE_CW:
        Spindle_Cw(&spl_data);
        break;
    case SPINDLE_CCW:
        Spindle_Ccw(&spl_data);
        break;
    default:
        break;
    }
    Hal_Driver_Set_Target_Vel(&spindle, (int32_t)spl_data.current_speed);
    setVarChanInt(0, VAR_CHAN_S_CODE, spl_data.current_speed, 0);
}

bool spindle_is_stopped(){
    return spl_data.spl_state == SPINDLE_STOP;
}

bool spindle_is_rotating(){
    return spindle_is_stopped() ? false : true;
}
static void Spindle_Stop(SPINDLE_DATA *spl_data)
{
    if(spl_data->current_speed < 0)
    {
        Spindle_Cw(spl_data);
    }
    else if(spl_data->current_speed > 0)
    {
        Spindle_Ccw(spl_data);
    }
    if(spl_data->current_speed == 0 )
    {
        int32_t vel = 0;
        Hal_Driver_Get_Actual_Vel(&spindle,&vel);
        if(vel == 0)
        {
            spindle.mode = DRV_MODE_DISABLED;
        }
        spl_data->spl_vel_state = SPINDLE_VEL_ZERO;
        spindle_mcode_feedback();
    }
}
// 主轴正转，输入参数d_speed 主轴转速 unit 转/分钟
static void Spindle_Cw(SPINDLE_DATA *spl_data)
{
    target_speed = fmin(sys_max_speed, spl_data->emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100);
    if(spl_data->current_speed + diff < target_speed)
    {
        spl_data->current_speed = spl_data->current_speed + diff;
        spl_data->spl_vel_state = SPINDLE_VEL_RAMP;
    }
    else
    {
        spl_data->current_speed = target_speed;
        spl_data->spl_vel_state = SPINDLE_VEL_ARRIVED;
        spindle_mcode_feedback();
    }
}

// 主轴反转
static void Spindle_Ccw(SPINDLE_DATA *spl_data)
{    
    target_speed = fmax(- sys_max_speed, spl_data->emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f / 100);
    if((spl_data->current_speed - diff) > target_speed)
    {
        spl_data->current_speed = spl_data->current_speed - diff;
        spl_data->spl_vel_state = SPINDLE_VEL_RAMP;
    }
    else
    {
        spl_data->current_speed = target_speed;
        spl_data->spl_vel_state = SPINDLE_VEL_ARRIVED;
        spindle_mcode_feedback();
    }
}

static void spindle_mcode_feedback(){
    // 只在完成M指令后执行一次
    if(!spl_data.mcode_feedback){
        spl_data.mcode_feedback = true;
        LOG(LOG_LEVEL_DEBUG, "主轴速度到达 %d", spl_data.current_speed);
    }
}

static bool is_spd_vel(SPINDLE_DATA *spl_data)
{
    if(!spl_data->mcp_target_speed)
    {
        int res = setAlarm(ALARM_AX, ALARM_ERROR, 20);
        if(res == -8) return false;
        LOG(LOG_LEVEL_ERR, "主轴速度值缺省");
    }
    return true;
}
