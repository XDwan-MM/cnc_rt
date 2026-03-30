#include <parapi.h>
#include "cnc_math.h"
#include "rt_data.h"
#include "spindle_test.h"

SPINDLE_DATA* spl_data;

void Spindle_init(SPINDLE_DATA *spl_data);
static void Spindle_Cw(SPINDLE_DATA *spl_data);
static void Spindle_Ccw(SPINDLE_DATA *spl_data);
static void spindle_mcode_feedback();

void spindle_set_state(){
    double emot_speed = 0;
    double sys_max_speed = par_axis[AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real;
    double para_acc = par_axis[AXIS_SP][PAR_AX_FEED_ACC].value.val_real * 10;
    spl_data->diff = para_acc * 0.001;
    Hal_Driver_Get_Actual_Vel(&spindle, &spl_data->current_speed);
    // 如果急停或按下复位
    if(rt_data->sys_state[SYS_IS_ESTOP_ON] || rt_data->nc_event == SYS_EVENT_RESET) spl_data->spl_event = SPINDLE_EVENT_STOP;
    // 根据主轴事件设置主轴状态（事件：面板按键和M代码）
    switch (spl_data->spl_event)
    {
    case SPINDLE_EVENT_STOP:
        spl_data->target_speed = 0;
        if (spl_data->spl_state == SPINDLE_CW) spl_data->spl_state = SPINDLE_CCW;
        if (spl_data->spl_state == SPINDLE_CCW) spl_data->spl_state = SPINDLE_CW;
        break;
    case SPINDLE_EVENT_CW:
        spl_data->target_speed = fmin(sys_max_speed, emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f);
        spl_data->spl_state = SPINDLE_CW;
        break;
    case SPINDLE_EVENT_CCW:
        spl_data->target_speed = fmax(sys_max_speed, emot_speed * var_chan[0][VAR_CHAN_SPDL_OVERRIDE].value.f);
        spl_data->spl_state = SPINDLE_CCW;
        break;
    case SPINDLE_EVENT_NONE:
    default:
        break;
    }
    spl_data->spl_event = SPINDLE_EVENT_NONE;
}

void spindle_cal_speed(){
    switch (spl_data->spl_state)
    {
    case SPINDLE_STOP:
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
    Hal_Driver_Set_Target_Vel(&spindle, (int32_t)spl_data->current_speed);
    setVarChanDouble(0, VAR_CHAN_CMD_SPDL_SPEED, spl_data->current_speed, 0);
}

bool spindle_is_stopped(){
    return spl_data->spl_state == SPINDLE_STOP;
}

bool spindle_is_rotating(){
    return spindle_is_stopped() ? false : true;
}

// 主轴正转，输入参数d_speed 主轴转速 unit 转/分钟
static void Spindle_Cw(SPINDLE_DATA *spl_data)
{
    if(spl_data->current_speed + spl_data->diff < spl_data->target_speed)
    {
        spl_data->current_speed = spl_data->current_speed + spl_data->diff;
    }
    else
    {
        spl_data->current_speed = spl_data->target_speed;
        spindle_mcode_feedback();
    }
}

// 主轴反转
static void Spindle_Ccw(SPINDLE_DATA *spl_data)
{
    if((spl_data->current_speed - spl_data->diff) > spl_data->target_speed)
    {
        spl_data->current_speed = spl_data->current_speed - spl_data->diff;
    }
    else
    {
        spl_data->current_speed = spl_data->target_speed;
        spindle_mcode_feedback();
    }
}

static void spindle_mcode_feedback(){
    // 只在完成M指令后执行一次
    if(!spl_data->mcode_feedback){
        rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
        spl_data->mcode_feedback = true;
    }
}

void Spindle_init(SPINDLE_DATA *spl_data){
    spl_data = (SPINDLE_DATA*)calloc(sizeof(SPINDLE_DATA), 1);
}