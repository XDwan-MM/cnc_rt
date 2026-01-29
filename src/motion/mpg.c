#include "mpg.h"
#include "handwheel.h"

uint32_t axisandrate = 0;
uint32_t current_count = 0; // 手轮计数器
bool mpg_statis = true;     // true为复位状态
int rate = 0;               // 倍率
int axis = 0;               // 轴选
int last_axis = 0;          // 上一个周期轴选
int last_rate = 0;          // 上一个周期倍率

void handle_mpg_cmd() {
    HW_Get_Count(&current_count);
    HW_Get_Axis_And_Rate(&axisandrate);
    rate = axisandrate & 0b111;
    rate = Get_Multiplier_Switch(rate);
    axis = (axisandrate >> 3) & 0b11111;
    axis = Get_Axis_Switch(axis);

    if (axis == -1 || (last_axis != axis) || (last_rate != rate)) {
        for (int i = 0; i < 5; ++i) {
            simple_tp[i].first_pass = 1;
        }
        mpg_statis = true;
    } else if (axis >= 0 && axis < 5) {
        simple_tp[axis].joint.count = current_count;
        simple_tp[axis].joint.rate = rate;
        if (axis == 0) {
            //            simple_tp[axis].joint.drivers_data = pos_act[axis];
            //假定加减速参数
            simple_tp[0].joint.vel_limit = par_axis[AXIS_X].mpg_spd;
            simple_tp[0].joint.jaccel_limit = par_axis[AXIS_X].rapid_acc;
        }
        if (axis == 1) {
            //假定加减速参数
            simple_tp[1].joint.vel_limit = par_axis[AXIS_Y].mpg_spd;
            simple_tp[1].joint.jaccel_limit = par_axis[AXIS_Y].rapid_acc;
        }
        if (axis == 2) {
            //假定加减速参数
            simple_tp[2].joint.vel_limit = par_axis[AXIS_Z].mpg_spd;
            simple_tp[2].joint.jaccel_limit = par_axis[AXIS_Z].rapid_acc;
        }

        //初始化
        if (mpg_statis) {
            if (axis == 0) {
                simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = var_axis[AXIS_X].cmd_pos;
            }
            if (axis == 1) {
                simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = var_axis[AXIS_Y].cmd_pos;
            }
            if (axis == 2) {
                simple_tp[axis].joint.pos_cmd = simple_tp[axis].pos_cmd = simple_tp[axis].curr_pos = var_axis[AXIS_Z].cmd_pos;
            }
            simple_tp[axis].curr_vel = 0;
            mpg_statis = false;
        }

        Handlewheels_Motion(&simple_tp[axis]);
        Simple_Tp_Update(&simple_tp[axis]);
        simple_tp[axis].joint.pos_cmd = simple_tp[axis].curr_pos;
        // printf("当前手轮计数：%u, 轴选：%d, 倍率：%d，手轮规划位置： %f  电机位置：%d\n", current_count, axis, rate,simple_tp[axis - 1].pos_cmd,pos_act);
    }
    if (!mpg_statis) {
        switch (axis) {
            case 0:
                var_axis[AXIS_X].cmd_pos = simple_tp[0].joint.pos_cmd;
                printf("simple_tp[0].joint.pos_cmd:%f\n", simple_tp[0].joint.pos_cmd);
                break;
            case 1:
                var_axis[AXIS_Y].cmd_pos = simple_tp[1].joint.pos_cmd;
                printf("simple_tp[1].joint.pos_cmd:%f %f\n", simple_tp[1].joint.pos_cmd, var_axis[AXIS_Y].cmd_pos);
                break;
            case 2:
                var_axis[AXIS_Z].cmd_pos = simple_tp[2].joint.pos_cmd;
                printf("simple_tp[2].joint.pos_cmd:%f\n", simple_tp[2].joint.pos_cmd);
                break;
        }
    }
    last_axis = axis;
    last_rate = rate;
    // hal write
    //   Hal_Driver_Update_All(false,true);
}
