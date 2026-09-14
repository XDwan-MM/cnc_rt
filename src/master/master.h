#ifndef MASTER_H
#define MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

extern int master_error_code;

// 启动主站
int Master_Start();
// 收发主站周期数据
void Master_Run();
// 等待中断信号
void Master_Wait_Sync();
// 复位
void Master_Reset();
// 检查驱动器报错
void Check_Driver_Error();
// 关闭主站
int Master_Close();

void driver_set_speed_mode();
void driver_set_pos_mode();
void driver_set_state_on();
void driver_set_state_off();

// 主轴驱动器上使能
extern void spindle_driver_servo_on();
// 主轴驱动器下使能
extern void spindle_driver_servo_off();
// 主轴驱动器设置速度模式
extern void spindle_driver_set_speed_mode();
// 主轴驱动器设置位置模式
extern void spindle_driver_set_position_mode();

#ifdef __cplusplus
}
#endif
#endif // MASTER_H
