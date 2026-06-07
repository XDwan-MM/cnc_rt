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

#ifdef __cplusplus
}
#endif
#endif // MASTER_H

