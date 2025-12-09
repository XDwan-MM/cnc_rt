#ifndef CNC_KEYBOARD_H
#define CNC_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cnc_shm.h"

// 启动键盘输入控制的CNC测试程序
void test_keyboard_cnc();

// 获取当前CNC状态指针
extern CncState *cnc_state;

#ifdef __cplusplus
}
#endif

#endif // CNC_KEYBOARD_H
