#ifndef __JOG_H__
#define __JOG_H__

#include <global.h>

#ifdef __cplusplus
extern "C" {
#endif

// #include "./datadef/shm.h" // 假设基础数据定义在此

// JOG 运行状态
typedef enum {
    JOG_STATE_IDLE = 0,      // 空闲
    JOG_STATE_RUNNING,       // 点动运行中
    JOG_STATE_STOPPING,      // 减速停止中
} JOG_STATE;

// 轴 JOG 控制结构体
typedef struct {
    JOG_STATE state;         // 状态机状态
    double cycle_time;       // 采样周期，默认 0.001
} JOG_CTRL;

// 外部接口
void handle_jog_cmd();

#ifdef __cplusplus
}
#endif

#endif
