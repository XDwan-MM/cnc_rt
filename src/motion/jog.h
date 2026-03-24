#ifndef __JOG_H__
#define __JOG_H__

#include <stdbool.h>
#include "simple_tp.h"
#include "pardata.h"

/* JOG方向定义 */
#define JOG_DIR_NONE      (0)
#define JOG_DIR_POSITIVE  (1)
#define JOG_DIR_NEGATIVE  (-1)

/* 运行控制常量 */
#define JOG_CYCLE_TIME   (0.001)   /* 伺服周期 1ms */
#define SPEED_THRESHOLD  (1.0e-5)  /* 停止速度判断阈值 */
#define SYS_AXES_NUM_JOG (3)       /* 轴数 */

#ifdef __cplusplus
extern "C" {
#endif

/* JOG 状态机定义 */
typedef enum {
    JOG_STATE_IDLE = 0,    /* 静态同步：规划器跟随当前实际位置 */
    JOG_STATE_RUNNING,     /* 运动执行：响应按键和倍率 */
    JOG_STATE_STOPPING,    /* 减速停止：线性减速至零 */
} JOG_STATE;

typedef struct {
    JOG_STATE state;
    int direction;
} JOG_AXIS_CTRL;

/* 全局规划器实例，供系统其他模块读取指令位置/速度 */
extern simple_tp_t jog_tp[SYS_AXES_NUM_JOG];  

/* 1ms 周期调用的主任务函数 */
extern void handle_jog_cmd();

/* 处理急停、复位、切模式等异常情况 */
extern void jog_handle_estop();

#ifdef __cplusplus
}
#endif

#endif /* __JOG_H__ */