#ifndef __JOG_H__
#define __JOG_H__

#include "simple_tp.h"

#define JOG_CYCLE_TIME   (0.001)   /* 伺服周期 1ms */
#define SPEED_THRESHOLD  (1.0e-5)  /* 停稳速度阈值 */
#define SYS_AXES_NUM_JOG (3)       /* 支持点动的轴数 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JOG_STATE_IDLE = 0,    /* 静态同步状态 */
    JOG_STATE_RUNNING,     /* 运动计算状态 */
    JOG_STATE_STOPPING,    /* 减速退出状态 */
} JOG_STATE;

typedef struct {
    JOG_STATE state;
    int direction;
} JOG_AXIS_CTRL;

/* 全局规划器实例 */
extern simple_tp_t jog_tp[3];  

/* 主入口函数 */
extern void handle_jog_cmd(void);

#ifdef __cplusplus
}
#endif

#endif /* __JOG_H__ */