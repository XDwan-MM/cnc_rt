#ifndef __JOG_H__
#define __JOG_H__

/* 宏定义 (标准做法：在 extern "C" 外) */
#define JOG_CYCLE_TIME   (0.001)
#define SPEED_THRESHOLD  (1.0e-5)
#define SYS_AXES_NUM_JOG     (3)

#ifdef __cplusplus
extern "C" {
#endif

/* 类型定义 */
typedef enum {
    JOG_STATE_IDLE = 0,      // 静止
    JOG_STATE_RUNNING,       // 运动中 (状态锁定，直到释放)
    JOG_STATE_STOPPING,      // 减速停止中
} JOG_STATE;

typedef struct {
    JOG_STATE state;
    int direction;           // 记忆方向：JOG_DIR_POSITIVE 或 JOG_DIR_NEGATIVE
} JOG_AXIS_CTRL;

/* 外部公共接口声明 */
void handle_jog_cmd();

#ifdef __cplusplus
}
#endif

#endif /* __JOG_H__ */
