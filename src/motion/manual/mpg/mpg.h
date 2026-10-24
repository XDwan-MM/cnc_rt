#ifndef MPG_H
#define MPG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* 手轮面板轴选编码；4、5 轴编码保留，但当前设备没有对应进给轴槽位。 */
typedef enum
{
    HANDWHEEL_AXIS_OFF = 0,
    HANDWHEEL_AXIS_X = 1,
    HANDWHEEL_AXIS_Y = 2,
    HANDWHEEL_AXIS_Z = 4,
    HANDWHEEL_AXIS_4 = 8,
    HANDWHEEL_AXIS_5 = 16
} handwheel_axis_t;

/* 手轮倍率开关编码，与面板读取值保持一致。 */
typedef enum
{
    HANDWHEEL_RATE_OFF = 0,
    HANDWHEEL_RATE_X1 = 1,
    HANDWHEEL_RATE_X10 = 2,
    HANDWHEEL_RATE_X100 = 4
} handwheel_rate_t;

/* 每运动周期处理一次 MPG 输入。 */
void handle_mpg_cmd(void);

/* 急停时清空 MPG 规划状态和速度输出。 */
void handwheel_handle_estop(void);

/* 模式入口或反馈坐标重建后，令下一 MPG 周期重建位置和计数基准。 */
void handwheel_resync_position(void);

#ifdef __cplusplus
}
#endif

#endif /* MPG_H */
