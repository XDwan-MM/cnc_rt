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

/*
 * 重建手轮位置基准：丢弃规划器里尚未输出的位移，下一周期按实时数据中的
 * wcs_cmd_pos_cur 重新对齐并从当前硬件计数重新开始（first_pass）。
 * 由 emcmot 在重新进入手轮模式时调用（离开手轮后其他模式可能已移动过机床）。
 */
void handwheel_resync_position(void);

#ifdef __cplusplus
}
#endif

#endif /* MPG_H */
