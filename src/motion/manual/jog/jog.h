/**
 * @file        jog.h
 * @brief       JOG模式控制主模块
 * @date        2026/09/04
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院一所
 */

#ifndef JOG_H
#define JOG_H

#include <stdbool.h>

/*
 * 公共 JOG 常量。
 * 状态机、按键锁存、停止原因和规划器均为 jog.c 私有实现，头文件不暴露。
 */
#define JOG_DIR_NONE (0)
#define JOG_DIR_POSITIVE (1)
#define JOG_DIR_NEGATIVE (-1)

/* 运行控制常量 */
#define JOG_SPEED_THRESHOLD (1.0e-5) /* 停止速度判断阈值 */
#define JOG_AXIS_COUNT (3)           /* JOG支持的轴数 */

/* 软限位刹车距离系数，增大该值会更早停止。 */
#define JOG_BRAKE_FACTOR (0.9999)

#ifdef __cplusplus
extern "C"
{
#endif

/* 1ms 周期调用的主任务函数。 */
extern void handle_jog_cmd(void);

/* 清除 JOG 运动请求并停止规划器；外部减速 PLC 电平会保留在 AxisStatus。 */
extern void jog_handle_estop(void);

/* 提供给模式切换逻辑的只读轴速度，非法轴号返回 0。 */
extern double jog_get_axis_velocity(int axis);

/*
 * 机械锁住解除后重新对齐规划位置：丢弃锁住期间产生的虚拟位移，
 * 并清除按键锁存，避免解除瞬间按残留的按键请求继续运动。
 */
extern void jog_resync_position(void);

#ifdef __cplusplus
}
#endif

#endif /* JOG_H */
