/**
 * @file rt_axis_view.h
 * @brief 轴量的**按需取用**接口：脉冲计数 + 两种速度口径。
 *
 * 只读 rt_data（RT 用户单位量）与机床参数（电子齿轮比），不动 HAL、不做周期工作，
 * 调一次算一次；返回的是"最近一次采样/规划的结果"。用户单位与驱动器脉冲的换算
 * 只在本层实现。
 */
#ifndef RT_AXIS_VIEW_H
#define RT_AXIS_VIEW_H

#include <stdint.h>

/* 返回码：0 = 成功，其余为失败码。 */
#define RT_AXIS_VIEW_OK               0
#define RT_AXIS_VIEW_ERR_ARG         (-1)  /* 轴号越界或出参为空 */
#define RT_AXIS_VIEW_ERR_UNSUPPORTED (-2)  /* 该轴没有这个量（主轴没有脉冲计数/mm/min） */
#define RT_AXIS_VIEW_ERR_PARAM       (-3)  /* 机床参数没给全（电子齿轮比为 0），算不出来 */

/**
 * @brief 指令脉冲（发给驱动器的位置），单位：指令计数
 *
 * 只有进给轴有；主轴返回 UNSUPPORTED（它是累计转角，不是直线计数）。
 *
 * @param axis AXIS_X / AXIS_Y / AXIS_Z / AXIS_SP
 * @param out  带出计数
 * @return int 见上面的返回码
 */
int rt_axis_view_cmd_counts(int axis, int64_t* out);

/**
 * @brief 实际反馈脉冲（驱动器反馈的位置），单位：反馈计数
 *
 * @param axis 同 rt_axis_view_cmd_counts()
 * @param out  带出计数
 * @return int 见上面的返回码
 */
int rt_axis_view_fb_counts(int axis, int64_t* out);

/**
 * @brief 进给轴实际速度，单位 mm/min
 *
 * @param axis 进给轴；主轴返回 UNSUPPORTED（没有 mm/min 这个量）
 * @param out  带出速度
 * @return int 见上面的返回码
 */
int rt_axis_view_speed_mm_min(int axis, double* out);

/**
 * @brief 实际速度，单位 rpm
 *
 * 主轴 = 主轴转速；进给轴 = 电机转速（由 mm/min ÷ 每转位移 得到）。
 *
 * @param axis 任意轴
 * @param out  带出转速
 * @return int 见上面的返回码
 */
int rt_axis_view_speed_rpm(int axis, double* out);

/**
 * @brief 主轴累计转角（度）：位置模式下"走过的角度"反馈
 *
 * 来源 HAL 的 HalCSpindleStatus.position_deg，跨圈已展开、**不取模**。
 * 只有主轴有角度；进给轴返回 UNSUPPORTED。
 *
 * @param axis 必须是主轴（AXIS_SP）
 * @param out  带出角度
 * @return int 0 = 成功；UNSUPPORTED（非主轴 / HAL 没数据）；ARG（越界或空指针）
 */
int rt_axis_view_pos_deg(int axis, double* out);

#endif /* RT_AXIS_VIEW_H */
