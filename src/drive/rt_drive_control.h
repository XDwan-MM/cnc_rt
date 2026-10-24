#ifndef RT_DRIVE_CONTROL_H
#define RT_DRIVE_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/*
 * RT 侧驱动管理接口。
 *
 * 这一组函数只由运动线程调用（alarm_report_hal_error 例外，见下），并且有严格的
 * 调用顺序：报警轮询 → 复位收口推进 → 最终使能意图提交。
 * 迁移时保留了原有函数名与调用顺序，避免"搬文件"和"改行为"同时发生。
 */

/* 是否有一笔故障复位正在等收口（等待中，或上次失败后被锁定）。 */
bool alarm_fault_reset_pending(void);

/* 新的复位动作调用一次：解除上次"故障复位未收口"造成的使能锁定，
 * 并清掉业务力矩需求与使能状态机。
 * 没有这一步，收口失败一次就只能重启软件才能恢复。 */
void alarm_reset_latch_clear(void);

/* 每拍检查 HAL 侧状态：周期错误码、驱动器故障码、持续未使能诊断、复位收口。
 * 只读适配层快照，不直接碰 HalContext。 */
void alarm_poll_hal();

/* 每拍提交最终使能意图。
 * 必须排在 alarm_poll_hal 与 nc_set_resetted 之后：业务力矩需求在本函数之前
 * 已经提交，而请求是否被采纳、真实硬件反馈如何，都要等之后某一拍 PDO 才知道。 */
void hal_axis_intent_poll();

/* 主轴力矩需求的唯一入口。
 *   false —— 无条件撤销，永远成功；
 *   true  —— 要通过复位状态、急停、轴屏蔽、轴故障等多道闸，被拒时直接返回
 *            false 且不缓存，由调用方自己决定下一步。
 * 注意这只是"需求"：真正发 enable/disable 的是 hal_axis_intent_poll()。 */
bool alarm_spindle_request_torque(bool needed);

/* 通知：该轴有一笔新的模式准备获准提交。
 * 主轴必须由业务指定要准备哪个模式；进给轴则在使能时自动选 CSP。 */
void alarm_axis_mode_requested(int slot, int mode);

/* 把 HAL 错误码（正数）报成 RT 报警。
 * 会被 PDO 线程直接调用：周期循环一退出，运动线程就卡在 sem_wait 上，
 * alarm_poll_hal 不再运行，只有发现错误的一方自己能把它报出去。 */
void alarm_report_hal_error(int32_t rc);

/* 报警事件触发的故障复位入口：只发请求并记录等待状态，
 * 不清运行轨迹，也不结束 NC 复位。 */
void rt_drive_request_fault_reset(void);
#ifdef __cplusplus
}
#endif
#endif
