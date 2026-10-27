#pragma once
#include <stdint.h>
#include "hal_config_api.h"
#include "rt_hal_config.h"

/* 单笔 SDO 的等待上限。停机时要等 worker 退出，所以这个值决定了最坏停机时间。 */
#define RT_HAL_SDO_MAX_TIMEOUT_MS 10000u

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SDO 通道：业务侧提交请求，独立工作线程执行，业务侧轮询取结果。
 *
 * 为什么要两段式：HAL 的 SDO 是阻塞传输，PDO / 运动周期线程一旦直接调用就会
 * 抖掉整拍。这里把阻塞限制在 worker 线程内，业务侧只做"提交 + 轮询"。
 *
 * 生命周期：
 *   rt_hal_sdo_prepare() → 提交 / 取结果 → rt_hal_sdo_stop()
 * stop 之后仍可能有一笔正在传输中，必须等 worker 真正退出才能销毁 context。
 *
 * 队列容量 8 笔，占满时提交返回 HAL_ERROR_BUSY。
 */

/* 开始接收请求。须在 HAL 已经 start 之后调用；重复调用返回 HAL_ERROR_BUSY。 */
int rt_hal_sdo_prepare(void);

/* 停止接收：排队中的请求立即以 HAL_ERROR_STOPPED 结束，并唤醒所有等待者。 */
void rt_hal_sdo_stop(void);

/* 工作线程入口，由线程管理层创建；业务侧不要直接调用。 */
void* rt_hal_sdo_worker(void* arg);

/* 按 RT 轴槽读写标准轴对象，index 传第一轴的基础对象号 0x6000..0x67FF。
 * 多轴从站的 0x800 偏移由适配层负责，调用方不要自己加。 */
int rt_hal_sdo_submit_read(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                           uint32_t size, uint32_t timeout_ms, uint32_t* request_id);
int rt_hal_sdo_submit_write(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                            const void* data, uint32_t size, uint32_t timeout_ms,
                            uint32_t* request_id);

/* 按物理从站读写，对象号原样使用，不做轴偏移。用于非轴设备或厂商对象。 */
int rt_hal_sdo_submit_slave_read(int slave_pos, uint16_t index, uint8_t subindex,
                                 uint32_t size, uint32_t timeout_ms, uint32_t* request_id);
int rt_hal_sdo_submit_slave_write(int slave_pos, uint16_t index, uint8_t subindex,
                                  const void* data, uint32_t size, uint32_t timeout_ms,
                                  uint32_t* request_id);

/* 取结果。消费式：取走后该 request_id 立即失效。
 * 未做完返回 HAL_ERROR_BUSY；做完返回 0，执行结果写进 *result。
 * 读请求成功才复制 data，且 size 必须与提交时一致；写请求的 size 必须为 0。 */
int rt_hal_sdo_take_result(uint32_t request_id, void* data, uint32_t size, int* result);

#ifdef __cplusplus
}
#endif
