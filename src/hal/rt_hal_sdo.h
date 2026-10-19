#pragma once
#include <stdint.h>
#include "hal_config_api.h"

#define RT_HAL_SDO_MAX_TIMEOUT_MS 10000u

#ifdef __cplusplus
extern "C" {
#endif

/* 非实时线程提交和取结果；最多保留 8 笔未取结果。
 * timeout_ms 不得超过 10 秒，以保证停机能等待工作线程退出。 */
int rt_hal_sdo_prepare(void);
void rt_hal_sdo_stop(void);
void* rt_hal_sdo_worker(void* arg);

int rt_hal_sdo_submit_read(int slave_pos, uint16_t index, uint8_t subindex,
                           uint32_t size, uint32_t timeout_ms, uint32_t* request_id);
int rt_hal_sdo_submit_write(int slave_pos, uint16_t index, uint8_t subindex,
                            const void* data, uint32_t size, uint32_t timeout_ms,
                            uint32_t* request_id);
/* 未完成返回 BUSY；完成返回 OK，执行结果写入 *result，取走后 ID 失效。
 * 读成功才复制 data；读的 size 须与提交时一致，写的 size 为 0。 */
int rt_hal_sdo_take_result(uint32_t request_id, void* data, uint32_t size, int* result);

#ifdef __cplusplus
}
#endif
