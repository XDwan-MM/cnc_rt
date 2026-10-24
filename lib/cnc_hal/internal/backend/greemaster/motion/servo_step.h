#pragma once

#include <stdint.h>
#include "backend/greemaster/export.h"
#include "backend/greemaster/motion/ds402.h"   /* Ds402Request / Ds402ModeSwitch */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 朝指定请求推进一拍 DS402 状态机，并保证运行模式。
 *
 * 请求每拍传入、本层不持有：同一个请求要一直传到达成为止（典型使能 3~4 拍）。
 * 模式不对会自动切（切前用实际位置预置目标防飞车），且模式不对就不上使能。
 *
 * @param[in]  slot    设备槽号（不是从站号）。
 * @param[in]  req     往哪推：使能 / 去使能 / 断电 / 清错 / 不动。
 * @param[in]  op_mode 期望运行模式（CSP / CSV），仅 REQ_ENABLE 用。
 * @param[out] sw      带出本拍状态字，可为 NULL。
 * @return 0 成功；< 0 参数、句柄或任一 PDO 读写失败。
 */
MASTER_API int Master_ServoStep(int slot, Ds402Request req,
                                uint16_t op_mode, uint16_t* sw);

/**
 * @brief 设置切模式策略（已使能时发现模式不对该怎么切）。
 *
 * @param[in] slot 设备槽号。
 * @param[in] how  DISABLE_FIRST（默认）/ IN_PLACE / DISABLE_THEN_MODE。
 * @return 0 成功；< 0 槽号越界或策略非法。
 */
MASTER_API int Master_ServoSetModeSwitch(int slot, Ds402ModeSwitch how);

/**
 * @brief 取切模式策略。
 *
 * @param[in]  slot 设备槽号。
 * @param[out] out  带出策略。
 * @return 0 成功；< 0 槽号越界或 out 为空。
 */
MASTER_API int Master_ServoGetModeSwitch(int slot, Ds402ModeSwitch* out);

/**
 * @brief 清空各槽的策略缓存；生命周期阶段调用，不与周期调用并发。
 */
void Master_ServoReset(void);

#ifdef __cplusplus
}
#endif
