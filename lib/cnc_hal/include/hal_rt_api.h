/**
 * @file  hal_rt_api.h
 * @brief 周期面接口：由唯一一个实时线程按 wait -> begin -> 请求/读数 -> commit 的三拍节奏调用。
 *
 * 约定：
 *  - 返回 0 只表示「请求已被受理」，不表示硬件已完成动作；实际状态一律从统一轴快照读取。
 *  - 在途相同控制请求幂等；冲突按各入口规则拒绝且不排队，停止/复位可取消已有动作。
 *  - 请求窗口指 begin 与 commit 之间；disable/estop 例外，可在窗口外登记停止意图。
 *  - 除 hal_context_request_stop() 外，本文件所有接口都不允许跨线程并发调用。
 *  - 快照读取只取缓存，读取失败时出参保持不变。
 *  - 模式准备不会顺带使能；使能完成后不保留「使能意图」，再次使能必须重新显式请求。
 *  - H1 版本不支持正常退出定向；通过前检的请求返回 HAL_ERROR_UNSUPPORTED，不改目标与控制字。
 */

#pragma once
#include "hal_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 等待主站下一个周期帧到达，开启本拍。
 *
 * 只能在上一拍已 commit 之后调用（内部 phase 必须为 0）；等待期间若收到停止请求，
 * 返回 HAL_ERROR_STOPPED 让调用方退出循环。
 *
 * @param[in] c HAL 上下文，须已 start。
 * @return 0 成功；HAL_ERROR_STATE 上一拍未 commit；
 *         HAL_ERROR_STOPPED 已被 request_stop 打断；其余为总线错误码。
 */
int32_t hal_rt_wait_cycle(HalContext* c);

/**
 * @brief 采样本拍：锁存时间戳、读回所有轴的反馈与 IO 输入、推进各轴控制状态机。
 *
 * 本拍不写任何 PDO，只做「读」和状态推进，因此采样结果反映的是上一拍下发的效果。
 * 全部轴采样完成后才推进动作，避免同一拍内先后顺序影响结果。
 *
 * @param[in] c HAL 上下文，须已 start。
 * @return 0 成功；HAL_ERROR_STATE 未在 wait 之后调用；
 *         其余为总线/时钟读取失败，失败后 context 进入闭锁错误。
 */
int32_t hal_rt_begin_cycle(HalContext* c);

/**
 * @brief 提交本拍：复检元操作超时，把缓存的命令与控制字写进 PDO，再提交主站发帧。
 *
 * 请求窗口拖得过长时，超时预算已耗尽的动作不再提交；这里只做超时复检，
 * 不二次消费完成反馈，也不伪造定向边沿。
 *
 * @param[in] c HAL 上下文，须已 start。
 * @return 0 成功；HAL_ERROR_STATE 未在 begin 之后调用；
 *         HAL_ERROR_STOPPED 提交前收到停止请求；其余为总线错误码。
 */
int32_t hal_rt_commit_cycle(HalContext* c);

/**
 * @brief 请求使能轴（DS402 上使能），只登记请求，写 PDO 发生在 commit。
 *
 * 要求模式已就绪、目标模式与实际模式一致、驱动器不在故障态；重复请求幂等。
 * 使能完成后不保留使能意图，再次使能必须重新显式请求。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 逻辑轴号。
 * @return 0 已受理；HAL_ERROR_BUSY 该轴有动作或停止未收口；
 *         HAL_ERROR_MODE_MISMATCH 模式未就绪；HAL_ERROR_NOT_ENABLED 驱动器处于故障态；
 *         HAL_ERROR_STOPPED 本拍刚接受了急停。
 */
int32_t hal_rt_axis_request_enable(HalContext* c, HalAxisId id);

/**
 * @brief 请求下使能轴，只登记停止意图，实际收口在后续周期完成。
 *
 * 可在请求窗口外调用；取消使能/定向，保留在途模式准备/故障复位及模式凭据。
 * 按 HAL_ESTOP_DISABLE_OPERATION 收口；已有撤压约束时继续撤压，不升电凑 Ready。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 逻辑轴号。
 * @return 0 已受理；HAL_ERROR_NOT_RUNNING 未 start；HAL_ERROR_ARGUMENT 轴号非法。
 */
int32_t hal_rt_axis_request_disable(HalContext* c, HalAxisId id);

/**
 * @brief 请求切换运行模式（CSP 或 CSV），模式准备完成前不会自动使能。
 *
 * 模式已生效时只更新目标模式；否则进入 SETTING 状态，按 mode_timeout_ms 预算等待。
 * 重复提交相同模式幂等，提交与当前动作冲突的模式返回 HAL_ERROR_BUSY。
 *
 * @param[in] c    HAL 上下文。
 * @param[in] id   逻辑轴号。
 * @param[in] mode HAL_AXIS_MODE_CSP 或 HAL_AXIS_MODE_CSV。
 * @return 0 已受理；HAL_ERROR_ARGUMENT 模式非法；
 *         HAL_ERROR_UNSUPPORTED 该轴/设备不具备该模式所需 PDO；
 *         HAL_ERROR_BUSY 该轴正忙或模式冲突；HAL_ERROR_STOPPED 本拍刚接受了急停。
 */
int32_t hal_rt_axis_request_mode(HalContext* c, HalAxisId id, HalAxisMode mode);

/**
 * @brief 请求急停：作废当前动作、作废模式凭据，按配置的急停动作收口。
 *
 * 可在请求窗口外调用；estop_action 决定是去使能还是撤销电压。
 * 同拍使能、模式、复位和新定向请求返回 HAL_ERROR_STOPPED。后续停止约束持续保留，
 * 新的明确模式准备/复位请求按各自规则处理，不能把未再次调用急停当作恢复授权。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 逻辑轴号。
 * @return 0 已受理；HAL_ERROR_NOT_RUNNING 未 start；HAL_ERROR_ARGUMENT 轴号非法。
 */
int32_t hal_rt_axis_request_estop(HalContext* c, HalAxisId id);

/**
 * @brief 请求故障复位：把驱动器从 Fault 拉回 Ready，随后的使能须另行请求。
 *
 * 已在 Ready 时无需设备动作；在途重复请求幂等，不重开预算。复位取消已有控制动作、
 * 作废模式凭据与未提交目标；普通下使能不取消复位，急停取消。复位在
 * fault_reset_timeout_ms 预算内等待驱动器状态变化。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 逻辑轴号。
 * @return 0 已受理或已处于 Ready；
 *         HAL_ERROR_STOPPED 本拍刚接受了急停。
 */
int32_t hal_rt_axis_request_fault_reset(HalContext* c, HalAxisId id);

/**
 * @brief 请求主轴定向：按设备字典配置的厂商控制字启动定向。
 *
 * 只在轴尚未处于定向态时发起；须已使能且运行模式到位。该轴未配置定向控制字
 * 或完成位时返回 HAL_ERROR_UNSUPPORTED。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 主轴逻辑轴号。
 * @return 0 已受理或已处于定向态；HAL_ERROR_ARGUMENT 该轴不是主轴；
 *         HAL_ERROR_UNSUPPORTED 未配置定向；HAL_ERROR_BUSY 轴正忙；
 *         HAL_ERROR_NOT_ENABLED / HAL_ERROR_MODE_MISMATCH 运动前置条件不满足。
 */
int32_t hal_rt_spindle_request_orient(HalContext* c, HalAxisId id);

/**
 * @brief 请求退出定向（H1 未支持，通过前检后返回 HAL_ERROR_UNSUPPORTED）。
 *
 * 参数仍做完整校验，便于上层在后续版本直接沿用：kind 为位置时要求实际模式为 CSP，
 * 为转速时要求实际模式为 CSV 且目标值必须为 0。
 *
 * @param[in] c      HAL 上下文。
 * @param[in] id     主轴逻辑轴号。
 * @param[in] target 退出目标；reserved 必须为 0，value 必须为有限值。
 * @return HAL_ERROR_UNSUPPORTED；参数或模式不合法时返回 HAL_ERROR_ARGUMENT /
 *         HAL_ERROR_MODE_MISMATCH / HAL_ERROR_BUSY。
 */
int32_t hal_rt_spindle_request_exit_orient(HalContext* c, HalAxisId id,
                                          const HalOrientExitTarget* target);

/**
 * @brief 清除该轴的执行错误槽（快照里的 execution_error）。
 *
 * @param[in] c  HAL 上下文。
 * @param[in] id 逻辑轴号。
 * @return 0 成功；其余为准入错误（未 start、轴号非法、不在请求窗口内等）。
 */
int32_t hal_rt_axis_clear_execution_error(HalContext* c, HalAxisId id);

/**
 * @brief 写 CSP 目标位置（0x607A），commit 时下发。
 *
 * 要求该轴已使能、模式已就绪且目标与实际模式都是 CSP；参数按命令当量换算成原始计数，
 * 非有限值或超出 32 位范围会被拒绝。
 *
 * @param[in] c   HAL 上下文。
 * @param[in] id  逻辑轴号。
 * @param[in] pos 目标位置，用户单位（与反馈同为工程单位）。
 * @return 0 已受理；HAL_ERROR_ARGUMENT 非有限值或超范围；
 *         HAL_ERROR_UNSUPPORTED 该轴无位置 PDO；其余为运动前置条件错误码。
 */
int32_t hal_rt_axis_write_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 写主轴 CSV 目标转速（0x60FF），commit 时下发。
 *
 * 要求该轴是主轴、已使能、目标与实际模式都是 CSV；rpm 取绝对值并受 max_speed 限制，
 * dir 取 -1/0/1 作为方向乘子，最终按 speed_unit 换算成设备单位。
 *
 * @param[in] c   HAL 上下文。
 * @param[in] id  主轴逻辑轴号。
 * @param[in] rpm 目标转速绝对值，单位 rpm，须在 [0, max_speed] 内。
 * @param[in] dir 方向：-1 反转、0 停止、1 正转。
 * @return 0 已受理；HAL_ERROR_ARGUMENT 非主轴、rpm 越界或 dir 非法；
 *         HAL_ERROR_UNSUPPORTED 该轴无速度 PDO；其余为运动前置条件错误码。
 */
int32_t hal_rt_spindle_write_speed(HalContext* c, HalAxisId id, double rpm, int32_t dir);

/**
 * @brief 设定软件坐标：把当前实际位置标定为 pos，只改偏置，不动硬件。
 *
 * 同时平移反馈偏置与命令偏置，使后续的目标位置、反馈位置都落在新坐标系里；
 * 已有命令位置回显时一并平移。
 *
 * @param[in] c   HAL 上下文。
 * @param[in] id  逻辑轴号。
 * @param[in] pos 新的当前位置，用户单位。
 * @return 0 成功；HAL_ERROR_ARGUMENT 参数非有限值或偏置溢出；
 *         HAL_ERROR_UNSUPPORTED 该轴尚无位置反馈。
 */
int32_t hal_rt_axis_set_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 读取该轴的统一快照；只读缓存，不访问总线。
 *
 * 读取失败时 *out 保持不变。是否可用要看 snapshot.available / feedback_valid /
 * command_valid 位，不要只看返回值。
 *
 * @param[in]  c   HAL 上下文。
 * @param[in]  id  逻辑轴号。
 * @param[out] out 带出快照。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 本拍尚未采样；
 *         HAL_ERROR_ARGUMENT 轴号非法或 out 为空。
 */
int32_t hal_rt_axis_read_snapshot(HalContext* c, HalAxisId id, HalAxisSnapshot* out);

/**
 * @brief 把本拍输入映像拷进调用方的 X 区；段内覆盖，段外补 0。
 *
 * 缓冲长度必须不小于 x_size（见 hal_io_image_size），否则报参数错误。
 *
 * @param[in]  c     HAL 上下文。
 * @param[out] image 调用方的 X 区缓冲，长度 >= x_size。
 * @param[in]  len   缓冲字节数。
 * @return 0 成功；HAL_ERROR_ARGUMENT 长度不足；HAL_ERROR_NOT_RUNNING 尚未采样。
 */
int32_t hal_rt_io_snapshot_inputs(HalContext* c, uint8_t* image, uint32_t len);

/**
 * @brief 把调用方的 Y 区按位拆进各输出 Entry 并置脏，真正下发在 commit。
 *
 * HAL 不做读改写：只改其中几位也由调用方自己保持整段一致。
 *
 * @param[in] c     HAL 上下文。
 * @param[in] image 调用方的 Y 区缓冲，长度 >= y_size。
 * @param[in] len   缓冲字节数。
 * @return 0 成功；HAL_ERROR_ARGUMENT 长度不足；
 *         HAL_ERROR_STATE 不在 begin 与 commit 之间。
 */
int32_t hal_rt_io_flush_outputs(HalContext* c, const uint8_t* image, uint32_t len);
#ifdef __cplusplus
}
#endif
