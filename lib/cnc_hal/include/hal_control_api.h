/**
 * @file  hal_control_api.h
 * @brief 非周期面接口：生命周期、设备查询、映像尺寸与总线健康。
 *
 * 与周期面（hal_rt_api.h）的分工：那边只允许单个 RT 线程按三拍调用；这里的接口在
 * 启动前后与运行过程中都可以调用。
 *
 * 线程约束：
 *  - hal_context_request_stop() 是唯一允许与周期调用并发的接口。
 *  - SDO 只允许在非实时线程调用，并且 stop/destroy 之前必须等 SDO 返回。
 *  - 查询类快照的并发由调用方自己同步；生命周期类接口在进程内串行，
 *    stop/destroy 之前还须确认所有周期调用已退出。
 *
 * 公共返回码（下面各函数不再逐个重复）：
 *  - HAL_ERROR_ARGUMENT    参数/轴号/从站号/function 非法
 *  - HAL_ERROR_NOT_RUNNING 未 start，或信息需 start 认领设备后才有
 *  - HAL_ERROR_BUSY        主站已被启动中的 context 占用（进程全局单例）
 *  - HAL_ERROR_TIMEOUT     start 总预算耗尽（start_timeout_ms）
 *  - HAL_ERROR_BUS         主站启停失败，或底层健康查询失败
 *
 * 两个计数函数（hal_axis_count / hal_slave_count）以**负数形式**返回错误码，
 * 因为 0 是合法数量。
 *
 * 参数约定：id = 逻辑轴号；c 为空一律返回 HAL_ERROR_ARGUMENT。
 */

#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校验静态配置、分配并初始化 context；不访问硬件、不启动主站。
 *
 * 配置按值整体拷入 context，调用方可以立即释放自己那份；出参 *out 在成功前一律置空。
 *
 * @param[in]  config  配置；abi_major / abi_minor / struct_size / reserved 必须正确。
 * @param[out] out     带出 context 句柄。
 * @param[out] err     错误文本缓冲，可为 NULL。
 * @param[in]  err_len err 长度；非空且 > 0 时始终以 NUL 结尾。
 * @return 0 成功；HAL_ERROR_ABI ABI 版本/结构大小/保留字段不符；
 *         HAL_ERROR_CONFIG 静态校验不过；HAL_ERROR_MEMORY 分配失败；其余见文件头。
 */
int32_t hal_context_create(const HalCConfig* config, HalContext** out, char* err, uint32_t err_len);

/**
 * @brief 启动主站：起主站 -> 扫从站 -> 按设备字典认领设备/PDO -> 算 X/Y 映像尺寸。
 *
 * 任何一步失败都整体回滚，不留半启动的 context。主站是进程全局单例，已有启动中的
 * context 时返回 HAL_ERROR_BUSY。真实轴数、PDO 位宽、X/Y 段重叠在这一步复查
 * （create 的静态校验看不到这些）。耗时受 start_timeout_ms 约束。
 *
 * @param[in]  c       HAL 上下文。
 * @param[out] err     错误文本缓冲，同 hal_context_create。
 * @param[in]  err_len err 长度。
 * @return 0 成功；HAL_ERROR_BUSY 主站被占用；HAL_ERROR_TIMEOUT 预算耗尽；
 *         HAL_ERROR_CONFIG 字典加载失败、轴/IO 无匹配设备、X/Y 段重叠或位长越界；
 *         HAL_ERROR_UNSUPPORTED 必需 PDO 位长不符 CiA402（或主轴缺速度 PDO）；
 *         HAL_ERROR_BUS 主站启动失败；其余见文件头。
 */
int32_t hal_context_start(HalContext* c, char* err, uint32_t err_len);

/**
 * @brief 请求停止：只写通知，不释放资源；本文件唯一允许与周期调用并发的接口。
 *
 * 同时置两份通知：HAL 侧使后续接口返回 HAL_ERROR_STOPPED；驱动层使阻塞中的
 * hal_rt_wait_cycle() 提前退出。收尾仍须走 stop / destroy。
 *
 * @param[in] c HAL 上下文。
 * @return 0 成功；HAL_ERROR_ARGUMENT c 为空。
 */
int32_t hal_context_request_stop(HalContext* c);

/**
 * @brief 关主站并清除运行状态；重复调用无害。
 *
 * 调用方必须先确认所有周期调用已退出（本函数不等待、也不检测）。非当前占用主站的
 * context 只清状态，不关主站。
 *
 * @param[in] c HAL 上下文。
 * @return 0 成功；HAL_ERROR_BUS 关主站失败；HAL_ERROR_ARGUMENT c 为空。
 */
int32_t hal_context_stop(HalContext* c);

/**
 * @brief 读一笔 SDO（物理从站寻址），供非实时线程访问对象字典。
 *
 * 须已 start；同一时刻只受理一笔请求，PDO 周期线程不得调用；SDK 单次数据最多 4 字节。
 * 读失败时 data 保持不变。
 *
 * @param[in]  c          HAL 上下文。
 * @param[in]  slave_pos  从站位置号。
 * @param[in]  index      对象索引。
 * @param[in]  subindex   对象子索引。
 * @param[out] data       接收缓冲。
 * @param[in]  size       接收字节数，上限 4。
 * @param[in]  timeout_ms 总等待时间。
 * @return 0 成功；HAL_ERROR_SDO 应答失败或长度不符；
 *         HAL_ERROR_BUSY / HAL_ERROR_STOPPED / HAL_ERROR_TIMEOUT / HAL_ERROR_BUS。
 */
int32_t hal_sdo_read(HalContext* c, int32_t slave_pos, uint16_t index,
                     uint8_t subindex, void* data, uint32_t size, uint32_t timeout_ms);

/**
 * @brief 写一笔 SDO（物理从站寻址）。
 *
 * 约束同 hal_sdo_read：须已 start、单笔受理、周期线程不得调用、单次最多 4 字节。
 *
 * @param[in] c          HAL 上下文。
 * @param[in] slave_pos  从站位置号。
 * @param[in] index      对象索引。
 * @param[in] subindex   对象子索引。
 * @param[in] data       待写数据。
 * @param[in] size       数据字节数，上限 4。
 * @param[in] timeout_ms 总等待时间。
 * @return 0 成功；HAL_ERROR_SDO 应答失败或长度不符；
 *         HAL_ERROR_BUSY / HAL_ERROR_STOPPED / HAL_ERROR_TIMEOUT / HAL_ERROR_BUS。
 */
int32_t hal_sdo_write(HalContext* c, int32_t slave_pos, uint16_t index,
                      uint8_t subindex, const void* data, uint32_t size, uint32_t timeout_ms);

/**
 * @brief 按逻辑轴读标准轴对象，index 传第一轴的基础对象号 0x6000..0x67FF。
 *
 * 多轴从站由 HAL 按轴序号自动加 0x800；厂商对象或非轴设备请改用物理从站接口
 * hal_sdo_read。其余约束同 hal_sdo_read。
 *
 * @param[in]  c          HAL 上下文。
 * @param[in]  axis       逻辑轴号。
 * @param[in]  index      基础对象索引。
 * @param[in]  subindex   对象子索引。
 * @param[out] data       接收缓冲。
 * @param[in]  size       接收字节数，上限 4。
 * @param[in]  timeout_ms 总等待时间。
 * @return 0 成功；HAL_ERROR_ARGUMENT 轴号非法或 index 不在标准轴对象范围；
 *         HAL_ERROR_SDO 应答失败或长度不符；其余同 hal_sdo_read。
 */
int32_t hal_axis_sdo_read(HalContext* c, HalAxisId axis, uint16_t index,
                          uint8_t subindex, void* data, uint32_t size, uint32_t timeout_ms);

/**
 * @brief 按逻辑轴写标准轴对象，index 传第一轴的基础对象号 0x6000..0x67FF。
 *
 * 多轴从站由 HAL 按轴序号自动加 0x800；厂商对象或非轴设备请改用物理从站接口
 * hal_sdo_write。其余约束同 hal_sdo_write。
 *
 * @param[in] c          HAL 上下文。
 * @param[in] axis       逻辑轴号。
 * @param[in] index      基础对象索引。
 * @param[in] subindex   对象子索引。
 * @param[in] data       待写数据。
 * @param[in] size       数据字节数，上限 4。
 * @param[in] timeout_ms 总等待时间。
 * @return 0 成功；HAL_ERROR_ARGUMENT 轴号非法或 index 不在标准轴对象范围；
 *         HAL_ERROR_SDO 应答失败或长度不符；其余同 hal_sdo_write。
 */
int32_t hal_axis_sdo_write(HalContext* c, HalAxisId axis, uint16_t index,
                           uint8_t subindex, const void* data, uint32_t size, uint32_t timeout_ms);

/**
 * @brief 非实时线程设置主轴定向位置，pulses 为 32 位原始脉冲值，不做角度换算。
 *
 * 蓝海华腾写 0x200E:11，四合一写固定对象 0x204B:00，不加轴偏移。
 * 只写参数，不触发定向；成功后再由周期线程请求定向。
 * 生命周期、并发与超时约束同 hal_sdo_write。
 *
 * @param[in] c          HAL 上下文。
 * @param[in] axis       主轴逻辑轴号。
 * @param[in] pulses     定向位置的原始脉冲值。
 * @param[in] timeout_ms 总等待时间。
 * @return 0 成功；HAL_ERROR_UNSUPPORTED 该型号不支持定向定位；
 *         HAL_ERROR_ARGUMENT 轴号非法；其余同 hal_sdo_write。
 */
int32_t hal_spindle_set_orient_pulses(HalContext* c, HalAxisId axis,
                                    uint32_t pulses, uint32_t timeout_ms);

/**
 * @brief stop + free 一步收尾，等价于先 hal_context_stop 再释放 context。
 *
 * @param[in] c HAL 上下文，可为 NULL（此时什么也不做）。
 */
void hal_context_destroy(HalContext* c);

/**
 * @brief 逻辑轴号转 HalAxisId；当前实现是校验存在后原样返回。
 *
 * 失败时 *out = HAL_INVALID_ID。
 *
 * @param[in]  c            HAL 上下文。
 * @param[in]  logical_axis 逻辑轴号，取值 [0, 32)。
 * @param[out] out          带出 HalAxisId。
 * @return 0 成功；HAL_ERROR_ARGUMENT 越界或该轴未配置。
 */
int32_t hal_axis_resolve(const HalContext* c, int32_t logical_axis, HalAxisId* out);

/**
 * @brief 可寻址逻辑轴数量（含主轴；装配下标为负的不算）；只读配置，不需要 start。
 *
 * @param[in] c HAL 上下文。
 * @return >= 0 为轴数；负数为错误码取负（-HAL_ERROR_ARGUMENT 表示 c 为空）。
 */
int32_t hal_axis_count(const HalContext* c);

/**
 * @brief 读轴身份（厂商/产品/修订/序列号/设备名/同型号序号）；在 start 认领设备时才填。
 *
 * @param[in]  c   HAL 上下文。
 * @param[in]  id  逻辑轴号。
 * @param[out] out 带出身份信息。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 还没 start；
 *         HAL_ERROR_ARGUMENT 轴号不存在或 out 为空。
 */
int32_t hal_device_identity(const HalContext* c, HalAxisId id, HalCIdentity* out);

/**
 * @brief 查询已配置轴在某功能上的能力；只读 start 的绑定结果，不访问总线。
 *
 * 能力不可用不算错误：返回 0 且 state = HAL_CAP_UNSUPPORTED，reason 里写明缺什么。
 * 判断能不能用要看 state，不能只看返回值。
 *
 * @param[in]  c        HAL 上下文。
 * @param[in]  id       逻辑轴号。
 * @param[in]  function HAL_FUNC_POSITION / HAL_FUNC_SPEED / HAL_FUNC_ERROR_CODE；
 *                      HAL_FUNC_ALARM_CONTROL 归 hal_slave_capability，这里拒绝。
 * @param[out] out      带出能力。
 * @return 0 成功（state 可能仍是 UNSUPPORTED）；HAL_ERROR_NOT_RUNNING 还没 start；
 *         HAL_ERROR_ARGUMENT 轴号或 function 非法。
 */
int32_t hal_axis_capability(const HalContext* c, HalAxisId id, uint32_t function,
                            HalCCapability* out);

/**
 * @brief 从站数量（start 时扫出）；未启动返回负的 HAL_ERROR_NOT_RUNNING。
 *
 * @param[in] c HAL 上下文。
 * @return >= 0 为从站数；负数为错误码取负。
 */
int32_t hal_slave_count(const HalContext* c);

/**
 * @brief 输入/输出映像实际字节数；start 时按各设备实际 Entry 位长算出。
 *
 * x_size 取各 X 段（起始地址 + 段长）的最大值，y_size 同理；调用方按它决定拷贝长度。
 * 未 start 时返回 HAL_ERROR_NOT_RUNNING 且出参不改；并发调用须调用方自己同步。
 *
 * @param[in]  c      HAL 上下文。
 * @param[out] x_size 带出输入映像字节数。
 * @param[out] y_size 带出输出映像字节数。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 还没 start；HAL_ERROR_ARGUMENT 空指针。
 */
int32_t hal_io_image_size(const HalContext* c, uint32_t* x_size, uint32_t* y_size);

/**
 * @brief 取最近一次收帧的总线健康快照；只记录不判断，异常口径由上层定义。
 *
 * 字段逐个从驱动层结构体搬入，不是 memcpy（两边类型与顺序不保证一致）。
 *
 * @param[in]  c   HAL 上下文。
 * @param[out] out 带出健康快照。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 还没 start；HAL_ERROR_BUS 底层查询失败；
 *         HAL_ERROR_ARGUMENT c 或 out 为空。
 */
int32_t hal_bus_health(const HalContext* c, HalCBusHealth* out);

/* 不经过故障门控：返回 3-1-2 错误的 SDK 零基站位，-1 表示未知或不适用。 */
int32_t hal_bus_lost_slave(const HalContext* c, int32_t* slave_pos);

/**
 * @brief 读一个从站的身份信息。
 *
 * @param[in]  c         HAL 上下文。
 * @param[in]  slave_pos 从站位置号，取值 [0, hal_slave_count())。
 * @param[out] out       带出信息。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 还没 start；
 *         HAL_ERROR_ARGUMENT 位置号为负或越界，或 out 为空。
 */
int32_t hal_slave_info(const HalContext* c, int32_t slave_pos, HalCSlaveInfo* out);

/**
 * @brief 查询从站在某功能上的能力；目前只接受 HAL_FUNC_ALARM_CONTROL，恒为
 *        HAL_CAP_NOT_CONFIGURED。判断同样看 state，不看返回值。
 *
 * @param[in]  c         HAL 上下文。
 * @param[in]  slave_pos 从站位置号，取值 [0, hal_slave_count())。
 * @param[in]  function  目前只接受 HAL_FUNC_ALARM_CONTROL。
 * @param[out] out       带出能力。
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 还没 start；
 *         HAL_ERROR_ARGUMENT 位置号越界、function 非 ALARM_CONTROL，或 out 为空。
 */
int32_t hal_slave_capability(const HalContext* c, int32_t slave_pos, uint32_t function,
                             HalCCapability* out);

#ifdef __cplusplus
}
#endif
