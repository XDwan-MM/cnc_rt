/**
 * @file  hal_config_api.h
 * @brief 公共返回码、错误文本与静态配置校验接口。
 *
 * 所有接口共用的一套返回码定义在本文件；各函数文档里只列本函数特有的码，
 * 其余一律按这里解释。
 */

#pragma once
#include "hal_types.h"
#define HAL_OK                0       /* 成功 */
#define HAL_ERROR_ARGUMENT    0x0009  /* 参数无效：context 为空、轴号不存在、入参越界或非有限值 */
#define HAL_ERROR_ABI         0x000A  /* ABI 不匹配：abi_major/minor、struct_size 或 reserved 与头文件不符 */
#define HAL_ERROR_CONFIG      0x0007  /* 配置无效：静态校验不过，或设备/PDO 绑定失败 */
#define HAL_ERROR_NOT_RUNNING 0x0003  /* 未运行、未采样或该读取所需上下文不可用 */
#define HAL_ERROR_STOPPED     0x000B  /* 已 request_stop，或阻塞等待被主动停止打断。请求之后仍须走 stop/destroy */
#define HAL_ERROR_BUS         0x0101  /* 总线或主站失败。运行期首个错误码被闭锁，此后周期调用持续返回它，直到 stop/start */
#define HAL_ERROR_BUSY        0x0008  /* 主站占用或当前冲突控制动作；不缓存请求 */
#define HAL_ERROR_STATE       0x0001  /* 调用顺序错误，phase 不对：write_* 要在 begin 与 commit 之间；wait 要求上一拍已 commit */
#define HAL_ERROR_MEMORY      0x000C  /* 内存分配失败（context 或内部缓冲） */
#define HAL_ERROR_TIMEOUT     0x0104  /* 启动或本次元操作总预算耗尽；不自动重试 */
#define HAL_ERROR_UNSUPPORTED 0x0105  /* 设备/绑定/当前首批交付策略不支持；不得伪报成功 */
#define HAL_ERROR_NOT_ENABLED   0x0106  /* 轴尚未使能，或状态机还没到位。写指令被拒，等使能/等状态机 */
#define HAL_ERROR_MODE_MISMATCH 0x0107  /* 运行模式未到位：实际或期望模式不是该指令要求的 CSP（写位置）或 CSV（写转速） */
#define HAL_ERROR_SDO         0x0108  /* SDO 应答失败或数据长度不符 */


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 静态校验配置：只看结构体内容，不访问硬件、不启动主站。
 *
 * 检查项包括 ABI 版本闸门、周期与各类超时、设备数量上限、轴绑定唯一性、
 * 换算参数合法性、IO/面板的字节地址范围与从站占用冲突。
 * 出参 err 不论成败都会先被清空，失败时写入中文原因。
 *
 * @param[in]  config  待校验的配置。
 * @param[out] err     错误文本缓冲，可为 NULL。
 * @param[in]  err_len err 的长度；> 0 时始终以 NUL 结尾。
 * @return 0 校验通过；HAL_ERROR_ARGUMENT 配置为空；
 *         HAL_ERROR_ABI 版本闸门不符；HAL_ERROR_CONFIG 其余静态检查未通过。
 */
int32_t hal_config_validate(const HalCConfig* config, char* err, uint32_t err_len);

/**
 * @brief 把返回码翻译成简短中文说明；未知码输出「未知错误」。
 *
 * @param[in]  code    待翻译的返回码。
 * @param[out] out     输出缓冲。
 * @param[in]  out_len 缓冲长度；为 0 时不写。
 */
void hal_error_text(int32_t code, char* out, uint32_t out_len);

#ifdef __cplusplus
}
#endif
