#pragma once
/* 适配层：cnc_rt 与 HAL 之间的封装。
 * 线程：main 走 create / start / shutdown；PDO 线程 rt_hal_pdo_loop() 是 HalContext 唯一周期访问者；其他线程只走"读反馈 / rt_hal_request_* / IO"三组口子。
 * 约定：slot 一律是 RT 轴槽（"主轴槽" = 该槽绑定主轴）；err / err_len 为错误文本缓冲，可空。
 * 返回码：0 = 成功；HAL_ERROR_ARGUMENT = 轴槽或出参非法；HAL_ERROR_NOT_RUNNING = 未 create / 未 start / 台账未清；RT_HAL_ERR_ORDER(-5) = 调用顺序错；其余为 HAL 错误码（hal_error_text 转文本）。 */

#include <stdint.h>
#include "hal_c_api.h"
#include "rt_hal_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 生命周期（主线程）---- */

/* 一根轴的机床参数，由 RT 从参数域取好传入 */
typedef struct {
    int              loaded;  /* 0 = 该轴缺参数：保持占位值并挂台账 */
    RtHalAxisScaling scaling; /* 仅 loaded 非零时有效 */
    double           max_speed; /* 仅主轴：转速上限 rpm（PAR_SPDL_MAX_SPEED），HAL 据此做下发前量程检查；<= 0 = 未填，保持占位值 */
} RtHalAxisParams;

/* 装配配置 -> 装载机床参数 -> 创建 context（不碰硬件）。缺参数的轴保持占位值并挂台账（运动被拦）。
 * params 按 RtHalAxisSlot 索引、长度 RT_HAL_AXIS_COUNT；err / err_len 可空。
 * HAL_ERROR_BUSY = 已 create；其余为配置校验或 HAL 错误码。 */
int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len);

/* 启动主站并绑定设备（碰硬件，无硬件必失败）；失败后仍须 rt_hal_shutdown() 回收。
 * RT_HAL_ERR_ORDER = 尚未 create；其余为 HAL 错误码。 */
int rt_hal_start(char* err, uint32_t err_len);

/* 停止并销毁 context，可重复调用；未 create 过时空操作。调用方须先保证 PDO 线程已退出（本函数不 join）。 */
void rt_hal_shutdown(void);

/* 1 = 已 start 且未 shutdown */
int rt_hal_running(void);

/* ---- 周期（仅 PDO 线程）---- */

/* 整圈循环，PDO 线程唯一入口；thread_exit_flag 置位或任一 HAL 调用失败时返回。
 * 非零 = 周期异常退出，调用方走停机流程。RT_HAL_ERR_ORDER = 未 start；其余为导致退出时 HAL 的错误码。 */
int rt_hal_pdo_loop(void);

/* ---- 反馈快照（任意线程只读）：PDO 线程在 begin 后填一次，本拍内不变 ---- */

/* 读某轴本拍实际位置（用户单位）。HAL_ERROR_NOT_RUNNING = 快照还没采过；HAL_ERROR_ARGUMENT = 槽号或出参非法。 */
int rt_hal_axis_pos(RtHalAxisSlot slot, double* out);

/* 读某轴本拍完整轴状态（整份拷出到 HalCAxisStatus）。HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus* out);

/* 读某轴本拍实际速度（0x606C 原始单位，HAL 不换算）。HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO
 * （不是 0，免得"读不到"被当成"停着"）；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out);

/* 读某轴实际速度的 rpm 口径（与 rt_hal_axis_speed 同一份采样，按该轴 speed_unit 换算）；显示与比较一律用这个。
 * HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO（不是 0）；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out);

/* 读某主轴本拍转速侧状态（模式、转速、累计转角）；必须主轴槽。HAL_ERROR_ARGUMENT = 槽号非法或非主轴
 * （快照主轴数组只填主轴槽，读非主轴会拿到"成功 + 0"）；HAL_ERROR_NOT_RUNNING。 */
int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out);

/* 读某主轴本拍实际转速（rpm）；必须主轴槽。HAL_ERROR_ARGUMENT = 槽号非法或非主轴；HAL_ERROR_NOT_RUNNING。 */
int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out);

/* 主轴实际转速（rpm），固定读 RT_HAL_AXIS_SP；给 reset.c 的"主轴停转才能复位"联锁用。
 * 失败开放：读不到或非主轴槽返回 0 = 判定已停转、联锁放行（与 rt_hal_axis_speed 的 UNSUPPORTED 相反）。 */
double rt_hal_spindle_rpm(void);

/* ---- 主轴定向（厂商专有控制字；只有配了定向的驱动器支持）---- */

/* 请求主轴定向（事件，PDO 线程取走一次）；不受台账阻挡（不产生位移，只让驱动器对准）。
 * HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT = 槽号非法。 */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot);

/* 读主轴定向进展；out 带出 HAL_ORIENT_NONE / RUNNING / DONE。其余见 HAL 的 hal_rt_spindle_orient_state()。 */
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t* out);

/* 释放定向，把控制字交回普通 DS402 状态机；拿到 DONE 之后必须调用（不释放则驱动器停在厂商状态）。
 * 释放后轴回到未使能，要继续动须重新下发运动指令。HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_spindle_release_orient(RtHalAxisSlot slot);

/* ---- 命令请求（运动线程写，PDO 线程取走）---- */

/* 请求某轴走到目标位置（用户单位）。按拍暂存，PDO 线程在 commit 前取走；同拍后者覆盖前者，不排队。
 * HAL_ERROR_NOT_RUNNING = 未 start 或台账未清（挡在这里）；HAL_ERROR_ARGUMENT = 槽号非法。 */
int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos);

/* 把该轴当前位置重新定义为 pos（用户单位）—— 设零 / 对刀：只平移软件坐标基准（命令侧与反馈侧一起移），不写驱动器、
 * 不动电机，故不受未确认台账阻挡。按拍请求，HAL 调用在 PDO 线程（set_pos 要求已采样过）；同拍先于位置写生效。
 * pos：主轴角度、进给轴 mm。HAL_ERROR_ARGUMENT / HAL_ERROR_NOT_RUNNING。 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos);
/* 在一次已受理的 set_pos 后，由同一运动线程查询：BUSY=尚未消费，
 * OK=HAL 已成功执行，否则为 HAL 错误。只对应该轴最近一次设零请求。 */
int rt_hal_axis_set_pos_result(RtHalAxisSlot slot);

/* 请求主轴转到单圈刻度（度）：deg ∈ [-360, 360]，符号定方向，0 取最近零刻度；与 axis_pos 同受台账闸门，
 * 同拍两者都置时以本接口为准；语义见 hal_rt_api.h 的 hal_rt_spindle_write_pos()。
 * HAL_ERROR_NOT_RUNNING（含台账未清）/ HAL_ERROR_ARGUMENT。 */
int rt_hal_request_spindle_turn(RtHalAxisSlot slot, double deg);

/* 请求主轴以某转速转动：rpm 不许为负，dir ∈ {+1, -1, 0}。
 * HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT = 槽号非法或 dir 越出 [-1, 1]。 */
int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir);

/* 请求主轴切到 CSV / CSP（HAL_SPINDLE_CSV / HAL_SPINDLE_CSP）；值没变不重复提交，切没切过去看快照 mode（0x6061）。
 * HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT = 槽号非法或 mode 非法。 */
int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalSpindleMode mode);

/* 声明期望使能状态（状态不是事件：每拍重申，值变化才下发；主轴同用这一个）；不受台账阻挡（台账只关位置 / 转速 / 模式）。
 * 但故障复位在飞时不下发——HAL 契约是"急停 / 去使能会取消待复位"，而下发它的正是 RT 停车路径；声明仍记账，复位收口后照常下发。
 * on：1 = 使能，0 = 去使能（其它值拒绝）。HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT。 */
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on);

/* 请求该轴急停；不受台账阻挡（安全路径），同拍急停优先（该轴其余请求全部作废）。HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_request_estop(RtHalAxisSlot slot);

/* 请求故障复位；不受台账阻挡（驱动器已故障时不该被"参数没确认"拦住）；同拍若有急停意图则复位被丢弃。
 * HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_request_fault_reset(RtHalAxisSlot slot);

/* 读某轴故障复位进展；out 带出 HAL_RESET_*（NONE / PENDING / DONE / TIMEOUT）。HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT。 */
int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t* out);

/* 四轴复位是否都已到终态；非零只表示可以收口，不代表成功。any_timeout 可空，非空时带出"至少有一根超时"；
 * 1 = 都到终态，0 = 还有轴没收口。 */
int rt_hal_fault_reset_settled(int* any_timeout);

/* 周期循环异常退出的 HAL 错误码；0 = 无 */

int32_t rt_hal_cycle_error(void);

/* ---- IO 映像：PDO 线程 begin 后取 X，PLC 线程 fetch / publish，PDO 线程 commit 前下发；
 * 单缓冲的安全靠 PLC 完成屏障（rt_sem.c）---- */

/* 取本拍输入映像（X）到 out（len >= 512 字节）。HAL_ERROR_NOT_RUNNING = 本拍没取到输入；HAL_ERROR_ARGUMENT = out 为空或 len 不足 512。 */
int rt_hal_io_fetch_inputs(uint8_t* out, uint32_t len);

/* 交出本拍输出映像（Y，len >= 512），由 PDO 线程在 commit 前下发。
 * HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT = y 为空或 len 不足 512。 */
int rt_hal_io_publish_outputs(const uint8_t* y, uint32_t len);

/* 映像实际段长（字节），由 HAL 按各设备装配到的 Entry 位长算出；拷贝一律按这个长度，不要自己拍数。x_size / y_size 带出段长；
 * HAL_ERROR_NOT_RUNNING = 未 start；HAL_ERROR_ARGUMENT = 出参为空。 */
/* PLC X/Y 映像上限（字节）。三处必须一致：本适配层缓冲、threads_controller 本地缓冲、共享内存 REG_X / REG_Y 长度；
 * 启动时拿 HAL 实算段长比，超了拒绝启动（线程层按 HAL 段长循环写自己缓冲，段长更大即越界写）。 */
#define RT_HAL_IO_IMAGE_MAX 512u

int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size);

/* 设置超时停机时要下发的 Y 映像（不配置时停机帧不写 Y，保持原状）；IO 点表核对清楚之前不要调用。
 * len >= 512 字节；HAL_ERROR_ARGUMENT = y 为空或 len 不足 512。 */
int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len);

#ifdef __cplusplus
}
#endif
