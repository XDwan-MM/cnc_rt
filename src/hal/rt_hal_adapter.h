#pragma once
/*
 * 适配层：cnc_rt 与 cnc_hal 之间的唯一通道。
 *
 * 这一层只做四件事：
 *   1. 把 RT 轴槽号（X/Y/Z/SP）换算成 HAL 逻辑轴号；
 *   2. 把 HAL 的周期快照转成业务可直接读的视图；
 *   3. 把业务请求按拍暂存，交给 PDO 线程在正确时机下发；
 *   4. 把 HAL 错误码翻译成 RT 侧可上报的形式。
 * 它不做业务判断，也不碰 DS402 状态机——那是 HAL 的职责。
 *
 * 线程模型（违反即数据竞争）：
 *   主线程    —— create / start / shutdown，以及 SDO 工作线程的生命周期
 *   PDO 线程  —— rt_hal_pdo_loop() 是 HalContext 唯一的周期访问者
 *   运动线程  —— 本拍读快照、写请求暂存区；自身不调用 HAL 周期接口
 *   PLC 线程  —— 只通过 fetch/publish 交接 IO 映像
 *   非周期接口（SDO、快照查询）的并发由调用方自行保证。
 *
 * 槽号约定：
 *   slot 一律是 RT 轴槽（RT_HAL_AXIS_X/Y/Z/SP），既不是 HAL 逻辑轴号，
 *   也不是装配下标。到 HAL 轴号的映射在适配层内部完成。
 *
 * 返回码约定：
 *   0                    成功
 *   HAL_ERROR_ARGUMENT   槽号、出参或入参非法
 *   HAL_ERROR_NOT_RUNNING 未 create、未 start，或台账未清（运动请求被挡在这里）
 *   RT_HAL_ERR_ORDER     调用顺序错（例如未 create 就 start）
 *   其余                 透传 HAL 错误码，可用 hal_error_text() 转成中文
 *
 * 请求语义（容易误解，务必先读）：
 *   所有 rt_hal_request_* 与 rt_hal_axis_set_pos() 返回 HAL_OK，只表示
 *   "已放进本拍暂存区"，不表示已下发，更不表示硬件已完成。被拒绝的原因
 *   写进下一拍快照的 request_rejection，由业务在下一拍自查。
 */

#include <stdint.h>
#include "hal_c_api.h"
#include "rt_hal_config.h"
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 一、状态视图
 *
 * 这两个结构是给既有消费者用的"薄视图"：字段全部由 HAL 快照现算，
 * 适配层不额外缓存，因此不会出现"视图与快照不一致"。
 * 配套的 *_valid 标志用来区分"值可用"与"值恰好是 0"——读不到能力时
 * 不能把 0 当成真实反馈。
 * ========================================================================== */

typedef struct {
    int32_t enabled, position_valid;
    double actual_pos, command_pos, actual_speed, actual_speed_rpm;
    int32_t speed_valid;
    uint16_t raw_status;
    uint32_t error_code;
    int32_t error_code_valid;
    double actual_speed_mm_min;
    int32_t linear_speed_valid;
} RtHalAxisStatus;
typedef struct {
    int32_t enabled, mode;
    double command_speed, actual_speed, position_deg;
    uint16_t raw_status;
} RtHalSpindleStatus;

/* 历史遗留的复位/定向查询结果。两者都只是快照的投影：
 * 既不代表 HAL 又收到了新请求，也不推进任何硬件状态。 */
#define RT_HAL_RESET_NONE 0
#define RT_HAL_RESET_PENDING 1
#define RT_HAL_RESET_DONE 2
#define RT_HAL_RESET_FAILED 3
#define RT_HAL_ORIENT_NONE 0
#define RT_HAL_ORIENT_RUNNING 1
#define RT_HAL_ORIENT_DONE 2
#define RT_HAL_ORIENT_UNKNOWN 3

/* 主状态入口：读某轴本拍快照。不可用的反馈保持不可用（返回错误码），
 * 读失败不会清除任何已记录的问题。 */
int rt_hal_axis_snapshot(RtHalAxisSlot slot, HalAxisSnapshot* out);
int rt_hal_request_axis_mode(RtHalAxisSlot slot, HalAxisMode mode);
int rt_hal_clear_execution_error(RtHalAxisSlot slot);
int rt_hal_spindle_exit_orient(RtHalAxisSlot slot, const HalOrientExitTarget* target);

/* ==========================================================================
 * 二、生命周期（仅主线程）
 * ========================================================================== */

/* 一根轴的机床参数。RT 从参数域取好后传入，适配层不自己读参数。 */
typedef struct {
    int              loaded;  /* 0 = 该轴参数缺失：保留占位换算量并记入台账，运动被拦 */
    RtHalAxisScaling scaling; /* 仅在 loaded 非零时生效 */
    double           max_speed; /* 仅主轴使用：转速上限 rpm（来自 PAR_SPDL_MAX_SPEED）。
                                 * HAL 在每次下发转速前用它做量程检查。<= 0 表示未填，
                                 * 保留占位值，超过占位上限的转速会被 HAL 拒收。 */
} RtHalAxisParams;

/* 创建 context：装配配置 → 装载机床参数 → 校验，全程不碰硬件。
 *
 * 缺参数的轴不会被 set_scaling，而是保留占位换算量并记入未确认台账——
 * 台账未清时运动请求一律被拒（见 rt_hal_config_motion_allowed）。
 *
 * @param params 按 RtHalAxisSlot 索引，长度必须是 RT_HAL_AXIS_COUNT
 * @param err    错误文本缓冲，可为空
 * @return 0 成功；HAL_ERROR_BUSY 已经 create 过；其余为配置校验或 HAL 错误码 */
int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len);

/* 启动主站并认领设备。这一步会真正碰硬件，无硬件必然失败。
 * 失败后仍须调用 rt_hal_shutdown() 回收已分配的 context。
 *
 * @return 0 成功；RT_HAL_ERR_ORDER 尚未 create；其余为 HAL 错误码 */
int rt_hal_start(char* err, uint32_t err_len);

/* 停止并销毁 context。可重复调用，未 create 过时为无操作。
 * 本函数不 join PDO 线程：调用方必须先确认周期循环已经退出。 */
void rt_hal_shutdown(void);

/* 是否处于"已 start 且未 shutdown"状态。 */
int rt_hal_running(void);

/* SDO 同步传输。这是阻塞调用，只允许由 rt_hal_sdo_worker 使用；
 * 业务侧应通过 rt_hal_sdo.h 提交请求再取结果，不得在 PDO/运动周期线程里直接调。 */
int rt_hal_axis_sdo_read(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                         void* data, uint32_t size, uint32_t timeout_ms);
int rt_hal_axis_sdo_write(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                          const void* data, uint32_t size, uint32_t timeout_ms);
/* 物理从站寻址版本，只用于非轴设备或维护场合。 */
int rt_hal_sdo_read(int slave_pos, uint16_t index, uint8_t subindex,
                    void* data, uint32_t size, uint32_t timeout_ms);
int rt_hal_sdo_write(int slave_pos, uint16_t index, uint8_t subindex,
                     const void* data, uint32_t size, uint32_t timeout_ms);

/* ==========================================================================
 * 三、周期循环（仅 PDO 线程）
 * ========================================================================== */

/* PDO 线程的唯一入口，跑完整圈循环：等节拍 → 采样 → 放行业务线程 → 取请求 → 提交。
 *
 * 返回意味着循环已结束：thread_exit_flag 置位（正常停机），或任一 HAL 调用失败
 * （异常退出，调用方须走停机流程）。异常退出的错误码同时记录在
 * rt_hal_cycle_error() 里，供 RT 侧报警使用。
 *
 * @return 0 正常退出；RT_HAL_ERR_ORDER 未 start；其余为导致退出时的 HAL 错误码 */
int rt_hal_pdo_loop(void);

/* ==========================================================================
 * 四、反馈快照（业务周期内只读）
 *
 * PDO 线程在 begin 成功后填一次，本拍内不再变化，因此同一拍里多次读到的
 * 值完全一致，不会跨越采样边界。快照没建立之前所有读接口返回
 * HAL_ERROR_NOT_RUNNING，不会给出 0 或旧值。
 * ========================================================================== */

/* 本拍实际位置（用户单位）。 */
int rt_hal_axis_pos(RtHalAxisSlot slot, double* out);

/* 本拍完整轴状态，整份拷出到 RtHalAxisStatus。 */
int rt_hal_axis_status(RtHalAxisSlot slot, RtHalAxisStatus* out);

/* 本拍实际速度，单位是 0x606C 的原始设备单位，HAL 不做换算。
 * 该轴没映射速度 PDO 时返回 HAL_ERROR_UNSUPPORTED 而不是 0——
 * 避免"读不到"被误判成"停着"。 */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out);

/* 与 rt_hal_axis_speed 同一份采样，按该轴 speed_unit 换算成 rpm。
 * 显示与比较一律用这个口径，不要自己再乘除。 */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out);

/* 进给轴本拍实际线速度（mm/min）。主轴槽、缺速度 PDO 或缺换算参数时
 * 返回 HAL_ERROR_UNSUPPORTED。 */
int rt_hal_axis_speed_mm_min(RtHalAxisSlot slot, double* out);

/* 主轴本拍转速侧状态（模式、转速、累计转角）。必须传主轴槽：
 * 快照的主轴数组只填主轴槽，传进给轴会拿到"成功 + 全 0"。 */
int rt_hal_spindle_status(RtHalAxisSlot slot, RtHalSpindleStatus* out);

/* 主轴本拍实际转速（rpm）。必须传主轴槽。 */
int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out);

/* 固定读 RT_HAL_AXIS_SP 的实际转速（rpm），供 reset.c 的
 * "主轴停转才允许复位"联锁使用。读取失败返回 NaN，
 * 绝不用一个未知值去证明"已经停转"。 */
double rt_hal_spindle_rpm(void);

/* ==========================================================================
 * 五、主轴定向（厂商专有控制字，只有配了定向的驱动器支持）
 * ========================================================================== */

/* 请求主轴定向。这是一次性事件，PDO 线程取走一次即消费。
 * 不受未确认台账阻挡：定向不产生位移，只是让驱动器对准。
 *
 * 正常退出定向需要提供保持目标，H1 版本由 HAL 明确拒绝（UNSUPPORTED），
 * 适配层不会伪造成"已退出"。 */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot);

/* 定向进展的兼容视图：从统一 control_state 派生，不缓存进展。 */
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t* out);

/* ==========================================================================
 * 六、命令请求（仅运动线程写，PDO 线程在业务屏障之后取走）
 *
 * 写入只是"按拍暂存"：同拍内后写覆盖先写，不排队。真正的 HAL 调用发生在
 * PDO 线程的 take_requests()。返回 HAL_OK 仅代表暂存成功；拒绝原因要等
 * 下一拍快照的 request_rejection 才看得到。
 * ========================================================================== */

/* 请求某轴走到目标位置（用户单位）。
 * 未 start 或台账未清时在这里就被挡住，不会进暂存区。 */
int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos);

/* 把该轴当前位置重新定义为 pos（用户单位），用于设零/对刀。
 *
 * 只平移软件坐标基准（命令侧与反馈侧一起移），不写驱动器、不动电机，
 * 因此不受未确认台账阻挡。同一个 HAL 调用要求已经采样过，所以真正的
 * set_pos 在 PDO 线程执行，且同拍内先于位置写生效。
 * pos 的单位：主轴为角度，进给轴为 mm。 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos);

/* 单槽设零回执：BUSY 表示未执行，OK 时 result 为 HAL 实际结果（也可能失败）。
 * 只供运动线程在周期屏障内查询；下一次受理的设零请求会覆盖旧回执。 */
int rt_hal_axis_set_pos_result(RtHalAxisSlot slot, int32_t* result);

/* 请求主轴以某转速转动。rpm 不许为负，方向由 dir（+1/-1/0）给出，
 * 两者分开表达是为了让"停止"和"极低速"不混为一谈。 */
int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir);
/* 带回执的速度请求：同拍覆盖共用回执，取消返回 STOPPED。下一拍查询。 */
int rt_hal_request_spindle_speed_tracked(RtHalAxisSlot slot, double rpm, int dir, uint32_t* request_id);
int rt_hal_spindle_speed_result(RtHalAxisSlot slot, uint32_t request_id, int32_t* result);

/* 一次性的主轴模式准备请求。
 *
 * 本函数只发起准备；是否准备好要看快照的 mode_setting_state，
 * 使能需要另行显式请求，运动则要等实际模式与 OE 都到位。 */
int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalAxisMode mode);

/* 一次性的上/下使能请求。
 *
 * 适配层不保存失败请求，也不自动重试——失败就是失败，由业务决定下一步。
 * 同一拍内下使能优先于上使能（后来的上使能会被判 BUSY）。
 * 模式准备与重新使能是两个独立请求，不会因为 enable 而顺带发生。 */
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on);

/* 请求该轴急停。安全路径，不受台账阻挡；同拍内急停优先，
 * 该轴其余待发请求全部作废。 */
int rt_hal_request_estop(RtHalAxisSlot slot);

/* 请求故障复位。不受台账阻挡——驱动器已经故障时，不该被"参数没确认"拦住。
 * 同拍内若已有急停意图，复位会被丢弃。 */
int rt_hal_request_fault_reset(RtHalAxisSlot slot);

/* 复位进展的兼容视图：从当前控制/驱动状态与诊断派生 RT_HAL_RESET_*，
 * 自身不维护硬件进展。 */
int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t* out);

/* 四轴复位是否都可以收口。
 * 注意返回值语义：1 表示"都已到终态"，并不表示四轴都成功——某轴失败也算到终态，
 * 此时通过 any_timeout 带出。
 *
 * @param any_timeout 可为空；非空时带出"至少有一轴失败"
 * @return 1 全部收口（可能有失败）；0 还有轴在等待 */
int rt_hal_fault_reset_settled(int* any_timeout);
/* 只等待 mask 中提交过故障复位的轴；其他轴可以继续保持使能。 */
int rt_hal_fault_reset_settled_mask(uint32_t mask, int* any_timeout);

/* 周期循环异常退出时记录的 HAL 错误码；0 表示没有错误。 */
int32_t rt_hal_cycle_error(void);

/* ==========================================================================
 * 七、IO 映像
 *
 * 单缓冲：PDO 线程 begin 之后取 X，PLC 线程 fetch 读、publish 写，
 * PDO 线程在 commit 之前下发 Y。中间的安全靠 PLC 完成屏障（rt_sem.c）保证，
 * 不要在这里再加锁。
 * ========================================================================== */

/* 取本拍输入映像（X）。len 必须 >= RT_HAL_IO_IMAGE_MAX。
 * 本拍没取到输入时返回 HAL_ERROR_NOT_RUNNING，不会给出上一次的内容。 */
int rt_hal_io_fetch_inputs(uint8_t* out, uint32_t len);

/* 交出本拍输出映像（Y），由 PDO 线程在 commit 前下发。
 * len 必须 >= RT_HAL_IO_IMAGE_MAX。 */
int rt_hal_io_publish_outputs(const uint8_t* y, uint32_t len);

/* PLC X/Y 映像上限（字节）。
 *
 * 这个值必须在三处保持一致：本适配层缓冲、threads_controller 的本地缓冲、
 * 共享内存里 REG_X / REG_Y 的长度。启动时会把 HAL 按设备 Entry 实算出的段长
 * 与它比较，超出就拒绝启动——因为线程层是按 HAL 段长写自己缓冲的，
 * 段长更大就会越界写。 */
#define RT_HAL_IO_IMAGE_MAX 512u

/* 取映像实际段长（字节）。段长由 HAL 按各设备实际装配到的 Entry 位长算出，
 * 拷贝一律按这个长度，不要自己拍数。
 *
 * @return 0 成功；HAL_ERROR_NOT_RUNNING 未 start；HAL_ERROR_ARGUMENT 出参为空 */
int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size);

/* 设置超时停机时要下发的 Y 映像。不设置时停机帧不写 Y，保持原电平。
 * IO 点表核对清楚之前不要调用——写错停机输出比不写更危险。
 * len 必须 >= RT_HAL_IO_IMAGE_MAX。 */
int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len);

/* 设置主轴定向位置的原始脉冲值（不做角度换算）。
 * slot 仍然是 RT 轴槽，到 HAL 轴号的映射在内部完成。
 * 这是阻塞 SDO 调用，只能在非实时线程使用。 */
int rt_hal_spindle_set_orient_pulses(RtHalAxisSlot slot, uint32_t pulses, uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif
