#pragma once
/* 适配层：cnc_rt 与 HAL 之间的封装。
 *
 * 线程：主线程 rt_hal_create / start / shutdown；PDO 线程 rt_hal_pdo_loop()（HalContext 的
 * 唯一周期访问者）；其他线程只走"读反馈 / rt_hal_request_* / IO"三组口子，不碰 context。
 *
 * 返回码：0 = 成功；HAL_ERROR_ARGUMENT = 轴槽或出参非法；HAL_ERROR_NOT_RUNNING = 未 create /
 * 未 start / 台账未清；RT_HAL_ERR_ORDER(-5) = 调用顺序错；其余为 HAL 错误码（hal_error_text 转文本）。
 * 参数约定：slot 一律是 RT 轴槽，"主轴槽"指该槽绑定的是主轴；下面省略 @param slot 与"0 = 成功"。 */

#include <stdint.h>
#include "hal_c_api.h"
#include "rt_hal_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 生命周期（主线程）---- */

/* 一根轴的机床参数，由 RT 从自己的参数域取好再传进来 */
typedef struct {
    int              loaded;  /* 0 = 该轴缺参数，保持占位值并挂台账 */
    RtHalAxisScaling scaling; /* loaded 非零时有效 */
    double           max_speed; /* 只主轴用：转速上限 rpm，来自 PAR_SPDL_MAX_SPEED；
                                 * HAL 用它做下发前量程检查；<= 0 = 参数没填，保持占位值 */
} RtHalAxisParams;

/**
 * @brief 装配配置 -> 装载机床参数 -> 创建 context（不碰硬件）
 * 缺参数的轴跳过装载：保持占位值并挂台账，运动被拦住。
 * @param params  按 RtHalAxisSlot 索引，长度 RT_HAL_AXIS_COUNT
 * @param err / err_len 错误文本缓冲，可为空
 * @return HAL_ERROR_BUSY = 已经 create 过；其余为配置校验或 HAL 错误码
 */
int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len);

/**
 * @brief 启动主站并绑定设备（碰硬件，无硬件必失败）；失败后仍须 rt_hal_shutdown() 回收
 * @param err / err_len 错误文本缓冲，可为空
 * @return RT_HAL_ERR_ORDER = 尚未 create；其余为 HAL 错误码
 */
int rt_hal_start(char* err, uint32_t err_len);

/**
 * @brief 停止并销毁 context，可重复调用；未 create 过时是空操作
 * 调用方必须先保证 PDO 线程已退出（本函数不 join）。
 */
void rt_hal_shutdown(void);

/** @brief 是否已 start 且未 shutdown；1 = 运行中 */
int rt_hal_running(void);

/* ---- 周期（仅 PDO 线程）---- */

/**
 * @brief 整圈循环，PDO 线程的唯一入口
 * thread_exit_flag 置位或任一 HAL 调用失败时返回；非零 = 周期异常退出，调用方走停机流程。
 * @return RT_HAL_ERR_ORDER = 未 start；其余为导致退出时 HAL 的错误码
 */
int rt_hal_pdo_loop(void);

/* ---- 反馈快照（任意线程只读）：PDO 线程在 begin 后填一次，本拍内不变 ---- */

/**
 * @brief 读某轴本拍的实际位置（用户单位）
 * @param out 带出实际位置
 * @return HAL_ERROR_NOT_RUNNING = 快照还没采过；HAL_ERROR_ARGUMENT = 槽号或出参非法
 */
int rt_hal_axis_pos(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某轴本拍的完整轴状态（整份拷出）
 * @param out 带出 HalCAxisStatus
 * @return HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus* out);

/**
 * @brief 读某轴本拍的实际速度（单位就是 0x606C 的单位，HAL 不换算）
 * @param out 带出实际速度
 * @return HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO（不是 0，免得上层把"读不到"当成"停着"）；
 *         HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某轴实际速度的 rpm 口径（与 rt_hal_axis_speed 同一份采样，按该轴 speed_unit 换算）
 * 显示与比较一律用这个。
 * @param out 带出 rpm
 * @return HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO（不是 0）；HAL_ERROR_NOT_RUNNING / ARGUMENT
 */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某主轴本拍的转速侧状态（模式、转速、累计转角）；必须是主轴槽
 * @param out 带出 HalCSpindleStatus
 * @return HAL_ERROR_ARGUMENT = 槽号非法或该槽不是主轴（快照的主轴数组只填主轴槽，
 *         读非主轴会拿到"成功 + 0"）；HAL_ERROR_NOT_RUNNING
 */
int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out);

/**
 * @brief 读某主轴本拍的实际转速（rpm）；必须是主轴槽
 * @param out 带出转速
 * @return HAL_ERROR_ARGUMENT = 槽号非法或该槽不是主轴；HAL_ERROR_NOT_RUNNING
 */
int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out);

/**
 * @brief 主轴实际转速（rpm），固定读 RT_HAL_AXIS_SP；给 reset.c 的"主轴停转才能复位"联锁用
 * 失败开放：读不到（或不是主轴槽）返回 0，即判定已停转、联锁放行。
 * @note 与 rt_hal_axis_speed() 相反（那边读不到返回 UNSUPPORTED，不许把"读不到"当"停着"）：
 *       那边是数据接口要如实报告，这边是联锁判据——不放行会导致 HMI 上复位按不动。
 * @return 转速；读不到时 0
 */
double rt_hal_spindle_rpm(void);

/* ---- 主轴定向（厂商专有控制字；只有配了定向的驱动器支持）---- */

/**
 * @brief 请求主轴定向（事件，PDO 线程取走一次）；不受台账阻挡（不产生位移，只让驱动器对准）
 * @return HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT = 槽号非法
 */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot);

/**
 * @brief 读主轴定向进展
 * @param out 带出 HAL_ORIENT_NONE / RUNNING / DONE
 * @return 其余见 HAL 的 hal_rt_spindle_orient_state()
 */
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t* out);

/**
 * @brief 释放定向，把控制字交回普通 DS402 状态机
 * 拿到 DONE 之后必须调用：不释放则驱动器停在厂商状态；释放后轴回到未使能，
 * 要继续动须重新下发运动指令。
 * @return HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_spindle_release_orient(RtHalAxisSlot slot);

/* ---- 命令请求（运动线程写，PDO 线程取走）---- */

/**
 * @brief 请求某轴走到目标位置（用户单位）
 * 按拍暂存，PDO 线程在 commit 前取走；同拍后者覆盖前者，不排队。
 * @param pos 目标位置
 * @return HAL_ERROR_NOT_RUNNING = 未 start 或台账未清（挡在这里）；HAL_ERROR_ARGUMENT = 槽号非法
 */
int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos);

/**
 * @brief 把该轴的当前位置重新定义为 pos（用户单位）—— 设零 / 对刀
 * 只平移软件坐标基准（命令侧与反馈侧一起移），不写驱动器、不动电机，故本身不会让轴动，
 * 也不受未确认台账阻挡。按拍请求，真正的 HAL 调用在 PDO 线程（set_pos 要求已采样过）；
 * 同一拍里先于位置写生效——攻丝起点就是"先设零、再写 0 度保持"。
 * @param pos 新的当前位置（主轴是角度，进给轴是 mm）
 * @return HAL_ERROR_ARGUMENT / HAL_ERROR_NOT_RUNNING
 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos);

/**
 * @brief 请求主轴转到单圈刻度（度）；deg ∈ [-360, 360]，符号定方向，0 取最近零刻度
 * 与 axis_pos 走同一条台账闸门（同样是运动）；同拍两者都置时以本接口为准。
 * 语义（零点、重复命令不再转一圈、±360 转整圈）见 hal_rt_api.h 的 hal_rt_spindle_write_pos()。
 * @param deg 目标刻度（度）
 * @return HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT
 */
int rt_hal_request_spindle_turn(RtHalAxisSlot slot, double deg);

/**
 * @brief 请求主轴以某转速转动
 * @param rpm 目标转速，不许为负
 * @param dir 转向：+1 / -1 / 0
 * @return HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT = 槽号非法或 dir 越出 [-1, 1]
 */
int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir);

/**
 * @brief 请求主轴切到 CSV / CSP；值没变不重复提交，切没切过去看快照的 mode（0x6061）
 * @param mode HAL_SPINDLE_CSV 或 HAL_SPINDLE_CSP
 * @return HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT = 槽号非法或 mode 非法
 */
int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalSpindleMode mode);

/**
 * @brief 声明期望使能状态（状态不是事件：每拍重申，值变化才下发；主轴也用这一个）
 * 不受台账阻挡（台账只关位置、转速、模式）；但故障复位在飞时不下发——HAL 契约是
 * "急停/去使能会取消待复位"，而下发它的正是 RT 自己的停车路径。声明仍记账，
 * 复位收口（DONE/TIMEOUT）后照常下发。
 * @param on 1 = 使能，0 = 去使能（其它值拒绝）
 * @return HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT
 */
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on);

/**
 * @brief 请求该轴急停；不受台账阻挡（安全路径），同拍急停优先（该轴其余请求全部作废）
 * @return HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_request_estop(RtHalAxisSlot slot);

/**
 * @brief 请求故障复位；不受台账阻挡（驱动器已故障时不该被"参数没确认"拦住）
 * 同拍若同时有急停意图，复位被丢弃。
 * @return HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_request_fault_reset(RtHalAxisSlot slot);

/**
 * @brief 读某轴的故障复位进展
 * @param out 带出 HAL_RESET_*（NONE / PENDING / DONE / TIMEOUT）
 * @return HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t* out);

/**
 * @brief 四轴复位是否都已到终态；非零只表示可以收口，不代表成功
 * @param any_timeout 可空；非空时带出"至少有一根超时"
 * @return 1 = 都到终态；0 = 还有轴没收口
 */
int rt_hal_fault_reset_settled(int* any_timeout);

/** @brief 周期循环异常退出的 HAL 错误码；0 = 无 */

int32_t rt_hal_cycle_error(void);

/* ---- IO 映像：PDO 线程 begin 后取 X，PLC 线程 fetch / publish，PDO 线程 commit 前下发；
 * 单缓冲的安全靠 PLC 完成屏障（rt_sem.c）---- */

/**
 * @brief 取本拍输入映像（X）
 * @param out 出参，长度至少 512 字节
 * @param len out 的字节数
 * @return HAL_ERROR_NOT_RUNNING = 本拍没取到输入；HAL_ERROR_ARGUMENT = out 为空或 len 不足 512
 */
int rt_hal_io_fetch_inputs(uint8_t* out, uint32_t len);

/**
 * @brief 交出本拍输出映像（Y），由 PDO 线程在 commit 前下发
 * @param y   要下发的 Y 映像
 * @param len y 的字节数，至少 512
 * @return HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT = y 为空或 len 不足 512
 */
int rt_hal_io_publish_outputs(const uint8_t* y, uint32_t len);

/**
 * @brief 映像实际段长（字节），由 HAL 按各设备装配到的 Entry 位长算出；拷贝按这个长度，不要自己拍数
 * @param x_size 带出 X 段长
 * @param y_size 带出 Y 段长
 * @return HAL_ERROR_NOT_RUNNING = 未 start；HAL_ERROR_ARGUMENT = 出参为空
 */
/* PLC X/Y 映像上限（字节）。三处必须一致：本适配层收发缓冲、threads_controller 本地缓冲、
 * 共享内存 REG_X/REG_Y 长度。启动时拿 HAL 实算段长比，超了拒绝启动——线程层按 HAL 段长
 * 循环写自己的缓冲，段长大过它就会越界写。 */
#define RT_HAL_IO_IMAGE_MAX 512u

int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size);

/**
 * @brief 设置超时停机时要下发的 Y 映像；IO 点表核对清楚之前不要调用
 * 不配置时停机帧不写 Y（保持原状）。
 * @param y   停机映像
 * @param len y 的字节数，至少 512
 * @return HAL_ERROR_ARGUMENT = y 为空或 len 不足 512
 */
int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len);

#ifdef __cplusplus
}
#endif
