#pragma once
/* 适配层：cnc_rt 与 HAL 之间的封装。
 *
 * 线程约定：
 *   主线程    rt_hal_create / rt_hal_start / rt_hal_shutdown
 *   PDO 线程  rt_hal_pdo_loop()，HalContext 的唯一周期访问者
 *   其他线程  只能走读反馈 / rt_hal_request_* / IO 这三组口子，一律不碰 context
 *
 * 公共返回码：0 = 成功；HAL_ERROR_ARGUMENT = 轴槽或出参非法；
 * HAL_ERROR_NOT_RUNNING = 未 create / 未 start / 台账未清；其余为 HAL 的错误码，
 * 可用 hal_error_text() 转文本。另有一个 RT 自己的 RT_HAL_ERR_ORDER（-5，调用顺序错）。 */

#include <stdint.h>
#include "hal_c_api.h"
#include "rt_hal_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 生命周期（主线程）---- */

/**
 * @brief 一根轴的机床参数，由 RT 从自己的参数域取好再传进来
 */
typedef struct {
    int              loaded;  /* 0 = 该轴缺参数，保持占位值并挂台账 */
    RtHalAxisScaling scaling; /* loaded 非零时有效 */
    double           max_speed; /* **只主轴用**：转速上限 rpm，来自参数域 PAR_SPDL_MAX_SPEED。
                                 * HAL 拿它做下发前的量程检查；<= 0 = 参数里没填，保持占位值 */
} RtHalAxisParams;

/**
 * @brief 装配配置 → 装载机床参数 → 创建 context（不碰硬件）
 *
 * 缺参数的轴跳过装载：保持占位值并挂台账，运动会被拦住。
 *
 * @param params  按 RtHalAxisSlot 索引的机床参数，长度 RT_HAL_AXIS_COUNT
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度
 * @return int 0 = 成功；HAL_ERROR_BUSY = 已经 create 过；其余为配置校验或 HAL 的错误码
 */
int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len);

/**
 * @brief 启动主站并绑定设备（会碰硬件，没有硬件必然失败）
 *
 * 失败后仍须调 rt_hal_shutdown() 回收。
 *
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度
 * @return int 0 = 成功；RT_HAL_ERR_ORDER = 尚未 create；其余为 HAL 的错误码
 */
int rt_hal_start(char* err, uint32_t err_len);

/**
 * @brief 停止并销毁 context，可重复调用；未 create 过时是空操作
 *
 * **调用方必须先保证 PDO 线程已退出**——本函数不负责 join。
 */
void rt_hal_shutdown(void);

/**
 * @brief 是否已 start 且未 shutdown
 *
 * @return int 1 = 运行中；0 = 未运行
 */
int rt_hal_running(void);

/* ---- 周期（仅 PDO 线程）---- */

/**
 * @brief 整圈循环，PDO 线程的唯一入口
 *
 * thread_exit_flag 置位或任一 HAL 调用失败时返回。返回非零表示周期异常退出，
 * 调用方应走停机流程。
 *
 * @return int 0 = 正常退出（thread_exit_flag）；RT_HAL_ERR_ORDER = 未 start；
 *             其余为导致退出时 HAL 的错误码
 */
int rt_hal_pdo_loop(void);

/* ---- 反馈快照（任意线程只读）----
 * PDO 线程在 begin 后填一次，本拍内不变。入参一律是 RT 轴槽。 */

/**
 * @brief 读某轴本拍的实际位置（用户单位）
 *
 * @param slot RT 轴槽
 * @param out  带出实际位置
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING = 快照还没采过；
 *             HAL_ERROR_ARGUMENT = 槽号或出参非法
 */
int rt_hal_axis_pos(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某轴本拍的完整轴状态（整份拷出）
 *
 * @param slot RT 轴槽
 * @param out  带出 HalCAxisStatus
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus* out);

/**
 * @brief 读某轴本拍的实际速度
 *
 * **单位就是 0x606C 的单位，HAL 不换算**——上机必须核口径。
 *
 * @param slot RT 轴槽
 * @param out  带出实际速度
 * @return int 0 = 成功；HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO（**不是 0**，
 *             免得上层把"读不到"当成"停着"）；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某轴实际速度的 **rpm 口径**（同一份读数的另一种单位）
 *
 * 与 rt_hal_axis_speed() 同一份采样，区别只在于单位：那个给驱动器原始值（0x606C
 * 自己的单位），这个给按该轴 speed_unit 换算好的 rpm。**显示与比较一律用这个**。
 *
 * @param slot RT 轴槽
 * @param out  带出 rpm
 * @return int 0 = 成功；HAL_ERROR_UNSUPPORTED = 该轴没映射速度 PDO（**不是 0**）；
 *             HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out);

/**
 * @brief 读某主轴本拍的转速侧状态（模式、转速、累计转角）
 *
 * @param slot RT 轴槽，**必须是主轴槽**
 * @param out  带出 HalCSpindleStatus
 * @return int 0 = 成功；HAL_ERROR_ARGUMENT = 槽号非法**或该槽不是主轴**
 *             （快照的主轴数组只填主轴槽，读非主轴会拿到"成功 + 0"）；
 *             HAL_ERROR_NOT_RUNNING
 */
int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out);

/**
 * @brief 读某主轴本拍的实际转速（rpm）
 *
 * @param slot RT 轴槽，**必须是主轴槽**
 * @param out  带出转速
 * @return int 0 = 成功；HAL_ERROR_ARGUMENT = 槽号非法**或该槽不是主轴**；
 *             HAL_ERROR_NOT_RUNNING
 */
int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out);

/**
 * @brief 主轴实际转速（rpm）
 *
 * 给 reset.c 的"主轴必须停转才能复位"联锁用。固定读 RT_HAL_AXIS_SP。
 *
 * **失败开放**：读不到（或该槽不是主轴）时返回 0，即判定为已停转、联锁放行。
 *
 * @note 这与 rt_hal_axis_speed() **正好相反**——那边读不到返回
 *       HAL_ERROR_UNSUPPORTED，不许上层把"读不到"当成"停着"。差别在用途：
 *       那边是**数据接口**（要如实报告），这边是**联锁判据**（读不到就不放行的话，
 *       HMI 上表现为复位永远按不动）。改这条判据前先想清是安全功能还是操作便利。
 *
 * @return double 转速；读不到时 0
 */
double rt_hal_spindle_rpm(void);

/* ---- 主轴定向（厂商专有控制字；只有配了定向的驱动器支持）----*/

/**
 * @brief 请求主轴定向
 *
 * **不受台账阻挡**——定向不产生位移，只是让驱动器对准。请求是事件，PDO 线程取走一次。
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @return int 0 = 已记录；HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT = 槽号非法
 */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot);

/**
 * @brief 读主轴定向进展
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @param out  带出 HAL_ORIENT_NONE / HAL_ORIENT_RUNNING / HAL_ORIENT_DONE
 * @return int 0 = 成功；其余见 HAL 的 hal_rt_spindle_orient_state()
 */
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t* out);

/**
 * @brief 释放定向，把控制字交回普通 DS402 状态机
 *
 * ！！！！在rt_hal_spindle_orient_state拿到 DONE 之后必须调它！！！！，
 * 否则驱动器会一直停在厂商状态。释放后轴回到未使能，
 * 要继续动得重新下发运动指令。
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @return int 0 = 已记录；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_spindle_release_orient(RtHalAxisSlot slot);

/* ---- 命令请求（运动线程写，PDO 线程取走）---- */

/**
 * @brief 请求某轴走到目标位置（用户单位）
 *
 * 按拍暂存，PDO 线程在 commit 前取走；**同拍后者覆盖前者，不排队**。
 *
 * @param slot RT 轴槽
 * @param pos  目标位置
 * @return int 0 = 暂存成功；HAL_ERROR_NOT_RUNNING = 未 start，或**台账未清**（挡在这里）；
 *             HAL_ERROR_ARGUMENT = 槽号非法
 */
int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos);

/**
 * @brief 把该轴的当前位置重新定义为 pos（用户单位）——设零 / 对刀
 *
 * 只平移软件坐标基准（命令侧与反馈侧一起移），**不写驱动器、不动电机**，
 * 所以调用它本身不会让轴动，也**不受未确认台账阻挡**。
 * 按拍请求：真正的 HAL 调用在 PDO 线程里完成（set_pos 要求已经采样过）。
 * 同一拍里它先于位置写生效 —— 攻丝起点就是"先设零、再写 0° 保持"。
 *
 * @param slot RT 轴槽
 * @param pos  新的当前位置（主轴是角度，进给轴是 mm）
 * @return int 0 = 成功；HAL_ERROR_ARGUMENT / HAL_ERROR_NOT_RUNNING
 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos);

/**
 * @brief 请求主轴转到单圈刻度（度）
 *
 * deg ∈ [-360, 360]，符号定方向；0 取最近的零刻度。与 axis_pos 走**同一条台账闸门**
 * （它同样是运动）；**同拍两者都置时以本接口为准**。语义（零点、重复命令不再转一圈、
 * ±360 转整圈）见 hal_rt_api.h 的 hal_rt_spindle_write_pos()。
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @param deg  目标刻度（度）
 * @return int 0 = 暂存成功；HAL_ERROR_NOT_RUNNING（含台账未清）；HAL_ERROR_ARGUMENT
 */
int rt_hal_request_spindle_turn(RtHalAxisSlot slot, double deg);

/**
 * @brief 请求主轴以某转速转动
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @param rpm  目标转速，**不许为负**
 * @param dir  转向：+1 / -1 / 0
 * @return int 0 = 暂存成功；HAL_ERROR_NOT_RUNNING（含台账未清）；
 *             HAL_ERROR_ARGUMENT = 槽号非法，或 dir 越出 [-1, 1]
 */
int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir);

/**
 * @brief 请求主轴切到 CSV / CSP
 *
 * 值没变就不重复提交。切没切过去由快照的 mode（驱动器 0x6061）反映。
 *
 * @param slot RT 轴槽，应当是主轴槽
 * @param mode HAL_SPINDLE_CSV 或 HAL_SPINDLE_CSP
 * @return int 0 = 暂存成功；HAL_ERROR_NOT_RUNNING（含台账未清）；
 *             HAL_ERROR_ARGUMENT = 槽号非法，或 mode 非法
 */
int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalSpindleMode mode);

/**
 * @brief 声明期望使能状态
 *
 * **是状态不是事件**：每拍重新声明，适配层只在值变化时下发。主轴也用这一个。
 * **不受台账阻挡**——台账关的是位置、转速、模式。
 * **但故障复位在飞时不下发**：HAL 的契约是"急停/去使能会取消待复位"，
 * 而下发它的恰恰是 RT 自己的停车路径。声明仍然记账，复位收口（DONE/TIMEOUT）后照常下发。
 *
 * @param slot RT 轴槽
 * @param on   1 = 使能，0 = 去使能（其它值拒绝）
 * @return int 0 = 已记录；HAL_ERROR_NOT_RUNNING = 未 create；HAL_ERROR_ARGUMENT
 */
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on);

/**
 * @brief 请求该轴急停
 *
 * **不受台账阻挡**（安全路径）。同拍内急停优先：有急停意图时该轴其余请求全部作废。
 *
 * @param slot RT 轴槽
 * @return int 0 = 已记录；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_request_estop(RtHalAxisSlot slot);

/**
 * @brief 请求故障复位
 *
 * **不受台账阻挡**——驱动器已故障时不该被"参数没确认"拦住。同拍内若同时有急停意图，
 * 复位会被丢弃。
 *
 * @param slot RT 轴槽
 * @return int 0 = 已记录；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_request_fault_reset(RtHalAxisSlot slot);

/**
 * @brief 读某轴的故障复位进展
 *
 * @param slot RT 轴槽
 * @param out  带出 HAL_RESET_*（NONE / PENDING / DONE / TIMEOUT）
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING / HAL_ERROR_ARGUMENT
 */
int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t* out);

/**
 * @brief 四轴复位是否都已到终态
 *
 * 返回非零**只表示可以收口，不代表成功**。
 *
 * @param any_timeout 可空；非空时带出"至少有一根超时"
 * @return int 1 = 都到终态；0 = 还有轴没收口
 */
int rt_hal_fault_reset_settled(int* any_timeout);

/**
 * @brief 周期循环异常退出的 HAL 错误码
 *
 * @return int32_t 0 = 无；非零 = 退出原因（HAL 的错误码）
 */
int32_t rt_hal_cycle_error(void);

/* ---- IO 映像 ----
 * PDO 线程 begin 后取 X；PLC 线程 fetch / publish；PDO 线程 commit 前下发。
 * 单缓冲能安全靠 PLC 完成屏障（rt_sem.c）。 */

/**
 * @brief 取本拍输入映像（X）
 *
 * @param out 出参，长度至少 512 字节
 * @param len out 的字节数
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING = 本拍没取到输入；
 *             HAL_ERROR_ARGUMENT = out 为空或 len 不足 512
 */
int rt_hal_io_fetch_inputs(uint8_t* out, uint32_t len);

/**
 * @brief 交出本拍输出映像（Y），由 PDO 线程在 commit 前下发
 *
 * @param y   要下发的 Y 映像
 * @param len y 的字节数，至少 512
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING = 未 create；
 *             HAL_ERROR_ARGUMENT = y 为空或 len 不足 512
 */
int rt_hal_io_publish_outputs(const uint8_t* y, uint32_t len);

/**
 * @brief 映像实际段长（字节）
 *
 * 由 HAL 按各设备装配到的 Entry 位长算出。拷贝时按这个长度，
 * **不要自己拍数**——拍错了是静默错位。
 *
 * @param x_size 带出 X 段长
 * @param y_size 带出 Y 段长
 * @return int 0 = 成功；HAL_ERROR_NOT_RUNNING = 未 start；HAL_ERROR_ARGUMENT = 出参为空
 */
int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size);

/**
 * @brief 设置超时停机时要下发的 Y 映像
 *
 * **IO 点表核对清楚之前不要调用**。不配置时停机帧不写 Y（保持原状）。
 *
 * @param y   停机映像
 * @param len y 的字节数，至少 512
 * @return int 0 = 成功；HAL_ERROR_ARGUMENT = y 为空或 len 不足 512
 */
int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len);

#ifdef __cplusplus
}
#endif
