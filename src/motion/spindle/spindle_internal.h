/**
 * @file spindle_internal.h
 * @brief 主轴控制模块私有状态与内部协作接口。
 *
 * 本文件只能被 src/motion/spindle 目录中的实现文件包含。外部模块不得依赖
 * 任何阶段枚举或状态字段，否则会重新形成跨线程直接修改状态的问题。
 */

#ifndef SPINDLE_INTERNAL_H
#define SPINDLE_INTERNAL_H

#include "global.h"
#include "rt_hal_adapter.h"
#include "spindle_control.h"
#include "spindle_velocity_planner.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 每周期最多处理一个已仲裁的主轴事件。 */
typedef enum
{
    SPINDLE_EVENT_NONE = 0,      /**< 当前没有等待控制周期消费的事件。 */
    SPINDLE_EVENT_STOP,          /**< 请求普通受控停车。 */
    SPINDLE_EVENT_CW,            /**< 请求主轴顺时针运行。 */
    SPINDLE_EVENT_CCW,           /**< 请求主轴逆时针运行。 */
    SPINDLE_EVENT_ORIENT,        /**< 请求启动主轴定向流程。 */
    SPINDLE_EVENT_RELEASE_ORIENT /**< 请求释放当前定向流程。 */
} spindle_event_t;

/** 对 PLC 发布的速度完成状态。 */
typedef enum
{
    SPINDLE_VELOCITY_ZERO = 0, /**< 当前目标/运行状态按零速处理。 */
    SPINDLE_VELOCITY_RAMPING,  /**< 规划速度或反馈尚未达到当前目标。 */
    SPINDLE_VELOCITY_ARRIVED   /**< 到速条件成立，可完成对应速度指令。 */
} spindle_velocity_state_t;

/** M19 定向状态机。 */
typedef enum
{
    SPINDLE_ORIENT_IDLE = 0,  /**< 定向模块空闲，没有待执行的定向事务。 */
    SPINDLE_ORIENT_WAIT_ZERO, /**< 等待普通旋转减速至零后再向驱动器发定向请求。 */
    SPINDLE_ORIENT_RUNNING,   /**< 驱动器正在执行定向，等待定向完成或超时。 */
    SPINDLE_ORIENT_DONE,      /**< 驱动器已报告定向完成，等待上层完成握手。 */
    SPINDLE_ORIENT_RELEASING  /**< 正在撤销定向请求并等待驱动器确认释放。 */
} spindle_orient_phase_t;

/** 非 IDLE 定向阶段连续多少拍既无在途动作也无待提交请求时，判定状态不一致并本地收口。
 *  100 ms 远小于 10 s 业务预算，只用于防御"释放时动作尚未提交"一类的不变量破坏。 */
#define SPINDLE_ORIENT_STALL_CYCLES 100U

/** 正常退出定向的受理结果。四态是必需的：IDLE 表示"没有在途动作、无需硬件退出"，
 *  与"已把退出交给 HAL"必须区分，否则调用方会进入永远等不到结果的 RELEASING。
 *  RETRY 表示当前还不能退出（仍在定向中或未停稳，HAL 返回 BUSY），后续周期继续重试，
 *  不能当作失败报警。 */
typedef enum
{
    SPINDLE_ORIENT_RELEASE_IDLE = 0,   /**< 没有在途定向动作；调用方本地收口即可。 */
    SPINDLE_ORIENT_RELEASE_SUBMITTED,  /**< 退出请求已交给 HAL，等待后续快照确认。 */
    SPINDLE_ORIENT_RELEASE_RETRY,      /**< 定向尚未到位或未停稳，稍后重试。 */
    SPINDLE_ORIENT_RELEASE_FAILED      /**< 快照不可读、目标非法或 HAL 明确拒绝；保留定向。 */
} spindle_orient_release_t;

/**
 * M29 状态机。
 *
 * 两种流程在分支专属定向/停车准备后，共用 CSP 切换、坐标对齐及 READY 阶段。
 * FIN 完成后归还 IDLE，CSP 输出由运动模块接管。
 */
typedef enum
{
    SPINDLE_M29_IDLE = 0,            /**< 没有正在执行的 M29 事务。 */
    SPINDLE_M29_ORIENTING,           /**< 普通流程：等待 M19 定向完成。 */
    SPINDLE_M29_ORIENT_SETTLING,     /* 保持定向，确认反馈稳定后再切模式 */
    SPINDLE_M29_NO_ORIENT_WAIT_CSV,  /**< 无定向流程：等待切换到 CSV。 */
    SPINDLE_M29_NO_ORIENT_STOPPING,  /**< 无定向流程：在 CSV 下受控停稳。 */
    SPINDLE_M29_SWITCHING_CSP,       /**< 两种流程共用：等待驱动器切换到 CSP。 */
    SPINDLE_M29_ALIGNING,            /**< 两种流程共用：确认软件偏置及保持目标。 */
    SPINDLE_M29_READY,               /**< CSP 对齐及稳定性检查完成，M29 可完成。 */
    SPINDLE_M29_ERROR,               /**< M29 流程失败或超时，等待外部恢复/复位。 */
    SPINDLE_M29_NO_ORIENT_SWITCHING = SPINDLE_M29_SWITCHING_CSP, /**< 兼容现有诊断名称。 */
    SPINDLE_M29_NO_ORIENT_ALIGNING = SPINDLE_M29_ALIGNING       /**< 兼容现有诊断名称。 */
} spindle_m29_phase_t;

/** 主轴业务等待模式准备的阶段；不是第二份硬件完成状态机。 */
typedef enum
{
    SPINDLE_MODE_SWITCH_IDLE = 0, /**< 没有待确认的模式切换事务。 */
    SPINDLE_MODE_SWITCH_PENDING,  /**< 已提交模式请求，等待 HAL 反馈目标模式。 */
    SPINDLE_MODE_SWITCH_DONE,     /**< HAL 的模式准备已 READY，实际模式确认仍以有效快照为准。 */
    SPINDLE_MODE_SWITCH_ERROR     /**< 模式请求失败；错误码见 mode_switch_result。 */
} spindle_mode_switch_state_t;

/**
 * HAL 未提供直接查询接口的本地输出镜像。
 *
 * target_velocity 和 target_position 只用于显示、完成判定及诊断；驱动器真实
 * 状态始终以本周期 HAL 反馈为准。mode_switch_* 保存异步模式请求的进度。
 */
typedef struct
{
    bool velocity_submitted; /* 本条 CSV 业务已提交目标，后续须检查异步拒绝。 */
    double target_velocity;                        /**< 最近写入/规划的主轴速度目标，单位 rpm；不是实际速度反馈。 */
    double target_position;                        /**< 最近写入的位置目标，单位为主轴 HAL 使用的位置单位。 */
    int requested_mode;                            /**< 最近一次请求切换到的 HAL 主轴模式，如 CSV 或 CSP。 */
    spindle_mode_switch_state_t mode_switch_state; /**< 本条业务等待模式准备的阶段。 */
    int32_t mode_switch_result;                    /**< 最近一次模式切换 HAL 操作的结果码，成功时为 HAL_OK。 */
} spindle_hal_state_t;

/**
 * 主轴控制器的全部可变状态。
 *
 * 字段按职责分组；业务状态由运动线程推进。公开入口并非线程安全队列，
 * 当前 PLC 线程直接更新速度及速度源的交接问题尚未消除。
 */
typedef struct
{
    /* 普通旋转命令与单槽内部事件。 */
    spindle_state_t run_state;               /**< 控制器当前生效的旋转方向/停止状态，由运动周期更新。 */
    spindle_state_t command_state;           /**< 最近仲裁得到的方向命令，等待运行状态机处理。 */
    spindle_velocity_state_t velocity_state; /**< 当前速度指令的零速、斜坡或到速状态。 */
    spindle_event_t pending_event;           /**< 单槽待处理事件；由命令入口写入、控制周期消费。 */
    /* M3/M4 的运行授权与速度目标分离；零速时授权仍可保持驱动器使能。 */
    bool run_authorized; /**< M3/M4 或手动启动授予的运行/使能授权；零速时也可保持为 true。 */
    /* 仅程序 M3/M4 授权屏蔽 PLC 方向边沿，纯手动运行仍可沿用原换向语义。 */
    bool program_run_authorized; /**< 当前授权来自程序 M3/M4；用于屏蔽 PLC 方向边沿对程序方向的干扰。 */

    /* CNC/PLC 速度输入，单位均为 rpm。 */
    double command_speed;                /**< 当前 NC 命令速度，单位 rpm；随新的程序/手动速度命令更新。 */
    double remembered_program_speed;     /**< 最近一次有效程序 S 速度，单位 rpm；供面板启动等场景恢复使用。 */
    double plc_speed;                    /**< PLC 实时给定速度，单位 rpm；由 PLC 接口更新。 */
    spindle_speed_source_t speed_source; /**< 当前速度目标选择源，决定使用 NC 命令还是 PLC 给定。 */

    /* CSV 速度斜坡规划器。 */
    spindle_velocity_planner_t velocity_planner; /**< CSV 速度斜坡规划状态，保存当前规划值及目标等数据。 */

    /* M 代码和模式切换握手。 */
    bool mcode_complete;             /**< 当前 M 代码完成握手状态；完成条件满足后置位。 */
    bool m19_active;                 /**< 当前有尚未完成的 M19 定向命令。 */
    bool m29_active;                 /**< 当前有尚未完成的 M29 刚性攻丝准备事务。 */
    spindle_m29_phase_t m29_phase;   /**< M29 主状态机阶段，供 M29 流程各步骤协同推进。 */
    bool m29_hold_submitted;          /* 设零确认后已提交 CSP 零位置保持请求 */
    double m29_expected_offset;       /* PDO 确认设零成功后的实际偏置，仅作诊断 */
    bool m29_orient_reference_valid;  /* 仅带定向流程校验切模式前后的位置连续性 */
    double m29_orient_reference;      /* 切模式前的反馈位置，不含软件偏置 */
    double m29_settle_min, m29_settle_max; /* 稳定窗口内反馈位置的极值，单位度 */
    uint32_t m29_stop_stable_cycles; /**< M29 停稳判定连续成立的控制周期数，周期为 1 ms。 */
    bool m29_feedback_stable;        /**< CSP 零速和位置误差连续稳定后置位。 */
    bool csv_switch_pending;         /**< 有待处理的 CSV 模式切换；完成/失败后清除。 */

    /* M19 定向及定向期间的命令交接。 */
    bool orient_hold_stop;               /**< 定向事务要求普通旋转保持停止时置位，事务释放后清除。 */
    bool orient_submitted;               /* 本条定向请求已交接，等待状态，不重发 */
    bool orient_exit_submitted;          /* 本条正常退出请求已交接 */
    bool orient_exit_requested;          /**< 业务要求退出，但硬件仍定向中/未停稳，等待重试。 */
    bool orient_request_pending;         /**< 定向硬件请求尚未提交或等待进入可提交阶段。 */
    spindle_orient_phase_t orient_phase; /**< M19 定向状态机当前阶段。 */
    uint32_t orient_elapsed_cycles;      /**< 当前定向/切换阶段已经过的 1 ms 控制周期数，用于超时判定。 */
    uint32_t orient_stall_cycles;        /**< 非 IDLE 阶段既无在途动作也无待提交请求的连续拍数，用于状态自愈。 */

    /* 复位/急停后的启动锁和受控停车状态。 */
    bool reset_requires_start;         /**< 复位/急停后启动锁；清除前须收到新的有效启动请求。 */
    bool csv_stop_pending;             /* 公共复位已把 CSV 减速交给本模块执行 */
    uint32_t csv_stop_cycles;
    uint32_t csv_stop_stable_cycles;

    /* 定向期间收到的启动请求，在释放完成后再执行。 */
    bool start_request_pending;           /**< 有已接收但尚未由旋转状态机处理的启动请求。 */
    bool deferred_start_pending;          /**< 定向期间暂存的启动命令，待释放定向后转为普通启动请求。 */
    spindle_state_t deferred_start_state; /**< 暂存启动请求的方向状态，仅 pending 为 true 时有效。 */
    double deferred_start_speed;          /**< 暂存启动请求的速度，单位 rpm，仅 pending 为 true 时有效。 */
} spindle_context_t;

extern spindle_context_t g_spindle;       /**< 主轴模块唯一的控制状态，由运动控制周期统一推进。 */
extern spindle_hal_state_t g_spindle_hal; /**< 主轴 HAL 输出意愿及异步操作结果的本地镜像。 */

/* HAL 反馈访问与输出意愿。 */
void spindle_hal_init(void);
int spindle_hal_is_enabled(void);
int spindle_hal_snapshot(HalAxisSnapshot* out);
void spindle_hal_poll_mode_switch(void);
int spindle_hal_mode(void);
double spindle_hal_actual_velocity(void);
double spindle_hal_actual_position(void);
uint16_t spindle_hal_status_word(void);
int spindle_hal_orientation_state(void);
bool spindle_hal_orientation_is_inactive(void);

/* 异步 CSV/CSP 模式切换事务。 */
bool spindle_hal_mode_switch_is_pending(void);
bool spindle_hal_mode_switch_is_done(void);
int32_t spindle_hal_mode_switch_result(void);
void spindle_hal_mode_switch_reset(void);
int spindle_hal_request_mode(int mode);

/* M19 定向子模块及其 HAL 命令。 */
bool spindle_orientation_submit(void);
bool spindle_orientation_hal_is_done(void);
/* 四态退出：IDLE=无在途动作，SUBMITTED=已受理，RETRY=等硬件就绪，FAILED=明确拒绝。 */
spindle_orient_release_t spindle_orientation_release(void);
/* 确认没有在途硬件动作后，把定向事务本地收口并执行排队启动/保持零速。 */
void spindle_orientation_complete_release(void);
void spindle_orientation_abort(void);
void spindle_orientation_failed(bool retain_hold, const char *reason);
void spindle_internal_update_orientation(void);

/* 单槽事件的唯一写入口；按注释中的优先级仲裁，禁止各入口直接覆盖槽位。 */
void spindle_post_event(spindle_event_t event);

/* 定向模块回调控制核心执行排队启动和完成反馈。 */
void spindle_internal_handle_rotation(spindle_context_t *context, spindle_state_t state);

/* 安全仲裁和 HMI 复位受控停车。 */
bool spindle_internal_stop_is_required(const spindle_context_t *context);
bool spindle_internal_reset_or_estop_active(void);
bool spindle_internal_reset_stop_abort_required(void);
void spindle_internal_update_csv_stop(void);

/* CSV 速度规划与完成判定。 */
bool spindle_internal_update_velocity(void);
void spindle_internal_fail_velocity(int result);
int spindle_prepare_csv_stop_step(void);
bool spindle_internal_validate_start_request(const spindle_context_t *context);
double spindle_internal_get_selected_speed(const spindle_context_t *context);
void spindle_internal_sync_planner_to_actual(void);
void spindle_internal_finish_stop_if_ready(void);
void spindle_internal_publish_status(void);
void spindle_internal_report_mcode_complete(void);

/* 控制核心提供给 M19/M29/安全停车模块的公共内部操作。 */

/* M29 子状态机，由 1 ms 控制入口按固定顺序推进。 */
bool spindle_internal_m29_no_orient_preparing(void);
void spindle_internal_update_m29_no_orient(void);
void spindle_internal_update_m29(void);
void spindle_internal_publish_rigid_state(void);

/* 只供主轴实现文件协作的内部函数。 */
void spindle_internal_clear_velocity_arrival(void);

#ifdef __cplusplus
}
#endif

#endif /* SPINDLE_INTERNAL_H */
