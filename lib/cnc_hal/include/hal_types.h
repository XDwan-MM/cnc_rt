#pragma once
#include <stdint.h>
#define HAL_C_ABI_MAJOR 10u
#define HAL_C_ABI_MINOR 0u
#define HAL_VERSION_MAJOR 0u
#define HAL_VERSION_MINOR 2u
#define HAL_VERSION_PATCH 0u
#define HAL_VERSION_STRING "0.2.0"

#define HAL_C_MAX_DEV     30u   /* 轴 / IO / 面板 各自的容量上限 */
#define HAL_C_MAX_SPINDLE 4u    /* 主轴容量上限 */
#define HAL_C_NAME_LEN    64u   /* 定长名称缓冲长度；字符串一律定长数组（见文件头） */
#define HAL_INVALID_ID    UINT16_MAX  /* 无效 HalAxisId 的哨兵；0 是合法轴号 */

/* ---- 不透明句柄 ---- */
typedef struct HalContext HalContext;

/* ---- 设备标识 ----*/
typedef uint16_t HalAxisId;      /* 就是逻辑轴号本身，不是装配下标 */

/* 按轴实例查询的功能码 */
#define HAL_FUNC_POSITION       1u  /* 位置：0x607A 目标位置 + 0x6064 实际位置 */
#define HAL_FUNC_SPEED          2u  /* 速度：0x60FF 目标转速 + 0x606C 实际转速（两个都要有） */
#define HAL_FUNC_ERROR_CODE     3u  /* 故障码 0x603F */
#define HAL_FUNC_ALARM_CONTROL  4u  /* 厂商专有报警控制 0x6FFF；当前无设备定义，恒不可用 */
#define HAL_CAP_READY           1u  /* 该功能可用 */
#define HAL_CAP_UNSUPPORTED     2u  /* 设备不支持，或没绑这个 PDO；reason 里写明缺什么 */
#define HAL_CAP_NOT_CONFIGURED  3u  /* HAL 侧尚未配置该功能（非设备问题） */

typedef struct {
    uint32_t function;
    uint32_t state;
    int32_t  slave_pos;
    int32_t  axis_index;
    char     reason[96];
} HalCCapability;

/* ---- 参数取值 ---- */
#define HAL_WORK_POSITION     1   /* 位置用途配置；不等于已准备CSP或设备增量参数 */
#define HAL_WORK_VELOCITY     3   /* 速度模式（CSV） */
#define HAL_ENC_INCREMENTAL_Z 1   /* 增量式、带 Z 相；回零策略不归 HAL */
#define HAL_ENC_ABSOLUTE      3   /* 绝对式 */

/* 驱动器速度对象（0x606C 读 / 0x60FF 写）的单位。HAL 同时提供原始速度与换算后的 rpm：
 *   HAL_SPEED_RPM          —— 驱动器本身就是 rpm（蓝海华腾 C2-00 = 0）；
 *   HAL_SPEED_PULSES_PER_S —— 驱动器是脉冲/秒（蓝海华腾 C2-00 出厂默认），
 *                             HAL 用 command_pulses_per_rev 双向换算。
 * 不填就是 0（rpm）。 */
#define HAL_SPEED_RPM           0
#define HAL_SPEED_PULSES_PER_S  1
#define HAL_WRAP_LINEAR       0   /* 线性计数 */
#define HAL_WRAP_MODULAR      1   /* 模循环（主轴：模每转脉冲数） */

/* 急停时对该轴做什么。0 无效，必须由机床配置显式选择。 */
#define HAL_ESTOP_DISABLE_OPERATION 1   /* 去使能：0x07 退回 SwitchedOn 后 0x06 关断，停在 Ready to switch on */
#define HAL_ESTOP_DISABLE_VOLTAGE   2   /* 撤销电压（写 0x00，断电），最彻底 */

/* 通用轴属性 —— 进给轴和主轴共用；转速侧的量是主轴专有（见 HalCSpindleCfg） */
typedef struct {
    int32_t slave_pos;                  /* 绑定的从站位置（总线拓扑，运行期不变） */
    int32_t axis_index;                 /* 从站内物理轴序号，从 0 起 */
    int32_t estop_action;               /* HAL_ESTOP_*，不得省略 */ 
    int32_t logical_axis;               /* 逻辑轴号；-1 = 不占轴槽 */
    int32_t work_mode;                  /* HAL_WORK_* */
    int32_t encoder_type;               /* HAL_ENC_* */
    int32_t feedback_pulses_per_rev;    /* 每转反馈脉冲数 */
    int32_t feedback_wrap;              /* HAL_WRAP_* */
    double  command_units_per_count;    /* 命令当量（用户单位/脉冲），> 0；1 mm = 10000 命令计数则填 0.0001 */
    double  feedback_units_per_count;   /* 反馈当量，> 0。可与命令不同 */
    int32_t command_invert;             /* 非零 = 位置及主轴 CSV 转速命令方向取反 */
    int32_t feedback_invert;            /* 非零 = 位置及速度反馈方向取反；与命令分开 */
    double  enc_off;                    /* 已保存的坐标偏置（原始位置 - enc_off） */

    /* ---- 主轴定向（厂商专有，可选）----
     * 定向不是标准 DS402 动作：驱动器收到厂商规定的控制字后自行减速并对准，完成标志在
     * 状态字的某一位上。值与位都是驱动器专有的，由设备字典配。
     * 两个都为 0 = 该轴不支持定向，此时定向接口返回 HAL_ERROR_UNSUPPORTED（不静默当成功）。
     * 进给轴保持 0。 */
    uint16_t vendor_orient_control_word; /* 如 0x040F */
    uint16_t vendor_orient_done_mask;    /* 如 0x0400（状态字 bit10） */
    int32_t  vendor_orient_delay_ms;     /* 发出控制字后至少等这么久（ms）才判完成；<= 0 = 不等。
                                          * 延时内仍记录低电平；每轮必须先低后高，延时不能代替边沿。
                                          * 完成位在别的模式下可能是别的含义：蓝海华腾 VTS 6041h.bit10 在 CSV 模式下
                                          * 表示「速度到达」，发 0x40F 后延时 10ms 才表示「定位完成」。 */

    /* ---- 速度对象单位（驱动器专有）----
     * 取值见 HAL_SPEED_*；默认 0（rpm）。填 pulse/s 时必须同时给
     * command_pulses_per_rev（不配会被拒）；进给轴返回 mm/min 也需要它。 */
    int32_t  speed_unit;                 /* 0 = rpm；1 = pulse/s */
    int32_t  command_pulses_per_rev;     /* 每转指令脉冲数；pulse/s↔rpm 及进给轴 mm/min 换算使用 */
} HalCAxisCfg;

/* 主轴 —— 通用轴属性之上加转速上限（定向与运行期 CSV↔CSP 切换是主轴专有）。
 * @note axis.logical_axis 必须 >= 0 —— 主轴角度接口也用 HalAxisId 寻址。 */
typedef struct {
    HalCAxisCfg axis;                   /* 通用属性全在这，不重复 */
    double      max_speed;              /* rpm */
} HalCSpindleCfg;

typedef struct {
    int32_t slave_pos;
    int32_t x_start;                    /* 输入段起始**字节**地址（PLC 域 X），>= 0 */
    int32_t y_start;                    /* 输出段起始**字节**地址（PLC 域 Y），>= 0 */
} HalCIoCfg;

typedef struct {
    int32_t slave_pos;
    int32_t panel_type;                 /* HMI 按型号解释语义布局；HAL 不解读 */
    int32_t x_start;                    /* 与普通 IO 共用 PLC X 字节地址域 */
    int32_t y_start;                    /* 与普通 IO 共用 PLC Y 字节地址域 */
} HalCPanelCfg;

typedef struct {
    /* ---- 版本闸门（必须在最前，且顺序不变）---- */
    uint16_t abi_major;                 /* 必须 == HAL_C_ABI_MAJOR */
    uint16_t abi_minor;                 /* 必须 == HAL_C_ABI_MINOR */
    uint16_t struct_size;               /* 必须 == sizeof(HalCConfig)，精确相等 */
    uint16_t reserved;                  /* 必须为 0。显式填充，不依赖编译器 */

    uint64_t topology_fingerprint;      /* 当前必须为 0（指纹算法未定） */

    uint32_t cycle_us;
    uint32_t start_timeout_ms;          /* 整个启动过程共用的总预算 */
    uint32_t cycle_timeout_ms;
    uint32_t fault_reset_timeout_ms;    /* 故障复位从发起算起的等待上限（ms），必须 > 0。
                                           超过记录FAULT_RESET执行错误，并保持下使能要求。 */
    uint32_t enable_timeout_ms;       /* >0：上下文统一总预算 */
    uint32_t disable_timeout_ms;      /* >0：普通下使能/撤压总预算 */
    uint32_t mode_timeout_ms;         /* >0：模式准备总预算 */
    uint32_t orient_timeout_ms;       /* >0：定向总预算 */
    uint32_t orient_exit_timeout_ms;  /* >0：保持退出总预算 */
    int32_t  dc_enable;                 /* 0 或 1；运行期间配置不可变 */

    /* ---- 设备数组，前 count 个有效 ---- */
    int32_t        axis_count;
    HalCAxisCfg    axes[HAL_C_MAX_DEV];
    int32_t        spindle_count;
    HalCSpindleCfg spindles[HAL_C_MAX_SPINDLE];
    int32_t        io_count;
    HalCIoCfg      ios[HAL_C_MAX_DEV];
    int32_t        panel_count;
    HalCPanelCfg   panels[HAL_C_MAX_DEV];
} HalCConfig;


typedef struct {
    int32_t  type;                      /* 设备类型（字典里的 type 映射而来） */
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
    char     name[HAL_C_NAME_LEN];      /* 设备字典查出；未接入前为空串 */
    int32_t  family_index;              /* 同 vendor+product 的第几台（0 起，按总线顺序） */
    uint64_t device_id;                 /* serial 非零取 serial，否则 FNV(vendor,product,revision) */
} HalCIdentity;

typedef struct {
    int32_t slave_pos;                  /* 实际扫描顺序；包含未配置和未识别的从站 */
    int32_t axis_count;                 /* 已装配的物理轴数 */
    HalCIdentity identity;
} HalCSlaveInfo;

/* 总线健康快照（出参）。只记录不判断：原始计数与标志，什么算异常、要不要报警由上层决定。
 * 全部为累计/瞬时原始值，未做单位换算；还没收过帧时全零。 */
typedef struct {
    int32_t  pdo_warn;                  /* 非零 = 最近一帧有 PDO 警告 */
    int32_t  pdo_warn_code;             /* PDO 警告号 */
    uint32_t pdo_warn_para;             /* 警告参数 */
    int32_t  dc_warn;                   /* 非零 = 有分布式时钟警告 */
    int32_t  dc_warn_code;
    uint32_t dc_warn_para;
    uint32_t crc_error_count;           /* CRC 错误累计次数 */
    uint32_t frame_timeout_count;       /* 帧超时累计次数 */
    uint32_t expect_wkc_tx;             /* 期望工作计数器（Tx 侧） */
    uint32_t expect_wkc_rx;             /* 期望工作计数器（Rx 侧） */
} HalCBusHealth;

/* Fixed-width public meanings; no C enum storage assumptions. */
typedef int32_t HalAxisMode;
#define HAL_AXIS_MODE_UNSET 0
#define HAL_AXIS_MODE_CSP   1
#define HAL_AXIS_MODE_CSV   2

typedef int32_t HalDriveState;
#define HAL_DRIVE_UNKNOWN               0
#define HAL_DRIVE_NOT_READY             1
#define HAL_DRIVE_SWITCH_ON_DISABLED    2
#define HAL_DRIVE_READY                 3
#define HAL_DRIVE_SWITCHED_ON           4
#define HAL_DRIVE_OPERATION_ENABLED     5
#define HAL_DRIVE_QUICK_STOP_ACTIVE     6
#define HAL_DRIVE_FAULT_REACTION_ACTIVE 7
#define HAL_DRIVE_FAULT                 8

typedef int32_t HalControlState;
#define HAL_CONTROL_IDLE                   0
#define HAL_CONTROL_ENABLING               1
#define HAL_CONTROL_DISABLING              2
#define HAL_CONTROL_DROPPING_VOLTAGE        3
#define HAL_CONTROL_SETTING_MODE           4
#define HAL_CONTROL_RESETTING_FAULT         5
#define HAL_CONTROL_ORIENTING              6
#define HAL_CONTROL_ORIENT_HOLD            7
#define HAL_CONTROL_EXITING_ORIENT          8
#define HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN 9

typedef int32_t HalModeSettingState;
#define HAL_MODE_SETTING_UNSET   0
#define HAL_MODE_SETTING_SETTING 1
#define HAL_MODE_SETTING_READY   2
#define HAL_MODE_SETTING_FAILED  3

typedef int32_t HalZeroSpeedState;
#define HAL_ZERO_SPEED_UNAVAILABLE 0
#define HAL_ZERO_SPEED_ZERO        1
#define HAL_ZERO_SPEED_NONZERO     2

#define HAL_FEEDBACK_STATUS       (1u << 0)
#define HAL_FEEDBACK_POSITION     (1u << 1)
#define HAL_FEEDBACK_SPEED        (1u << 2)
#define HAL_FEEDBACK_MODE         (1u << 3)
#define HAL_FEEDBACK_ERROR_CODE   (1u << 4)
#define HAL_FEEDBACK_LINEAR_SPEED (1u << 5)
#define HAL_COMMAND_POSITION          (1u << 0)
#define HAL_COMMAND_SPINDLE_SPEED_RPM (1u << 1)

#define HAL_BLOCK_STOP_REQUIRED       (1u << 0)
#define HAL_BLOCK_FEEDBACK_UNTRUSTED  (1u << 1)
#define HAL_BLOCK_NOT_ENABLED        (1u << 2)
#define HAL_BLOCK_MODE_NOT_READY     (1u << 3)
#define HAL_BLOCK_MODE_MISMATCH      (1u << 4)
#define HAL_BLOCK_CONTROL_OCCUPIED   (1u << 5)
#define HAL_BLOCK_CAPABILITY_MISSING (1u << 6)

typedef int32_t HalIssueCategory;
#define HAL_ISSUE_NONE                  0
#define HAL_ISSUE_ENABLE                1
#define HAL_ISSUE_DISABLE               2
#define HAL_ISSUE_MODE                  3
#define HAL_ISSUE_ESTOP                 4
#define HAL_ISSUE_FAULT_RESET           5
#define HAL_ISSUE_ORIENT                6
#define HAL_ISSUE_EXIT_ORIENT           7
#define HAL_ISSUE_POSITION              8
#define HAL_ISSUE_SPINDLE_SPEED         9
#define HAL_ISSUE_SET_POSITION         10
#define HAL_ISSUE_CLEAR_EXECUTION_ERROR 11
#define HAL_ISSUE_AXIS_CONDITION       12

typedef struct HalAxisIssue {
    int32_t category;
    int32_t code;
} HalAxisIssue;

typedef struct HalAxisSnapshot {
    uint32_t available;
    uint32_t feedback_valid;
    uint32_t command_valid;
    HalZeroSpeedState zero_speed_state;
    HalDriveState drive_state;
    uint16_t raw_status;
    uint16_t reserved_status; /* Always zero. */
    uint32_t drive_error_code;
    HalControlState control_state;
    HalAxisMode target_mode;
    HalAxisMode actual_mode;
    HalModeSettingState mode_setting_state;
    uint32_t motion_block_reasons;
    HalAxisIssue request_rejection;
    HalAxisIssue execution_error;
    double coordinate_offset;
    double actual_pos;
    double command_pos;
    double actual_speed_device_units;
    double actual_speed_rpm;
    double actual_speed_mm_min;
    double command_speed_rpm;
} HalAxisSnapshot;

typedef struct HalOrientExitTarget {
    int32_t kind;
    uint32_t reserved; /* Caller sets zero. */
    double value;
} HalOrientExitTarget;
#define HAL_ORIENT_EXIT_POSITION      1
#define HAL_ORIENT_EXIT_SPINDLE_SPEED 2
