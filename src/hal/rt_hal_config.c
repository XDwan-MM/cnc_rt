/* 台架本地 profile 实现。取值须有依据；没依据的挂 RT_HAL_TODO_* 台账。 */

#include "rt_hal_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* hal_config_validate / HAL_OK 在 hal_config_api.h；rt_hal_config.h 只带
 * hal_types.h（类型），避免让配置头把校验接口也拖进来。 */
#include "hal_config_api.h"

/* 模块内状态。台账按轴记，全局掩码分不清"全都缺"和"只缺一根"。 */
static int      s_built;                                  /* 是否已 build 过 */
static uint32_t s_todo = 0xFFFFFFFFu;                     /* 对外台账掩码 */
static uint8_t  s_scaling_ok[RT_HAL_AXIS_COUNT];          /* 当量+enc_off 已装载 */
static uint8_t  s_pulses_ok[RT_HAL_AXIS_COUNT];           /* 每转反馈计数已知 */
static uint8_t  s_spindle_max_ok;                         /* 主轴转速上限已从参数域装载 */

/* "现场确认开关"已删：IO_MAP / MOTION 两位不再进台账。
 * 留下来的保守行为不变：**参数域没给换算就挡住运动**
 * （SCALING / ENC_OFF / ENCODER 三位仍按轴挂，见 recompute_todo()）。 */

static void recompute_todo(void) {
    /* 台账只反映"参数域给全了没有"，按轴算。 */
    uint32_t t = 0;
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        if (!s_scaling_ok[i]) t |= RT_HAL_TODO_SCALING | RT_HAL_TODO_ENC_OFF;
        if (!s_pulses_ok[i])  t |= RT_HAL_TODO_ENCODER;
    }
    s_todo = t;
}

/* 按 HAL 逻辑轴号定位；不按下标，axes[] 与 spindles[] 是两张表 */
static HalCAxisCfg* find_axis_cfg(HalCConfig* cfg, int hal_axis) {
    for (int i = 0; i < cfg->axis_count; ++i)
        if (cfg->axes[i].logical_axis == hal_axis) return &cfg->axes[i];
    for (int i = 0; i < cfg->spindle_count; ++i)
        if (cfg->spindles[i].axis.logical_axis == hal_axis) return &cfg->spindles[i].axis;
    return NULL;
}

/* 从站拓扑：0 面板、1 IO、2 主轴、3/4/5 进给，沿用旧 RT 编号 */
#define RT_SLAVE_PANEL   0
#define RT_SLAVE_IO      1
#define RT_SLAVE_SPINDLE 2
#define RT_SLAVE_X       3
#define RT_SLAVE_Y       4
#define RT_SLAVE_Z       5

/* 轴绑定表：RT 槽 ↔ 从站 ↔ HAL 逻辑轴号。
 * 第 4 列与行号相同是有意的——多一套编号就多一类静默错误。不要改。 */
static const RtHalBinding kBindings[RT_HAL_AXIS_COUNT] = {
    {"X",  RT_SLAVE_X,       0, 0, 0},
    {"Y",  RT_SLAVE_Y,       0, 1, 0},
    {"Z",  RT_SLAVE_Z,       0, 2, 0},
    {"SP", RT_SLAVE_SPINDLE, 0, 3, 1},
};

/* 主站参数，沿用旧 RT 的 main_demo.h；待与 RT 插补周期核对 */
#define RT_CYCLE_US        1000u
#define RT_START_TIMEOUT   120000u
#define RT_CYCLE_TIMEOUT   5000u
#define RT_FAULT_RESET_TIMEOUT 5000u   /* 故障复位等待上限（ms） */
#define RT_DC_ENABLE       1

/* 未确认占位值。用恒等 1.0/1 而不是抄示例——忘了看台账时结果是明显不对的。 */
#define RT_PLACEHOLDER_UNITS_PER_COUNT 1.0
#define RT_PLACEHOLDER_PULSES_PER_REV  1
#define RT_PLACEHOLDER_ENC_OFF         0.0
/* 主轴转速上限。只是配置域里的占位，转速约束实际由驱动器侧决定。 */
#define RT_PLACEHOLDER_SPINDLE_MAX_RPM 1000.0

static int fail(char* err, uint32_t len, int code, const char* text) {
    if (err && len) snprintf(err, len, "%s", text);
    return code;
}

uint32_t rt_hal_config_todo(void) {
    /* TODO_ESTOP 不在列：四轴一律"去使能"，见 fill_axis()。 */
    return s_todo;
}

int rt_hal_config_todo_count(void) {
    int n = 0;
    for (uint32_t bit = 1u; bit; bit <<= 1u)
        if (s_todo & bit) ++n;
    return n;
}

int rt_hal_config_scaling_loaded(RtHalAxisSlot slot) {
    if (slot < 0 || slot >= RT_HAL_AXIS_COUNT) return 0;
    return s_scaling_ok[slot] && s_pulses_ok[slot];
}

int rt_hal_config_hal_axis(RtHalAxisSlot slot) {
    if (slot < 0 || slot >= RT_HAL_AXIS_COUNT) return -1;
    return kBindings[slot].hal_axis;
}

/* 主轴判定，唯一来源是绑表第 5 列（build() 按同一列装配 spindles[]）。 */
int rt_hal_config_is_spindle(RtHalAxisSlot slot) {
    if (slot < 0 || slot >= RT_HAL_AXIS_COUNT) return 0;
    return kBindings[slot].is_spindle;
}

const char* rt_hal_config_todo_text(uint32_t bit) {
    switch (bit) {
    case RT_HAL_TODO_SCALING: return "命令/反馈当量与方向未现场确认";
    case RT_HAL_TODO_ENCODER: return "编码器类型、每转反馈计数与 wrap 未确认";
    case RT_HAL_TODO_ENC_OFF: return "enc_off 坐标偏置未确认";
    case RT_HAL_TODO_ESTOP:   return "急停动作（去使能 or 撤销电压）未选定";
    default:                  return "未知项";
    }
}

int rt_hal_config_motion_allowed(void) {
    return rt_hal_config_todo() == 0u;
}

/* 填一根轴。spindle 非零按主轴规则填（work_mode 可以不是 CSP）。 */
static void fill_axis(HalCAxisCfg* a, const RtHalBinding* b, int spindle) {
    a->slave_pos = b->slave_pos;
    a->axis_index = b->axis_index;
    a->logical_axis = b->hal_axis;
    /* 急停动作必须显式给（0 会被拒）；四轴一律"去使能" */
    a->estop_action = HAL_ESTOP_DISABLE_OPERATION;
    /* 进给轴必须是位置模式（HAL 强制）；主轴默认速度模式 */
    a->work_mode = spindle ? HAL_WORK_VELOCITY : HAL_WORK_POSITION;
    /* HAL 只校验不消费这个字段 */
    a->encoder_type = HAL_ENC_ABSOLUTE;
    a->feedback_pulses_per_rev = RT_PLACEHOLDER_PULSES_PER_REV;
    /* 主轴模循环（每转回绕），进给线性计数 */
    a->feedback_wrap = spindle ? HAL_WRAP_MODULAR : HAL_WRAP_LINEAR;
    a->command_units_per_count = RT_PLACEHOLDER_UNITS_PER_COUNT;
    a->feedback_units_per_count = RT_PLACEHOLDER_UNITS_PER_COUNT;
    /* 命令与反馈方向分开配：外接光栅尺可能只需翻反馈 */
    a->command_invert = 0;
    a->feedback_invert = 0;
    a->enc_off = RT_PLACEHOLDER_ENC_OFF;
    /* 定向只有主轴用；进给轴留 0（= 不支持），M19 在进给轴上会被拒。 */
    a->vendor_orient_control_word = spindle ? RT_PLACEHOLDER_ORIENT_CW : 0u;
    a->vendor_orient_done_mask    = spindle ? RT_PLACEHOLDER_ORIENT_DONE : 0u;
    a->vendor_orient_delay_ms     = spindle ? RT_ORIENT_DELAY_MS : 0;
    /* 速度对象单位（见 kSpeedUnit）；hal_axis 是恒等映射，这里再兜一次越界 */
    a->speed_unit = (b->hal_axis >= 0 && b->hal_axis < RT_HAL_AXIS_COUNT)
                        ? kSpeedUnit[b->hal_axis] : HAL_SPEED_RPM;
}

int rt_hal_config_build(HalCConfig* out, char* err, uint32_t err_len) {
    if (!out) return fail(err, err_len, RT_HAL_ERR_NULL_ARG, "配置出参为空");
    if (err && err_len) err[0] = '\0';

    /* 重置模块状态：换算从占位值开始，由 set_scaling() 逐轴换真值 */
    s_built = 1;
    memset(s_scaling_ok, 0, sizeof(s_scaling_ok));
    memset(s_pulses_ok, 0, sizeof(s_pulses_ok));
    s_spindle_max_ok = 0;
    /* 初始台账必须走 recompute_todo() 算，不能写死常量：
     * 参数域为空时要挂满 SCALING/ENC_OFF/ENCODER，让运动被挡住。 */
    recompute_todo();

    /* ABI 闸门：必须拦在编译期。HalCAxisStatus 是**出参**，身上没有 struct_size
     * 可查，HAL 只给它加字段时 hal_config_validate() 查不出来，而 RT 备的是栈上小
     * 一号的结构体 —— 结果是**写坏栈**。
     * 改了 HAL 的 HAL_C_ABI_* 就要同步改这一行。 */
#if HAL_C_ABI_MAJOR != 7u || HAL_C_ABI_MINOR != 2u
#  error "HAL ABI 与 rt_hal_config 预期不符（期望 7.2）：先读 HAL/include/hal_types.h 的版本说明，再改这一行"
#endif

    memset(out, 0, sizeof(*out));

    /* HAL 用 struct_size 精确相等挡字段插入 */
    out->abi_major = HAL_C_ABI_MAJOR;
    out->abi_minor = HAL_C_ABI_MINOR;
    out->struct_size = (uint16_t)sizeof(*out);
    out->reserved = 0;
    out->topology_fingerprint = 0; /* 算法未定，HAL 现在拒绝非零值 */

    out->cycle_us = RT_CYCLE_US;
    out->start_timeout_ms = RT_START_TIMEOUT;
    out->cycle_timeout_ms = RT_CYCLE_TIMEOUT;
    /* 故障复位等待上限（ms） */
    out->fault_reset_timeout_ms = RT_FAULT_RESET_TIMEOUT;
    out->dc_enable = RT_DC_ENABLE;

    /* 三进给 + 一主轴。主轴也放在 spindles[] 且要能被 HalAxisId 寻址，
     * 所以 logical_axis 必须 >= 0。 */
    out->axis_count = 3;
    for (int i = 0; i < 3; ++i)
        fill_axis(&out->axes[i], &kBindings[i], 0);

    out->spindle_count = 1;
    fill_axis(&out->spindles[0].axis, &kBindings[RT_HAL_AXIS_SP], 1);
    out->spindles[0].max_speed = RT_PLACEHOLDER_SPINDLE_MAX_RPM;

    /* IO 与面板共用 PLC 的 X/Y 地址域，靠起始地址分段：
     * IO 占 X0..X3 / Y0..Y3，面板占 X4..X25 / Y4..Y17。 */
    out->io_count = 1;
    out->ios[0].slave_pos = RT_SLAVE_IO;
    out->ios[0].x_start = 0;
    out->ios[0].y_start = 0;

    out->panel_count = 1;
    out->panels[0].slave_pos = RT_SLAVE_PANEL;
    out->panels[0].panel_type = 0; /* HAL 不解读，语义由 HMI 按型号解释 */
    out->panels[0].x_start = 4;
    out->panels[0].y_start = 4;

    return 0;
}

/* 按 HAL 逻辑轴号反查 RT 轴槽，找不到返回 -1 */
static int slot_of_hal_axis(int hal_axis) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i)
        if (kBindings[i].hal_axis == hal_axis) return i;
    return -1;
}

/* 标注换算来源：机床真值还是占位值 */
static const char* scaling_note(int hal_axis) {
    const int slot = slot_of_hal_axis(hal_axis);
    if (slot < 0) return "   [换算: 非本 profile 的轴]";
    if (!s_scaling_ok[slot]) return "   [换算: **占位值**]";
    if (!s_pulses_ok[slot])  return "   [换算: 已装载, 每转计数未知]";
    return "   [换算: 已装载]";
}

void rt_hal_config_print(const HalCConfig* cfg) {
    if (!cfg) return;
    printf("==== HAL 台架 profile: %s rev%d ====\n", RT_HAL_PROFILE_NAME, RT_HAL_PROFILE_REV);
    printf("HAL ABI %u.%u  struct_size=%u\n",
           (unsigned)cfg->abi_major, (unsigned)cfg->abi_minor, (unsigned)cfg->struct_size);
    printf("周期 %u us, 启动预算 %u ms, 周期超时 %u ms, DC=%d\n",
           cfg->cycle_us, cfg->start_timeout_ms, cfg->cycle_timeout_ms, cfg->dc_enable);

    printf("-- 轴绑定 --\n");
    for (int i = 0; i < cfg->axis_count; ++i) {
        const HalCAxisCfg* a = &cfg->axes[i];
        printf("  进给 #%d: 从站 %d 轴 %d -> HAL ID %d, %s, 当量 %.6g/%.6g, enc_off %.6g, 每转计数 %d%s\n",
               i, a->slave_pos, a->axis_index, a->logical_axis,
               a->work_mode == HAL_WORK_POSITION ? "CSP" : "CSV",
               a->command_units_per_count, a->feedback_units_per_count, a->enc_off,
               a->feedback_pulses_per_rev, scaling_note(a->logical_axis));
    }
    for (int i = 0; i < cfg->spindle_count; ++i) {
        const HalCSpindleCfg* s = &cfg->spindles[i];
        printf("  主轴 #%d: 从站 %d 轴 %d -> HAL ID %d, %s, 上限 %.6g rpm%s, 每转计数 %d%s\n",
               i, s->axis.slave_pos, s->axis.axis_index, s->axis.logical_axis,
               s->axis.work_mode == HAL_WORK_POSITION ? "CSP" : "CSV",
               s->max_speed,
               s_spindle_max_ok ? " [上限: 已装载]" : " [上限: **占位值**]",
               s->axis.feedback_pulses_per_rev,
               scaling_note(s->axis.logical_axis));
    }

    printf("-- IO/面板 --\n");
    for (int i = 0; i < cfg->io_count; ++i)
        printf("  IO   #%d: 从站 %d, X 起 %d, Y 起 %d\n",
               i, cfg->ios[i].slave_pos, cfg->ios[i].x_start, cfg->ios[i].y_start);
    for (int i = 0; i < cfg->panel_count; ++i)
        printf("  面板 #%d: 从站 %d, type %d, X 起 %d, Y 起 %d\n",
               i, cfg->panels[i].slave_pos, cfg->panels[i].panel_type,
               cfg->panels[i].x_start, cfg->panels[i].y_start);

    printf("-- 未确认台账 --\n");
    const uint32_t todo = rt_hal_config_todo();
    if (!todo) {
        printf("  无（可以开放运动）\n");
    } else {
        for (uint32_t bit = 1u; bit; bit <<= 1u)
            if (todo & bit) printf("  [未确认] %s\n", rt_hal_config_todo_text(bit));
        printf("  => 运动被阻止：rt_hal_config_motion_allowed() 返回 0\n");
    }
    printf("============================\n");
}

int rt_hal_config_finish(HalCConfig* cfg, char* err, uint32_t err_len) {
    if (!cfg) return RT_HAL_ERR_NULL_ARG;
    if (!s_built) return fail(err, err_len, RT_HAL_ERR_ORDER, "未先调用 rt_hal_config_build()");

    /* 复用调用方的错误缓冲，直接把 HAL 的原因带上去 */
    char local[192];
    char* buf = (err && err_len) ? err : local;
    const uint32_t cap = (err && err_len) ? err_len : (uint32_t)sizeof(local);

    const int32_t v = hal_config_validate(cfg, buf, cap);
    if (v != HAL_OK) return v;

    rt_hal_config_print(cfg);
    return 0;
}

/* 用 profile 的占位值装配一套并校验，参数域为空时的入口。
 * **不是 RT 的启动入口**：启动走 rt_hal_create() 的
 * build → set_scaling ×N → set_spindle_max → finish。 */
int rt_hal_config_build_and_check(HalCConfig* out, char* err, uint32_t err_len) {
    const int rc = rt_hal_config_build(out, err, err_len);
    if (rc) return rc;
    return rt_hal_config_finish(out, err, err_len);
}

int rt_hal_config_set_scaling(HalCConfig* cfg, RtHalAxisSlot slot, const RtHalAxisScaling* s) {
    if (!cfg || !s) return RT_HAL_ERR_NULL_ARG;
    if (!s_built) return RT_HAL_ERR_ORDER;
    if (slot < 0 || slot >= RT_HAL_AXIS_COUNT) return RT_HAL_ERR_SLOT;

    /* 先全部校验再落字段：不留半套换算（否则坐标静默偏移） */
    if (!isfinite(s->units_per_count) || s->units_per_count <= 0.0 ||
        !isfinite(s->enc_off) || s->feedback_pulses_per_rev < 0 ||
        s->command_pulses_per_rev < 0)
        return RT_HAL_ERR_PARAM;

    HalCAxisCfg* a = find_axis_cfg(cfg, kBindings[slot].hal_axis);
    if (!a) return RT_HAL_ERR_SLOT;

    a->command_units_per_count = s->units_per_count;
    /* RT 命令与反馈共用一个电子齿轮比；HAL 允许不同，本台架不需要 */
    a->feedback_units_per_count = s->units_per_count;
    a->command_invert = s->command_invert ? 1 : 0;
    a->command_pulses_per_rev = s->command_pulses_per_rev;
    a->feedback_invert = s->feedback_invert ? 1 : 0;
    a->enc_off = s->enc_off;
    if (s->feedback_pulses_per_rev > 0) {
        a->feedback_pulses_per_rev = s->feedback_pulses_per_rev;
        s_pulses_ok[slot] = 1;
    }
    s_scaling_ok[slot] = 1;
    recompute_todo();
    return 0;
}

int rt_hal_config_set_spindle_max(HalCConfig* cfg, double max_rpm) {
    if (!cfg) return RT_HAL_ERR_NULL_ARG;
    if (!s_built) return RT_HAL_ERR_ORDER;
    if (!isfinite(max_rpm) || max_rpm <= 0.0) return RT_HAL_ERR_PARAM;
    if (cfg->spindle_count < 1) return RT_HAL_ERR_PARAM;
    cfg->spindles[0].max_speed = max_rpm;
    s_spindle_max_ok = 1;
    return 0;
}
