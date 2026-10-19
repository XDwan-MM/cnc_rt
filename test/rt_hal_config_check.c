/* 台架 profile 离线校验：只链 HAL 静态库（与旧 libethercat/master.c 有 31 个重名符号，
 * 不能同时进一次链接）。正向装配/校验/打印 + 反向闸门拒绝；退出码 0 = 全部通过。 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rt_hal_config.h"
#include "hal_config_api.h"

static int g_pass = 0;
static int g_fail = 0;

static void expect(int ok, const char* what) {
    if (ok) { ++g_pass; printf("  PASS  %s\n", what); }
    else    { ++g_fail; printf("  FAIL  %s\n", what); }
}

/* HAL 错误码转文本：便于区分配置问题与 ABI 问题 */
static void print_rc(const char* tag, int rc) {
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    printf("  %s -> rc=%d (%s)\n", tag, rc, text);
}

int main(void) {
    HalCConfig cfg;
    char err[192];

    printf("######## 台架 profile 校验 ########\n\n");

    /* ---- 1. 装配 ---- */
    printf("[1] rt_hal_config_build\n");
    int rc = rt_hal_config_build(&cfg, err, sizeof(err));
    print_rc("build", rc);
    expect(rc == 0, "build 返回 0");
    if (rc) { printf("err=%s\n", err); return 1; }

    /* ---- 2. 正向校验 ---- */
    printf("\n[2] hal_config_validate（正向）\n");
    rc = hal_config_validate(&cfg, err, sizeof(err));
    print_rc("validate", rc);
    if (rc) printf("      err=%s\n", err);
    expect(rc == HAL_OK, "profile 能通过 HAL 静态校验");

    /* ---- 3. 打印 ---- */
    printf("\n[3] rt_hal_config_print\n");
    rt_hal_config_print(&cfg);

    /* ---- 4. 未确认台账 ---- */
    printf("\n[4] 未确认台账\n");
    const uint32_t todo = rt_hal_config_todo();
    printf("  todo=0x%02x, motion_allowed=%d\n", todo, rt_hal_config_motion_allowed());
    expect(todo != 0u, "当前有未确认项（符合预期：换算依据尚未现场确认）");
    expect(rt_hal_config_motion_allowed() == 0, "未确认项存在时运动被阻止");
    for (uint32_t bit = 1u; bit; bit <<= 1u)
        if (todo & bit) printf("    - %s\n", rt_hal_config_todo_text(bit));

    /* ---- 4b. 从机床参数装载换算量 ---- */
    printf("\n[4b] rt_hal_config_set_scaling\n");
    {
        /* 记下占位值：证明非法输入不改动任何字段 */
        const double before_units = cfg.axes[0].command_units_per_count;
        const double before_off   = cfg.axes[0].enc_off;

        /* 非法值：当量 0 必须整体拒绝，不留半套换算 */
        RtHalAxisScaling bad = {0.0, 12.0, 0, 0, 10000};
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_X, &bad) == RT_HAL_ERR_PARAM,
               "当量为 0 被拒 (RT_HAL_ERR_PARAM)");
        expect(cfg.axes[0].command_units_per_count == before_units &&
               cfg.axes[0].enc_off == before_off,
               "被拒时未改动任何字段（不留半套换算）");

        bad.units_per_count = NAN;
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_X, &bad) == RT_HAL_ERR_PARAM,
               "当量为 NaN 被拒");

        bad.units_per_count = 0.0001;
        bad.feedback_pulses_per_rev = -1;
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_X, &bad) == RT_HAL_ERR_PARAM,
               "每转计数为负被拒");

        expect(rt_hal_config_set_scaling(NULL, RT_HAL_AXIS_X, &bad) == RT_HAL_ERR_NULL_ARG,
               "cfg 为空被拒");
        expect(rt_hal_config_set_scaling(&cfg, (RtHalAxisSlot)99, &bad) == RT_HAL_ERR_SLOT,
               "轴槽越界被拒");

        /* 合法值：当量 = 1mm/转 ÷ 10000 脉冲/转 = 0.0001 mm/count */
        const RtHalAxisScaling feed = {0.0001, 150.0, 0, 0, 100000};
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_X, &feed) == 0, "X 装载成功");
        expect(cfg.axes[0].command_units_per_count == 0.0001 &&
               cfg.axes[0].feedback_units_per_count == 0.0001 &&
               cfg.axes[0].enc_off == 150.0,
               "X 的当量与 enc_off 已换成真值");
        expect(rt_hal_config_scaling_loaded(RT_HAL_AXIS_X), "X 标记为已装载");
        expect(!rt_hal_config_scaling_loaded(RT_HAL_AXIS_Y), "Y 仍未装载");

        /* 主轴：带符号 rpm，每转计数未知（传 0） */
        const RtHalAxisScaling sp = {0.16, 0.0, 0, 0, 0};
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_SP, &sp) == 0, "SP 装载成功");
        expect(!rt_hal_config_scaling_loaded(RT_HAL_AXIS_SP),
               "SP 每转计数未知 → 仍算未装载");

        /* 只装 X/SP 时台账仍置 SCALING：一根轴没参数就挡住整机运动（有意保守） */
        const uint32_t t2 = rt_hal_config_todo();
        expect((t2 & RT_HAL_TODO_SCALING) != 0,
               "只装 2/4 根轴时 SCALING 仍置起（保守挡住整机）");

        /* 补 Y/Z 当量；ENCODER 因缺每转计数仍留着 */
        const RtHalAxisScaling y = {0.0001, 150.0, 0, 0, 100000};
        const RtHalAxisScaling z = {0.0001, 150.0, 0, 0, 100000};
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_Y, &y) == 0, "Y 装载成功");
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_Z, &z) == 0, "Z 装载成功");
        const uint32_t t3 = rt_hal_config_todo();
        expect((t3 & RT_HAL_TODO_SCALING) == 0 && (t3 & RT_HAL_TODO_ENC_OFF) == 0,
               "四轴当量与 enc_off 全部装载后，SCALING/ENC_OFF 位清空");
        expect((t3 & RT_HAL_TODO_ENCODER) != 0,
               "主轴每转计数仍未知 → ENCODER 位保留");
        expect(t3 == RT_HAL_TODO_ENCODER,
               "剩余台账正好是 ENCODER（主轴每转计数还未知）");
        expect(rt_hal_config_motion_allowed() == 0, "仍有未确认项，运动继续被阻止");

        /* 补上主轴每转计数（取参数域 pm_pulse）→ ENCODER 清空：参数域给全即台账清空、运动放开 */
        const RtHalAxisScaling sp2 = {0.16, 0.0, 0, 0, 120000};
        expect(rt_hal_config_set_scaling(&cfg, RT_HAL_AXIS_SP, &sp2) == 0,
               "SP 补上每转计数后重新装载成功");
        expect(rt_hal_config_scaling_loaded(RT_HAL_AXIS_SP), "SP 转为已装载");
        const uint32_t t4 = rt_hal_config_todo();
        expect(t4 == 0u, "四轴全装载后台账清空（不再有手工门）");
        expect(rt_hal_config_motion_allowed() == 1, "参数域给全后运动放开");

        printf("\n[4c] rt_hal_config_finish（带真值收口）\n");
        int frc = rt_hal_config_finish(&cfg, err, sizeof(err));
        if (frc) printf("      err=%s\n", err);
        expect(frc == 0, "装载真值后 finish 通过 HAL 校验");
    }

    /* ---- 5. 反向：闸门必须拒绝 ---- */
    printf("\n[5] 反向用例（闸门必须拒绝）\n");

    HalCConfig bad;

    bad = cfg; bad.abi_minor = HAL_C_ABI_MINOR + 1;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_ABI, "abi_minor 不匹配被拒 (HAL_ERROR_ABI)");

    bad = cfg; bad.abi_major = HAL_C_ABI_MAJOR + 1;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_ABI, "abi_major 不匹配被拒");

    bad = cfg; bad.struct_size = (uint16_t)(bad.struct_size - 1);
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_ABI, "struct_size 差一字节被拒（挡字段插入）");

    bad = cfg; bad.reserved = 1;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_ABI, "reserved 非零被拒");

    bad = cfg; bad.topology_fingerprint = 1;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "非零拓扑指纹被拒（算法未定）");

    bad = cfg; bad.cycle_us = 100;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "周期低于 250us 被拒");

    /* 重复逻辑轴号 */
    bad = cfg; bad.axes[0].logical_axis = bad.axes[1].logical_axis;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "逻辑轴号重复被拒");

    /* 进给只能位置模式（CSP） */
    bad = cfg; bad.axes[0].work_mode = HAL_WORK_VELOCITY;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "进给轴配 CSV 被拒（进给只能 CSP）");

    /* 当量必须为正 */
    bad = cfg; bad.axes[0].command_units_per_count = 0.0;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "命令当量为 0 被拒");

    /* 急停动作必须显式选，0 无效 */
    bad = cfg; bad.spindles[0].axis.estop_action = 0;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "主轴急停动作留 0 被拒（必须显式选）");

    /* IO 与轴的从站冲突 */
    bad = cfg; bad.ios[0].slave_pos = bad.axes[0].slave_pos;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "IO 与伺服共用从站被拒");

    /* IO 与面板从站重复 */
    bad = cfg; bad.panels[0].slave_pos = bad.ios[0].slave_pos;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "IO 与面板从站重复被拒");

    /* 主轴必须有有效逻辑轴号（角度接口要用 HalAxisId 寻址） */
    bad = cfg; bad.spindles[0].axis.logical_axis = -1;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "主轴逻辑轴号为 -1 被拒");

    /* 主轴转速上限必须为正 */
    bad = cfg; bad.spindles[0].max_speed = 0.0;
    rc = hal_config_validate(&bad, err, sizeof(err));
    expect(rc == HAL_ERROR_CONFIG, "主轴 max_speed 为 0 被拒");

    /* ---- 6. rt_hal_config_build_and_check 一把过 ---- */
    printf("\n[6] rt_hal_config_build_and_check（装配+校验+打印）\n");
    rc = rt_hal_config_build_and_check(&cfg, err, sizeof(err));
    print_rc("init", rc);
    if (rc) printf("      err=%s\n", err);
    expect(rc == 0, "init 返回 0");

    /* ---- 7. 空指针 ---- */
    printf("\n[7] 参数防御\n");
    rc = rt_hal_config_build(NULL, err, sizeof(err));
    expect(rc == RT_HAL_ERR_NULL_ARG, "build(NULL) 返回 RT_HAL_ERR_NULL_ARG");
    expect(rt_hal_config_build_and_check(NULL, NULL, 0) != 0,
           "build_and_check(NULL, NULL, 0) 不崩且返回非零");

    printf("\n######## 结果：%d 项通过，%d 项失败 ########\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
