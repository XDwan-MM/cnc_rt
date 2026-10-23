/* 轴量接口离线自检：假 rt_data + 真实现，验换算与拒绝路径；不连总线、不跑周期。 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rt_data.h"
#include "datatype.h"
#include "rt_axis_view.h"
#include "rt_hal_adapter.h"   /* 主轴角度读 HAL 快照（本测试给桩）*/

/* 本测试不链接 rt_data.c：自身实例顶掉 RT 全局 */
static RT_DATA g;
RT_DATA* rt_data = &g;

static double g_stub_deg;
int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out) {
    if (!out) return HAL_ERROR_ARGUMENT;
    if ((int)slot != AXIS_SP) return HAL_ERROR_ARGUMENT;
    memset(out, 0, sizeof(*out));
    out->position_deg = g_stub_deg;
    return HAL_OK;
}

/* 直接编进 rt_axis_view.c 真实现 */
#include "rt_axis_view.c"

static int fails = 0;

static void expect(int cond, const char* what) {
    if (cond) printf("  PASS  %s\n", what);
    else { printf("  FAIL  %s\n", what); ++fails; }
}

int main(void) {
    int64_t c = 0;
    double v = 0.0;

    memset(&g, 0, sizeof(g));
    g.axis_data[AXIS_X].pm_munit = 10.0;
    g.axis_data[AXIS_X].pm_pulse = 10000;
    g.axis_data[AXIS_X].enc_off = 150.0;
    g.axis_data[AXIS_X].motor_cmd_pos = 1000.0;
    g.axis_data[AXIS_X].motor_act_pos = 1000.5;
    g.axis_data[AXIS_X].act_vel = 6000.0;            /* mm/min */

    expect(rt_axis_view_cmd_counts(AXIS_X, &c) == RT_AXIS_VIEW_OK && c == 1150000,
           "指令脉冲 = (指令位置+enc_off)/当量");
    expect(rt_axis_view_fb_counts(AXIS_X, &c) == RT_AXIS_VIEW_OK && c == 1150500,
           "实际脉冲 = (反馈位置+enc_off)/当量");
    expect(rt_axis_view_speed_mm_min(AXIS_X, &v) == RT_AXIS_VIEW_OK && v == 6000.0,
           "mm/min 原样带出");
    expect(rt_axis_view_speed_rpm(AXIS_X, &v) == RT_AXIS_VIEW_OK && fabs(v - 600.0) < 1e-9,
           "rpm = mm/min ÷ 每转位移（6000/10）");

    /* 负当量 = 反向：每转位移取绝对值 */
    g.axis_data[AXIS_Y].pm_munit = -5.0;
    g.axis_data[AXIS_Y].pm_pulse = 5000;
    g.axis_data[AXIS_Y].enc_off = 0.0;
    g.axis_data[AXIS_Y].motor_cmd_pos = 1.0;         /* 当量 0.001 → 1000 计数 */
    g.axis_data[AXIS_Y].act_vel = 1000.0;
    expect(rt_axis_view_cmd_counts(AXIS_Y, &c) == RT_AXIS_VIEW_OK && c == 1000,
           "分子为负（反向）时计数照样对");
    expect(rt_axis_view_speed_rpm(AXIS_Y, &v) == RT_AXIS_VIEW_OK && fabs(v - 200.0) < 1e-9,
           "反向轴 rpm = 1000/5");

    /* Z 轴：缺参数报 PARAM，不能静默给 0 */
    g.axis_data[AXIS_Z].pm_munit = 0.0;
    g.axis_data[AXIS_Z].pm_pulse = 0;
    expect(rt_axis_view_cmd_counts(AXIS_Z, &c) == RT_AXIS_VIEW_ERR_PARAM,
           "缺参数：指令脉冲报错而不是给 0");
    expect(rt_axis_view_fb_counts(AXIS_Z, &c) == RT_AXIS_VIEW_ERR_PARAM,
           "缺参数：反馈脉冲报错");
    expect(rt_axis_view_speed_rpm(AXIS_Z, &v) == RT_AXIS_VIEW_ERR_PARAM,
           "缺参数：rpm 报错");
    expect(rt_axis_view_speed_mm_min(AXIS_Z, &v) == RT_AXIS_VIEW_OK,
           "缺参数不影响 mm/min（那是 emcmot 给的现成值）");

    /* 主轴：无脉冲计数/mm/min，rpm 即自身转速 */
    g.axis_data[AXIS_SP].act_vel = 1200.0;
    expect(rt_axis_view_cmd_counts(AXIS_SP, &c) == RT_AXIS_VIEW_ERR_UNSUPPORTED,
           "主轴没有指令脉冲");
    expect(rt_axis_view_fb_counts(AXIS_SP, &c) == RT_AXIS_VIEW_ERR_UNSUPPORTED,
           "主轴没有反馈脉冲");
    expect(rt_axis_view_speed_mm_min(AXIS_SP, &v) == RT_AXIS_VIEW_ERR_UNSUPPORTED,
           "主轴没有 mm/min");
    expect(rt_axis_view_speed_rpm(AXIS_SP, &v) == RT_AXIS_VIEW_OK && v == 1200.0,
           "主轴 rpm 原样带出");

    /* 主轴角度：走 HAL 快照，不落 rt_data */
    g_stub_deg = 1234.5;
    expect(rt_axis_view_pos_deg(AXIS_SP, &v) == RT_AXIS_VIEW_OK && v == 1234.5,
           "主轴角度原样带出（累计、不取模）");
    g_stub_deg = -720.0;
    expect(rt_axis_view_pos_deg(AXIS_SP, &v) == RT_AXIS_VIEW_OK && v == -720.0,
           "主轴角度可以为负（反向累计）");
    expect(rt_axis_view_pos_deg(AXIS_X, &v) == RT_AXIS_VIEW_ERR_UNSUPPORTED,
           "进给轴没有角度：报 UNSUPPORTED 而不是给位置当角度");
    expect(rt_axis_view_pos_deg(AXIS_SP, NULL) == RT_AXIS_VIEW_ERR_ARG, "角度接口空出参被拒");

    /* 越界与空指针 */
    expect(rt_axis_view_cmd_counts(-1, &c) == RT_AXIS_VIEW_ERR_ARG, "负轴号被拒");
    expect(rt_axis_view_cmd_counts(SYS_AXES_NUM, &c) == RT_AXIS_VIEW_ERR_ARG, "轴号越界被拒");
    expect(rt_axis_view_speed_rpm(AXIS_X, NULL) == RT_AXIS_VIEW_ERR_ARG, "空出参被拒");
    expect(rt_axis_view_speed_mm_min(SYS_AXES_NUM + 3, &v) == RT_AXIS_VIEW_ERR_ARG, "空指针外的越界也被拒");

    printf("\n######## 结果：%s（失败 %d 项）########\n", fails ? "有失败" : "全部 PASS", fails);
    return fails ? 1 : 0;
}
