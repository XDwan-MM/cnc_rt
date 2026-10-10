#include "rt_axis_view.h"

#include <math.h>

#include "rt_data.h"
#include "datatype.h"

#include "rt_hal_adapter.h"

/* 与 global.h 里那条声明一致；这里自己写一遍是为了不把 global.h 的一堆依赖
 * 带进来（本文件只读 rt_data，不碰别的全局）。声明不一致时编译期就会报。 */
extern RT_DATA* rt_data;

/* 每转位移(mm) 与当量(mm/计数)。参数域没给全就报 PARAM —— 不返回 0 蒙人。
 * 每转位移就是 |电子齿轮比分子|：它本身就是"电机每转的负载位移"；
 * 当量 = 每转位移 / 每转指令脉冲数。 */
static int scaling_of(int axis, double* mm_per_count, double* mm_per_rev) {
    const RT_AXIS* a = &rt_data->axis_data[axis];
    if (a->pm_pulse == 0 || a->pm_munit == 0.0) return RT_AXIS_VIEW_ERR_PARAM;
    *mm_per_rev = fabs(a->pm_munit);
    *mm_per_count = *mm_per_rev / (double)a->pm_pulse;
    return RT_AXIS_VIEW_OK;
}

static int axis_ok(int axis) { return axis >= 0 && axis < SYS_AXES_NUM; }

/* 位置(mm) → 原始计数。驱动器位置 = 原始计数×当量 − enc_off（HAL 的约定，
 * RT 的坐标也照这个来），所以要拿原始计数得把 enc_off 加回去。 */
static int counts_of(int axis, double pos, int64_t* out) {
    double mm_per_count = 0.0, mm_per_rev = 0.0;
    const int rc = scaling_of(axis, &mm_per_count, &mm_per_rev);
    if (rc) return rc;
    *out = (int64_t)llround((pos + rt_data->axis_data[axis].enc_off) / mm_per_count);
    return RT_AXIS_VIEW_OK;
}

int rt_axis_view_cmd_counts(int axis, int64_t* out) {
    if (!out) return RT_AXIS_VIEW_ERR_ARG;
    if (!axis_ok(axis)) return RT_AXIS_VIEW_ERR_ARG;
    if (axis == AXIS_SP) return RT_AXIS_VIEW_ERR_UNSUPPORTED;
    return counts_of(axis, rt_data->axis_data[axis].motor_cmd_pos, out);
}

int rt_axis_view_fb_counts(int axis, int64_t* out) {
    if (!out) return RT_AXIS_VIEW_ERR_ARG;
    if (!axis_ok(axis)) return RT_AXIS_VIEW_ERR_ARG;
    if (axis == AXIS_SP) return RT_AXIS_VIEW_ERR_UNSUPPORTED;
    return counts_of(axis, rt_data->axis_data[axis].motor_act_pos, out);
}

int rt_axis_view_speed_mm_min(int axis, double* out) {
    if (!out || !axis_ok(axis)) return RT_AXIS_VIEW_ERR_ARG;
    if (axis == AXIS_SP) return RT_AXIS_VIEW_ERR_UNSUPPORTED;
    *out = rt_data->axis_data[axis].act_vel;    /* emcmot 已换算成 mm/min */
    return RT_AXIS_VIEW_OK;
}

int rt_axis_view_pos_deg(int axis, double* out) {
    if (!out || !axis_ok(axis)) return RT_AXIS_VIEW_ERR_ARG;
    if (axis != AXIS_SP) return RT_AXIS_VIEW_ERR_UNSUPPORTED;   /* 进给轴的位置是 mm */
    HalCSpindleStatus ss;
    /* 直接读 HAL 快照：主轴角度没有落进 rt_data（避免污染 motor_act_pos 的语义）。 */
    if (rt_hal_spindle_status((RtHalAxisSlot)axis, &ss) != HAL_OK)
        return RT_AXIS_VIEW_ERR_UNSUPPORTED;
    *out = ss.position_deg;
    return RT_AXIS_VIEW_OK;
}

int rt_axis_view_speed_rpm(int axis, double* out) {
    if (!out || !axis_ok(axis)) return RT_AXIS_VIEW_ERR_ARG;
    if (axis == AXIS_SP) {                      /* 主轴本来就是 rpm */
        *out = rt_data->axis_data[AXIS_SP].act_vel;
        return RT_AXIS_VIEW_OK;
    }
    double mm_per_count = 0.0, mm_per_rev = 0.0;
    const int rc = scaling_of(axis, &mm_per_count, &mm_per_rev);
    if (rc) return rc;
    *out = rt_data->axis_data[axis].act_vel / mm_per_rev;   /* mm/min ÷ 每转位移 = rpm */
    return RT_AXIS_VIEW_OK;
}
