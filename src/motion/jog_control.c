#include "jog_control.h"
#include <stddef.h>

void prepare_jog_params(const ParamUnit* axis_para, simple_tp_t *jog_tp, int direction, double feed_override, bool is_rapid)
{
    if (axis_para == NULL || jog_tp == NULL || feed_override < 0.0) return;
    if (direction == 0) return;

    /* 1. 设置目标位置：根据方向指向软限位终点 */
    if (direction > 0) {
        jog_tp->pos_cmd = axis_para[PAR_AX_PLMT].value.val_real;
    } else {
        jog_tp->pos_cmd = axis_para[PAR_AX_NLMT].value.val_real;
    }

    /* 2. 设置速度和加速度限制 */
    if (is_rapid) {
        jog_tp->max_vel = axis_para[PAR_AX_JOG_FASTSPD].value.val_real * feed_override;
        jog_tp->max_acc = axis_para[PAR_AX_RAPID_ACC].value.val_real;
    } else {
        jog_tp->max_vel = axis_para[PAR_AX_JOG_LOWSPD].value.val_real * feed_override;
        jog_tp->max_acc = axis_para[PAR_AX_FEED_ACC].value.val_real;
    }

    /* 3. 使能规划器：标记开始进入加减速计算 */
    jog_tp->enable = 1;
}

void request_jog_stop(simple_tp_t *jog_tp, bool immediate)
{
    if (jog_tp == NULL) return;

    /* 断开使能：规划器在下一次 update 时会发现此信号并开始减速 */
    jog_tp->enable = 0;

    if (immediate) {
        jog_tp->curr_vel = 0.0;
        jog_tp->active = 0;
    }
}