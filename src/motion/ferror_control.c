#include "ferror_control.h"
#include <math.h>
#include <stdlib.h>

/*
* axis_para:传入的误差值单位mm
*/
void updata_axis_ferror(ferror_control_t *axis_ferrorCtrl,
                        const PAR_AXIS* axis_para,
                        double pos_cmd,
                        double pos_fb,
                        double vel_cmd)
{
    if (axis_ferrorCtrl == NULL || axis_para == NULL) return;

    axis_ferrorCtrl->pos_cmd = pos_cmd;
    axis_ferrorCtrl->pos_fb = pos_fb;
    axis_ferrorCtrl->vel_cmd = vel_cmd;

    /* 计算实际跟随误差 */
    axis_ferrorCtrl->ferror = fabs(pos_cmd - pos_fb);

    /* 计算动态跟随误差限制值
     * 根据速度命令动态调整误差限制
     * 速度越高，允许的误差越大
     */
    // 如果轴参数中最大速度限制大于0，则根据速度命令计算误差限制值
    if (axis_para->vel_limit > 0.0) {
        axis_ferrorCtrl->ferror_limit = axis_para->max_ferror * fabs(axis_ferrorCtrl->vel_cmd) / axis_para->vel_limit;
    } else {
        axis_ferrorCtrl->ferror_limit = 0.0;
    }
    /* 低速运动时计算出的跟随误差限制值可能接近0，就要引入一个最小跟随误差来判定 */
    if (axis_ferrorCtrl->ferror_limit < axis_para->min_ferror) {
        axis_ferrorCtrl->ferror_limit = axis_para->min_ferror;
    }

    /* 检查是否超出误差限制 */
    if (fabs(axis_ferrorCtrl->ferror) > axis_ferrorCtrl->ferror_limit) {
        axis_ferrorCtrl->error_flag = 1;
    } else {
        axis_ferrorCtrl->error_flag = 0;
    }
}
