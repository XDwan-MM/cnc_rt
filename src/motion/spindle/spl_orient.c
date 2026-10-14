#include "spl_orient.h"

#include "rt_hal_adapter.h"
#include <hmisysapi.h>

/* 这几个函数只是把 HAL 的定向接口包一层，保持调用点形状；
 * HAL 每拍下发厂商控制字，完成位也从状态字上取。 */

bool spl_orient_request(void)
{
    const int rc = rt_hal_spindle_request_orient(RT_HAL_AXIS_SP);

    if (rc != HAL_OK)
    {
        /* 轴没使能时 HAL 回 NOT_ENABLED（调用方下一拍重试，不算失败）；
         * 只有"驱动器没配定向控制字"才真报出来。 */
        if (rc == HAL_ERROR_UNSUPPORTED)
        {
            LOG(LOG_LEVEL_ERR, "主轴定向请求失败：驱动器未配置定向控制字");
        }
        return false;
    }

    return true;
}

bool spl_orient_is_done(void)
{
    int32_t state = HAL_ORIENT_NONE;

    if (rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &state) != HAL_OK)
    {
        return false;
    }

    return state == HAL_ORIENT_DONE;
}

bool spl_orient_release(void)
{
    if (rt_hal_spindle_release_orient(RT_HAL_AXIS_SP) != HAL_OK)
    {
        LOG(LOG_LEVEL_ERR, "解除主轴定向失败");
        return false;
    }

    return true;
}

void spl_orient_abort(void)
{
    /* 急停/复位：控制字由 HAL 按急停覆盖，这里只让定向状态退回 NONE，
     * 免得下次 M19 以为还在定向中。 */
    (void)rt_hal_spindle_release_orient(RT_HAL_AXIS_SP);
}
