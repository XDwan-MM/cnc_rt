#include "spl_orient.h"

#include "driver.h"
#include "driver_hal.h"
#include "spl_control.h"
#include <hmisysapi.h>

bool spl_orient_request(void)
{
    if (Driver_Orient(&spindle_driver) != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "主轴定向请求失败");
        return false;
    }

    return true;
}

bool spl_orient_is_done(void)
{
    return Driver_Is_Orient_Done(&spindle_driver);
}

bool spl_orient_release(void)
{
    if (Driver_Release_Orient(&spindle_driver) != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "解除主轴定向失败");
        return false;
    }

    return true;
}

void spl_orient_abort(void)
{
    Driver_Abort_Orient(&spindle_driver);
}
