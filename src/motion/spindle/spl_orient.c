#include "spl_orient.h"

#include "driver.h"
#include "driver_hal.h"
#include "spl_control.h"
#include <hmisysapi.h>

static bool orient_requested;
static bool orient_done;

void spl_orient_request(void)
{
    if (Driver_Orient(&spindle_driver) != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "主轴定向请求失败");
        orient_requested = false;
        orient_done = false;
        return;
    }

    orient_requested = true;
    orient_done = false;
}

void spl_orient_update(void)
{
    if (!orient_requested)
    {
        return;
    }

    if (Driver_Is_Orient_Done(&spindle_driver))
    {
        orient_done = true;
    }
}

bool spl_orient_is_done(void)
{
    return orient_done;
}

void spl_orient_release(void)
{
    if (Driver_Release_Orient(&spindle_driver) != HAL_DRIVER_SUCCESS)
    {
        LOG(LOG_LEVEL_ERR, "解除主轴定向失败");
        return;
    }

    orient_requested = false;
    orient_done = false;
}
