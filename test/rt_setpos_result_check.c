#include "../src/hal/rt_hal_adapter.c"
#include <assert.h>

int main(void)
{
    static char context_placeholder;
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_ERROR_NOT_RUNNING);
    s_ctx = (HalContext *)&context_placeholder; // Queue/query only, never dereferenced.
    s_snap_valid = 1;
    assert(rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0) == HAL_OK);
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_ERROR_BUSY);
    // Model the existing PDO consumer/report_req result storage.
    s_req_setpos_dirty[RT_HAL_AXIS_SP] = 0;
    s_req_fail_rc[RT_HAL_AXIS_SP][RT_REQ_SETPOS] = HAL_ERROR_BUS;
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_ERROR_BUS);
    assert(rt_hal_axis_set_pos(RT_HAL_AXIS_SP, 0.0) == HAL_OK);
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_ERROR_BUSY);
    s_req_setpos_dirty[RT_HAL_AXIS_SP] = 0;
    s_req_fail_rc[RT_HAL_AXIS_SP][RT_REQ_SETPOS] = HAL_OK;
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_OK);
    assert(rt_hal_axis_set_pos_result((RtHalAxisSlot)-1) == HAL_ERROR_ARGUMENT);
    s_snap_valid = 0;
    assert(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP) == HAL_ERROR_NOT_RUNNING);
    puts("PASS: production set-pos result query: pending / failure / retry / success / invalid snapshot");
    return 0;
}
