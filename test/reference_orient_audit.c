#include <assert.h>
#include <stdio.h>
#include <hmisysapi.h>
#undef LOG
#define LOG(level, ...) ((void)0)
#include REFERENCE_DRIVER

hal_retcode_t Ethercat_Driver_Write_Controlword(hal_driver_t *d, uint16_t value)
{ d->control_word = value; return HAL_DRIVER_SUCCESS; }
hal_retcode_t Ethercat_Driver_Write_Mode(hal_driver_t *d, drv_mode_t value)
{ d->req_mode = value; return HAL_DRIVER_SUCCESS; }
hal_retcode_t Ethercat_Driver_Write_Target_Position(hal_driver_t *d, int32_t value)
{ d->target_pos = value; return HAL_DRIVER_SUCCESS; }
hal_retcode_t Ethercat_Driver_Write_Target_Velocity(hal_driver_t *d, int32_t value)
{ d->target_vel = value; return HAL_DRIVER_SUCCESS; }

int main(void)
{
    hal_driver_t d = {0};
    Driver_Init(&d);
    d.initialized = true;
    d.status_word = 0x0437;
    d.act_mode = DRV_MODE_CSV;
    d.state = d.target_state = DRV_STATE_OPERATION_ENABLED;
    assert(Driver_Orient(&d) == HAL_DRIVER_SUCCESS);
    for (int i = 0; i < 30; ++i) {
        assert(Driver_Cyclic_Process(&d) == HAL_DRIVER_SUCCESS);
        assert(!Driver_Is_Orient_Done(&d));
    }
    puts("REFERENCE: bit10 high for 30 cycles -> still RUNNING");
    d.status_word = 0x0037;
    assert(Driver_Cyclic_Process(&d) == HAL_DRIVER_SUCCESS);
    assert(!Driver_Is_Orient_Done(&d));
    d.status_word = 0x0437;
    assert(Driver_Cyclic_Process(&d) == HAL_DRIVER_SUCCESS);
    assert(Driver_Is_Orient_Done(&d));
    puts("REFERENCE: subsequent low -> high -> DONE");
    return 0;
}
