#include "backend/greemaster/master.h"
#include "hal_c_api.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int main(void) {
    assert(hal_context_request_stop(NULL) == HAL_ERROR_ARGUMENT);
    MasterConfig cfg = {0};
    cfg.cycle_us = 1000;
    assert(ethercat_init(&cfg) == 0);

    uint8_t mode = 9;
    assert(Master_SdoTransfer(2, 0x6060, 0, &mode, 1, 1, 1000) == 0);
    mode = 0;
    assert(Master_SdoTransfer(2, 0x6061, 0, &mode, 1, 0, 1000) == 0);
    assert(mode == 9);

    uint32_t pos = 12345;
    assert(Master_SdoTransfer(3, 0x607A, 0, &pos, 4, 1, 1000) == 0);
    pos = 0;
    assert(Master_SdoTransfer(3, 0x6064, 0, &pos, 4, 0, 1000) == 0);
    assert(pos == 12345);
    assert(Master_SdoTransfer(3, 0x6861, 0, &mode, 1, 0, 1000) == -3);

    Master_RequestStop();
    assert(Master_SdoTransfer(2, 0x6061, 0, &mode, 1, 0, 1000) == -4);
    assert(ethercat_close() == 0);
    puts("rt_hal_sim_sdo_check: passed");
    return 0;
}
