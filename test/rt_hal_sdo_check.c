#include "rt_hal_sdo.h"
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static int running = 1;
static int read_calls;
static int write_calls;
static uint32_t written;

int rt_hal_running(void) { return running; }
int rt_hal_sdo_read(int slave, uint16_t index, uint8_t subindex,
                    void* data, uint32_t size, uint32_t timeout_ms) {
    (void)timeout_ms;
    assert(slave == 2 && index == 0x606C && subindex == 0 && size == 4);
    struct timespec delay = {0, 20000000};
    nanosleep(&delay, NULL);
    uint32_t value = 0x12345678;
    memcpy(data, &value, size);
    ++read_calls;
    return HAL_OK;
}
int rt_hal_sdo_write(int slave, uint16_t index, uint8_t subindex,
                     const void* data, uint32_t size, uint32_t timeout_ms) {
    (void)timeout_ms;
    assert(slave == 2 && index == 0x6060 && subindex == 0 && size == 4);
    memcpy(&written, data, size);
    ++write_calls;
    return HAL_OK;
}

static int wait_result(uint32_t id, void* data, uint32_t size, int* result) {
    struct timespec delay = {0, 1000000};
    for (int i = 0; i < 200; ++i) {
        int rc = rt_hal_sdo_take_result(id, data, size, result);
        if (rc != HAL_ERROR_BUSY) return rc;
        nanosleep(&delay, NULL);
    }
    return HAL_ERROR_TIMEOUT;
}

int main(void) {
    pthread_t worker;
    uint32_t id[8], value = 0, out = 0xabcdef01;
    int result = -1;
    assert(rt_hal_sdo_prepare() == HAL_OK);
    assert(rt_hal_sdo_prepare() == HAL_ERROR_BUSY);
    assert(pthread_create(&worker, NULL, rt_hal_sdo_worker, NULL) == 0);
    assert(rt_hal_sdo_submit_read(2, 0x606C, 0, 4,
                                  RT_HAL_SDO_MAX_TIMEOUT_MS + 1, &id[0]) == HAL_ERROR_ARGUMENT);
    assert(rt_hal_sdo_submit_read(2, 0x606C, 0, 4, 1000, &id[0]) == HAL_OK);
    assert(wait_result(id[0], &value, 4, &result) == HAL_OK);
    assert(result == HAL_OK && value == 0x12345678 && read_calls == 1);
    assert(rt_hal_sdo_submit_write(2, 0x6060, 0, &out, 4, 1000, &id[0]) == HAL_OK);
    assert(wait_result(id[0], NULL, 0, &result) == HAL_OK);
    assert(result == HAL_OK && written == out && write_calls == 1);

    for (int i = 0; i < 8; ++i)
        assert(rt_hal_sdo_submit_read(2, 0x606C, 0, 4, 1000, &id[i]) == HAL_OK);
    assert(rt_hal_sdo_submit_read(2, 0x606C, 0, 4, 1000, &id[0]) == HAL_ERROR_BUSY);
    rt_hal_sdo_stop();
    assert(pthread_join(worker, NULL) == 0);
    for (int i = 0; i < 8; ++i) {
        assert(rt_hal_sdo_take_result(id[i], &value, 4, &result) == HAL_OK);
        assert(result == HAL_ERROR_STOPPED);
    }
    assert(rt_hal_sdo_submit_read(2, 0x606C, 0, 4, 1000, &id[0]) == HAL_ERROR_NOT_RUNNING);
    running = 0;
    assert(rt_hal_sdo_prepare() == HAL_ERROR_NOT_RUNNING);
    puts("rt_hal_sdo_check: passed");
    return 0;
}
