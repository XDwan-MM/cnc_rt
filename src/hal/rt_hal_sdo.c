#include "rt_hal_sdo.h"
#include "rt_hal_adapter.h"
#include <pthread.h>
#include <string.h>

#define RT_SDO_QUEUE_SIZE 8

typedef enum { SDO_FREE, SDO_QUEUED, SDO_ACTIVE, SDO_DONE } SdoState;
typedef struct {
    SdoState state;
    uint32_t id;
    int slave_pos;
    uint16_t index;
    uint8_t subindex;
    uint8_t write;
    uint32_t size;
    uint32_t timeout_ms;
    uint8_t data[4];
    int result;
} SdoRequest;

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_ready = PTHREAD_COND_INITIALIZER;
static SdoRequest s_requests[RT_SDO_QUEUE_SIZE];
static uint32_t s_next_id;
static int s_accepting;
static int s_stopping;

int rt_hal_sdo_prepare(void) {
    pthread_mutex_lock(&s_mutex);
    if (s_accepting || !rt_hal_running()) {
        const int rc = s_accepting ? HAL_ERROR_BUSY : HAL_ERROR_NOT_RUNNING;
        pthread_mutex_unlock(&s_mutex);
        return rc;
    }
    memset(s_requests, 0, sizeof(s_requests));
    s_stopping = 0;
    s_accepting = 1;
    pthread_mutex_unlock(&s_mutex);
    return HAL_OK;
}

void rt_hal_sdo_stop(void) {
    pthread_mutex_lock(&s_mutex);
    s_accepting = 0;
    s_stopping = 1;
    for (int i = 0; i < RT_SDO_QUEUE_SIZE; ++i) {
        if (s_requests[i].state == SDO_QUEUED) {
            s_requests[i].state = SDO_DONE;
            s_requests[i].result = HAL_ERROR_STOPPED;
        }
    }
    pthread_cond_broadcast(&s_ready);
    pthread_mutex_unlock(&s_mutex);
}

static int submit(int slave_pos, uint16_t index, uint8_t subindex,
                   const void* data, uint32_t size, uint32_t timeout_ms,
                   uint32_t* request_id, int write) {
    if (!request_id || (size != 1 && size != 2 && size != 4) ||
        !timeout_ms || timeout_ms > RT_HAL_SDO_MAX_TIMEOUT_MS ||
        slave_pos < 0 || (write && !data)) return HAL_ERROR_ARGUMENT;
    pthread_mutex_lock(&s_mutex);
    if (!s_accepting) {
        pthread_mutex_unlock(&s_mutex);
        return HAL_ERROR_NOT_RUNNING;
    }
    for (int i = 0; i < RT_SDO_QUEUE_SIZE; ++i) {
        SdoRequest* req = &s_requests[i];
        if (req->state != SDO_FREE) continue;
        memset(req, 0, sizeof(*req));
        if (++s_next_id == 0) ++s_next_id;
        req->id = s_next_id;
        req->state = SDO_QUEUED;
        req->slave_pos = slave_pos;
        req->index = index;
        req->subindex = subindex;
        req->write = write;
        req->size = size;
        req->timeout_ms = timeout_ms;
        if (write) memcpy(req->data, data, size);
        *request_id = req->id;
        pthread_cond_signal(&s_ready);
        pthread_mutex_unlock(&s_mutex);
        return HAL_OK;
    }
    pthread_mutex_unlock(&s_mutex);
    return HAL_ERROR_BUSY;
}

int rt_hal_sdo_submit_read(int slave_pos, uint16_t index, uint8_t subindex,
                           uint32_t size, uint32_t timeout_ms, uint32_t* request_id) {
    return submit(slave_pos, index, subindex, NULL, size, timeout_ms, request_id, 0);
}

int rt_hal_sdo_submit_write(int slave_pos, uint16_t index, uint8_t subindex,
                            const void* data, uint32_t size, uint32_t timeout_ms,
                            uint32_t* request_id) {
    return submit(slave_pos, index, subindex, data, size, timeout_ms, request_id, 1);
}

int rt_hal_sdo_take_result(uint32_t request_id, void* data, uint32_t size, int* result) {
    if (!request_id || !result) return HAL_ERROR_ARGUMENT;
    pthread_mutex_lock(&s_mutex);
    for (int i = 0; i < RT_SDO_QUEUE_SIZE; ++i) {
        SdoRequest* req = &s_requests[i];
        if (req->state == SDO_FREE || req->id != request_id) continue;
        if ((!req->write && (!data || size != req->size)) ||
            (req->write && size != 0)) {
            pthread_mutex_unlock(&s_mutex);
            return HAL_ERROR_ARGUMENT;
        }
        if (req->state != SDO_DONE) {
            pthread_mutex_unlock(&s_mutex);
            return HAL_ERROR_BUSY;
        }
        *result = req->result;
        if (!req->write && req->result == HAL_OK) memcpy(data, req->data, size);
        memset(req, 0, sizeof(*req));
        pthread_mutex_unlock(&s_mutex);
        return HAL_OK;
    }
    pthread_mutex_unlock(&s_mutex);
    return HAL_ERROR_ARGUMENT;
}

void* rt_hal_sdo_worker(void* arg) {
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&s_mutex);
        int slot = -1;
        while (!s_stopping) {
            for (int i = 0; i < RT_SDO_QUEUE_SIZE; ++i) {
                if (s_requests[i].state == SDO_QUEUED &&
                    (slot < 0 || s_requests[i].id < s_requests[slot].id)) slot = i;
            }
            if (slot >= 0) break;
            pthread_cond_wait(&s_ready, &s_mutex);
        }
        if (s_stopping) {
            pthread_mutex_unlock(&s_mutex);
            return NULL;
        }
        SdoRequest req = s_requests[slot];
        s_requests[slot].state = SDO_ACTIVE;
        pthread_mutex_unlock(&s_mutex);

        const int rc = req.write ?
            rt_hal_sdo_write(req.slave_pos, req.index, req.subindex,
                             req.data, req.size, req.timeout_ms) :
            rt_hal_sdo_read(req.slave_pos, req.index, req.subindex,
                            req.data, req.size, req.timeout_ms);

        pthread_mutex_lock(&s_mutex);
        s_requests[slot].result = s_stopping ? HAL_ERROR_STOPPED : rc;
        if (!s_stopping && !req.write && rc == HAL_OK)
            memcpy(s_requests[slot].data, req.data, req.size);
        s_requests[slot].state = SDO_DONE;
        pthread_mutex_unlock(&s_mutex);
    }
}
