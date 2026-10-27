/*
 * SDO 请求队列与工作线程。
 *
 * 为什么需要这一层：HAL 的 SDO 是阻塞传输（一次最多 4 字节，要等驱动器应答），
 * 绝对不能从 PDO 线程或运动周期线程里调用。所以业务侧只把请求放进队列，
 * 由独立的工作线程串行执行，业务侧拿 request_id 轮询结果。
 *
 * 队列语义：
 *   - 最多 RT_SDO_QUEUE_SIZE 笔未取结果，占满时提交返回 HAL_ERROR_BUSY；
 *   - 每笔请求带单调递增的 id，工作线程按 id 升序处理（先来先做）；
 *   - 取结果是消费式的：取走后槽位立即释放，该 id 随即失效。
 *
 * 锁的范围：只保护队列本身。工作线程在锁外执行真正的 SDO 传输，
 * 所以一笔慢 SDO 不会挡住别人提交新请求，也不会卡住 rt_hal_sdo_stop()。
 * 代价是停机时 worker 可能正在传输中，见 rt_hal_sdo_stop() 的说明。
 */
#include "rt_hal_sdo.h"
#include "rt_hal_adapter.h"
#include <pthread.h>
#include <string.h>

#define RT_SDO_QUEUE_SIZE 8

/* 一笔请求的生命周期：FREE → QUEUED → ACTIVE → DONE →（取走后）FREE。 */
typedef enum { SDO_FREE, SDO_QUEUED, SDO_ACTIVE, SDO_DONE } SdoState;
/* 寻址方式：按 RT 轴槽（index 走 0x6000..0x67FF），或按物理从站（原始对象号）。 */
typedef enum { SDO_AXIS, SDO_SLAVE } SdoTarget;
typedef struct {
    SdoState state;
    uint32_t id;                /* 单调递增，worker 据此做 FIFO */
    int target_id;              /* SDO_AXIS 时是 RT 轴槽；SDO_SLAVE 时是从站号 */
    SdoTarget target;
    uint16_t index;
    uint8_t subindex;
    uint8_t write;
    uint32_t size;              /* 只接受 1 / 2 / 4 字节 */
    uint32_t timeout_ms;
    uint8_t data[4];            /* HAL 单次 SDO 最多 4 字节 */
    int result;
} SdoRequest;

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_ready = PTHREAD_COND_INITIALIZER;   /* 有新请求入队时唤醒 worker */
static SdoRequest s_requests[RT_SDO_QUEUE_SIZE];
static uint32_t s_next_id;
static int s_accepting;   /* 0 = 停止接收新请求 */
static int s_stopping;    /* 1 = worker 应当退出循环 */

/*
 * 开始接收 SDO 请求。
 *
 * 必须在 HAL 已经 start 之后调用；重复调用返回 BUSY 而不是重置队列，
 * 避免把别人正在等的请求清掉。
 *
 * @return 0 成功；HAL_ERROR_BUSY 已经在接收；HAL_ERROR_NOT_RUNNING 尚未 start
 */
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

/*
 * 停止接收请求，并唤醒所有等待者。
 *
 * 排队中的请求直接标成"已完成 + HAL_ERROR_STOPPED"，让提交者不会永远等下去；
 * 已经在传输中的那一笔没法打断（HAL 的 SDO 是阻塞调用），
 * 其结果在 worker 回来时被改写成 STOPPED，同样不会让调用方拿到过期结论。
 *
 * 本函数返回时 worker 可能仍在收尾，调用方须等它真正退出后才能销毁 context。
 */
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

/*
 * 提交一笔请求的公共实现。
 *
 * 参数检查全部在加锁之前完成：非法输入不该占用队列槽位。
 * target_id 的含义由 target 决定（RT 轴槽 / 物理从站），两者都不做越界换算，
 * 换算只发生在 worker 调用适配层的时候。
 *
 * @param request_id 带出请求号，调用方用它来取结果
 * @param write      非零表示写；写操作要求 data 非空
 * @return 0 已入队；HAL_ERROR_ARGUMENT 参数非法；HAL_ERROR_NOT_RUNNING 未接收；
 *         HAL_ERROR_BUSY 队列已满
 */
static int submit(SdoTarget target, int target_id, uint16_t index, uint8_t subindex,
                   const void* data, uint32_t size, uint32_t timeout_ms,
                   uint32_t* request_id, int write) {
    if (!request_id || (size != 1 && size != 2 && size != 4) ||
        !timeout_ms || timeout_ms > RT_HAL_SDO_MAX_TIMEOUT_MS ||
        target_id < 0 || (write && !data)) return HAL_ERROR_ARGUMENT;
    if (target == SDO_AXIS &&
        (target_id >= RT_HAL_AXIS_COUNT || index < 0x6000u || index > 0x67FFu))
        return HAL_ERROR_ARGUMENT;
    pthread_mutex_lock(&s_mutex);
    if (!s_accepting) {
        pthread_mutex_unlock(&s_mutex);
        return HAL_ERROR_NOT_RUNNING;
    }
    for (int i = 0; i < RT_SDO_QUEUE_SIZE; ++i) {
        SdoRequest* req = &s_requests[i];
        if (req->state != SDO_FREE) continue;
        memset(req, 0, sizeof(*req));
        /* id 从 1 开始，0 保留给"无效请求号" */
        if (++s_next_id == 0) ++s_next_id;
        req->id = s_next_id;
        req->state = SDO_QUEUED;
        req->target_id = target_id;
        req->target = target;
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

/* 按 RT 轴槽读标准轴对象（0x6000..0x67FF）。 */
int rt_hal_sdo_submit_read(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                           uint32_t size, uint32_t timeout_ms, uint32_t* request_id) {
    return submit(SDO_AXIS, slot, index, subindex, NULL, size, timeout_ms, request_id, 0);
}

/* 按 RT 轴槽写标准轴对象（0x6000..0x67FF）。 */
int rt_hal_sdo_submit_write(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                            const void* data, uint32_t size, uint32_t timeout_ms,
                            uint32_t* request_id) {
    return submit(SDO_AXIS, slot, index, subindex, data, size, timeout_ms, request_id, 1);
}

/* 按物理从站读，对象号原样使用，不做轴偏移。 */
int rt_hal_sdo_submit_slave_read(int slave_pos, uint16_t index, uint8_t subindex,
                                 uint32_t size, uint32_t timeout_ms, uint32_t* request_id) {
    return submit(SDO_SLAVE, slave_pos, index, subindex, NULL, size, timeout_ms, request_id, 0);
}

/* 按物理从站写，对象号原样使用，不做轴偏移。 */
int rt_hal_sdo_submit_slave_write(int slave_pos, uint16_t index, uint8_t subindex,
                                  const void* data, uint32_t size, uint32_t timeout_ms,
                                  uint32_t* request_id) {
    return submit(SDO_SLAVE, slave_pos, index, subindex, data, size, timeout_ms, request_id, 1);
}

/*
 * 取一笔请求的结果。这是消费式读取：取走后槽位立即释放，该 id 随之失效。
 *
 * 读请求成功时才把数据拷回调用方缓冲，失败时缓冲保持不变；
 * 读的 size 必须与提交时一致（否则说明调用方拿错了请求），写的 size 必须为 0。
 *
 * @param request_id 提交时带出的请求号
 * @param result     带出这一笔的执行结果（HAL 错误码或 0）
 * @return 0 已取走；HAL_ERROR_BUSY 还没做完；HAL_ERROR_ARGUMENT 请求号或参数不匹配
 */
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

/*
 * SDO 工作线程主循环。
 *
 * 每轮：在锁内挑出 id 最小的排队请求并标成 ACTIVE，然后**释放锁**去执行传输，
 * 回来后再加锁写结果。锁外传输是这一层的关键——否则一笔慢 SDO 会让所有提交者
 * 和 rt_hal_sdo_stop() 一起卡住。
 *
 * 只有 s_stopping 会让它返回；届时在传输中的那一笔结果会被改写成 STOPPED，
 * 避免调用方拿着停机前的旧结论继续走。
 */
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

        /* 锁外执行：这一笔可能要等几百毫秒到 timeout_ms */
        int rc;
        if (req.target == SDO_AXIS)
            rc = req.write ?
                rt_hal_axis_sdo_write((RtHalAxisSlot)req.target_id, req.index, req.subindex,
                                      req.data, req.size, req.timeout_ms) :
                rt_hal_axis_sdo_read((RtHalAxisSlot)req.target_id, req.index, req.subindex,
                                     req.data, req.size, req.timeout_ms);
        else
            rc = req.write ?
                rt_hal_sdo_write(req.target_id, req.index, req.subindex,
                                 req.data, req.size, req.timeout_ms) :
                rt_hal_sdo_read(req.target_id, req.index, req.subindex,
                                req.data, req.size, req.timeout_ms);

        pthread_mutex_lock(&s_mutex);
        s_requests[slot].result = s_stopping ? HAL_ERROR_STOPPED : rc;
        if (!s_stopping && !req.write && rc == HAL_OK)
            memcpy(s_requests[slot].data, req.data, req.size);
        s_requests[slot].state = SDO_DONE;
        pthread_mutex_unlock(&s_mutex);
    }
}
