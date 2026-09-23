#include "hmi_request.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <hmirequestapi.h>

/* 请求区指针：懒加载（不改 global.c/global.h 的全局变量表） */
static HmiRequestData *s_hmi_req = NULL;
static bool s_hmi_req_probed = false;

HmiRequestData *hmi_request_data(void)
{
    if (!s_hmi_req_probed) {
        s_hmi_req = getCncHmiRequestDataPtr();
        s_hmi_req_probed = true;
    }
    return s_hmi_req;
}

/*! 系统级单调毫秒：必须与 HMI 侧同一时基（HMI 用 CLOCK_MONOTONIC 刷时间戳） */
static uint64_t hmi_request_monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000);
}

int hmi_request_take(uint32_t expectedAction, HmiRequestSnapshot *snap, int32_t *errOut)
{
    if (snap == NULL) {
        return 0;
    }
    HmiRequestData *req = hmi_request_data();
    if (req == NULL) {
        return 0;
    }

    // 1、版本闸门：不相等不消费（正常部署下不会发生，按无请求处理）
    if (__atomic_load_n(&req->protocolVersion, __ATOMIC_ACQUIRE)
            != HMI_REQUEST_PROTOCOL_VERSION) {
        return 0;
    }

    // 2、动作判定：NONE = 无请求；其它未定义值 = 界面已有新功能而本版 RT 不认识
    const uint32_t action = __atomic_load_n(&req->action, __ATOMIC_ACQUIRE);
    if (action == HMI_REQUEST_ACTION_NONE) {
        return 0;
    }
    if (action != expectedAction) {
        snap->seq = __atomic_load_n(&req->requestSequence, __ATOMIC_ACQUIRE);
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_UNKNOWN_ACTION;
        }
        return -2;
    }

    // 3、读序号 → 整块拷贝参数/时间戳 → 复查序号（HMI 是先写数据、最后 RELEASE 写序号）
    const uint32_t seq1 = __atomic_load_n(&req->requestSequence, __ATOMIC_ACQUIRE);
    snap->seq = seq1;
    snap->action = action;
    snap->timestampMs = __atomic_load_n(&req->requestTimestampMs, __ATOMIC_ACQUIRE);
    memcpy(&snap->params, &req->params, sizeof(snap->params));
    if (seq1 != __atomic_load_n(&req->requestSequence, __ATOMIC_ACQUIRE)) {
        return -1;      // 读到一半被 HMI 改写，本轮不执行
    }

    // 4、时效：HMI 在画面内每 150ms 刷时间戳，超时说明 HMI 已离开画面/崩溃/卡死
    if (!HMI_REQUEST_IS_FRESH(req, hmi_request_monotonic_ms())) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_STALE;
        }
        return -2;
    }

    // 5、参数合法性：全 0 生成不出程序（HMI 侧本就不会挂全 0，这里防御）
    if (snap->params.mst.tValue == 0
            && snap->params.mst.sValue == 0
            && snap->params.mst.mValue == 0) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
        }
        return -2;
    }

    return 1;
}

void hmi_request_ack(uint32_t seq, uint32_t state, int32_t errorCode)
{
    HmiRequestData *req = hmi_request_data();
    if (req == NULL) {
        return;
    }

    // 只写 RT 自己的三个字段；state 最后写（发布点）
    __atomic_store_n(&req->acknowledgedSequence, seq, __ATOMIC_RELEASE);
    __atomic_store_n(&req->errorCode, errorCode, __ATOMIC_RELAXED);
    __atomic_store_n(&req->state, state, __ATOMIC_RELEASE);
}

int hmi_mst_build_program(const HmiRequestMstParams *mst, char *out, int outLen)
{
    if (mst == NULL || out == NULL || outLen <= 0) {
        return 0;
    }

    // int32 最坏每项 11 位，三项加空格与换行约 37 字节，本地缓冲固定 64 必然够
    char buf[HMI_MST_PROGRAM_BUF_LEN];
    int n = 0;

    if (mst->tValue != 0) {
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "T%d", (int)mst->tValue);
    }
    if (mst->sValue != 0) {
        if (n > 0) {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " ");
        }
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "S%d", (int)mst->sValue);
    }
    if (mst->mValue != 0) {
        if (n > 0) {
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " ");
        }
        n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                      ((mst->mValue < 100) ? "M%02d" : "M%d"), (int)mst->mValue);
    }
    if (n > 0) {
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "\n");
    }

    if (n <= 0 || n >= outLen) {
        return 0;       // 无有效内容或调用方缓冲不足
    }
    memcpy(out, buf, (size_t)n + 1U);   // 连结尾 '\0' 一起带过去
    return n;
}
