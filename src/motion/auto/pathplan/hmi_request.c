#include "hmi_request.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <hmirequestapi.h>
#include <hmisysapi.h>
#include "tapreturn.h"
#include "alarmapi.h"

/* 请求区指针 */
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

/*! 时间戳 */
static uint64_t hmi_request_monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000);
}

/*! 本版 RT 认识的动作：MST 下发、攻丝回退。其它非 0 动作一律拒绝 */
static bool hmi_request_action_known(uint32_t action)
{
    return action == HMI_REQUEST_ACTION_MST_DISPATCH
           || action == HMI_REQUEST_ACTION_TAP_RETRACT;
}

int hmi_request_take(HmiRequestSnapshot *snap, int32_t *errOut)
{
    if (snap == NULL) {
        return 0;
    }
    HmiRequestData *req = hmi_request_data();
    if (req == NULL) {
        return 0;
    }

    // 1、验证请求版本
    if (__atomic_load_n(&req->protocolVersion, __ATOMIC_ACQUIRE)
            != HMI_REQUEST_PROTOCOL_VERSION) {
        return 0;
    }

    // 2、动作判定：NONE = 无请求；未定义的动作 = HMI新增请求，但RT不能识别
    const uint32_t action = __atomic_load_n(&req->action, __ATOMIC_ACQUIRE);
    if (action == HMI_REQUEST_ACTION_NONE) {
        return 0;
    }
    if (!hmi_request_action_known(action)) {
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

    // 5、参数合法性：按动作分派。两个动作的参数在同一个联合体里但布局不同，
    //    拿一个动作的判据去看另一个动作的字节毫无意义（float 的字节几乎不会全 0）
    if (action == HMI_REQUEST_ACTION_MST_DISPATCH) {
        // 全 0 生成不出程序
        if (snap->params.mst.tValue == 0
                && snap->params.mst.sValue == 0
                && snap->params.mst.mValue == 0) {
            if (errOut) {
                *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
            }
            return -2;
        }
    } else {    // HMI_REQUEST_ACTION_TAP_RETRACT
        const float pitch = snap->params.tapret.pitchMm;
        // !(pitch > 0.0f) 同时挡掉 NaN；isfinite 再挡 ±inf
        if (!(pitch > 0.0f) || !isfinite(pitch)
                || snap->params.tapret.sValue <= 0
                || (snap->params.tapret.mValue != 3 && snap->params.tapret.mValue != 4)) {
            if (errOut) {
                *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
            }
            return -2;
        }
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

/* ---------------------------------------------------------------------------
 * todo：攻丝回退程序串生成：**空桩，待实现**
 *
 * 当前固定返回 0，调用方（parser.cpp 的 MODE_MDI 分支）据此回执
 * REJECTED + HMI_REQUEST_ERROR_PROGRAM，界面提示"攻丝回退被拒绝（错误码 5）"，
 * --------------------------------------------------------------------------- */
int hmi_tapret_build_program(const HmiRequestTapRetParams *tapret, char *out, int outLen)
{
    if (tapret == NULL || out == NULL || outLen <= 0) {
        return 0;
    }


    if (!tapreturn_ready()) {
        setAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_NO_RECORD);
        LOG(LOG_LEVEL_WARN, "攻丝回退被拒：当前不可回退（无记录 / 刀不在孔内 / 位置基准未建立）");
        return 0;
    }

    if (!tapreturn_page_pitch_matches(tapret->pitchMm)) {
        setAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_PITCH);
        return HMI_TAPRET_REJECT_PARAM;
    }
    if(!tapreturn_page_direction_matches(tapret->mValue))
    {
        setAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_DIR);
        return HMI_TAPRET_REJECT_PARAM;
    }
    if(!tapreturn_page_speed_ok(tapret->sValue))
    {
        setAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_SPEED);
        return HMI_TAPRET_REJECT_PARAM;
    }
    // ③ 生成（行式与记录自洽校验都在模块里）
    char buf[HMI_TAPRET_PROGRAM_BUF_LEN];
    if (tapreturn_generate_with_speed(tapret->sValue, buf, (int)sizeof(buf))
            != GNC_OPERATOR_SUCCESS) {
        return 0;
    }

    const int len = snprintf(out, outLen, "%s\n", buf);
    if (len <= 0 || len >= outLen) {
        return 0;   // 调用方缓冲不足（与 hmi_mst_build_program 同一约定）
    }
    tapreturn_clear_alarms();
    return len;
}
