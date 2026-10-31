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
        // 只缓存成功取到的指针：RT 若早于 HMI 初始化共享内存，后续调用还能重试
        if (s_hmi_req != NULL) {
            s_hmi_req_probed = true;
        }
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

/*! 本版 RT 认识的动作：MST 下发、攻丝回退、对刀启动。其它非 0 动作一律拒绝 */
static bool hmi_request_action_known(uint32_t action)
{
    return action == HMI_REQUEST_ACTION_MST_DISPATCH
           || action == HMI_REQUEST_ACTION_TAP_RETRACT
           || action == HMI_REQUEST_ACTION_TOOL_MEASURE;
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

    // 5、参数合法性：按动作分派。各动作的参数在同一个联合体里但布局不同，
    //    拿一个动作的判据去看另一个动作的字节毫无意义（float 的字节几乎不会全 0）。
    //    注意：这里加动作时要与 hmi_request_action_known() 一起改
    switch (action) {
    case HMI_REQUEST_ACTION_MST_DISPATCH:
        // 全 0 生成不出程序
        if (snap->params.mst.tValue == 0
                && snap->params.mst.sValue == 0
                && snap->params.mst.mValue == 0) {
            if (errOut) {
                *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
            }
            return -2;
        }
        break;
    case HMI_REQUEST_ACTION_TAP_RETRACT: {
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
        break;
    }
    case HMI_REQUEST_ACTION_TOOL_MEASURE:
        // 刀号必须有效；补偿号 0 合法（0 = 把刀长写入外部坐标系 EXT）
        if (snap->params.toolMeasure.toolNo <= 0
                || snap->params.toolMeasure.offsetNo < 0) {
            if (errOut) {
                *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
            }
            return -2;
        }
        break;
    default:
        break;
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

/* ===================== PROGRAM_START_LINE（仅 MODE_MEM） ===================== */

/* 已做过首按报警的序号（0=无） */
static uint32_t s_program_start_confirmed_seq = 0;
/* 最近一次看到的 action=5 序号 */
static uint32_t s_program_start_last_seq = 0;
/* 当前报警 CH_MSG_0000003 是否是本模块挂上的（避免误清 rerun 的报警） */
static bool s_program_start_warning_active = false;
/* 已通过首按确认、等待 parser_set_content(MODE_MEM) 提交的快照 */
static bool s_program_start_pending_valid = false;
static HmiProgramStartSnapshot s_program_start_pending;

void hmi_request_publish_capabilities(void)
{
    HmiRequestData *req = hmi_request_data();
    if (req == NULL) {
        return;
    }
    // 用 OR 保留其它能力位，便于后续 RT 追加能力
    __atomic_fetch_or(&req->capabilityFlags,
                      HMI_REQUEST_CAPABILITY_PROGRAM_START_LINE,
                      __ATOMIC_RELEASE);
}

/*! 统计程序文件的物理行数（与 ItpSetFile/QTextStream::readLine 的行号口径一致：
 *  以 '\n' 分行，末尾换行不额外产生空行；空文件为 0 行） */
static bool hmi_program_line_count(const char *path, int *lineCount)
{
    if (path == NULL || lineCount == NULL) {
        return false;
    }
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        return false;
    }

    char buf[4096];
    size_t n = 0;
    int count = 0;
    bool hasData = false;
    bool lastIsNewline = false;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        hasData = true;
        for (size_t i = 0; i < n; ++i) {
            if (buf[i] == '\n') {
                ++count;
            }
        }
        lastIsNewline = (buf[n - 1] == '\n');
    }
    const bool readError = (ferror(fp) != 0);
    fclose(fp);
    if (readError) {
        return false;
    }
    if (hasData && !lastIsNewline) {
        ++count;
    }
    *lineCount = count;
    return true;
}

bool hmi_request_program_start_validate_file(const char *path, int32_t startLine)
{
    if (path == NULL || path[0] == '\0' || startLine < 0) {
        return false;
    }
    int lineCount = 0;
    if (!hmi_program_line_count(path, &lineCount)) {
        return false;
    }
    return startLine < lineCount;
}

/*! 读取并校验一条 PROGRAM_START_LINE 请求
 *  @return 1=有效快照；0=无请求/非本动作；-1=HMI 正在写入；-2=拒绝（已回执） */
static int hmi_request_take_program_start(HmiProgramStartSnapshot *snap, int32_t *errOut)
{
    if (errOut) {
        *errOut = HMI_REQUEST_ERROR_NONE;
    }
    if (snap == NULL) {
        return 0;
    }
    HmiRequestData *req = hmi_request_data();
    if (req == NULL) {
        return 0;
    }

    if (__atomic_load_n(&req->protocolVersion, __ATOMIC_ACQUIRE)
            != HMI_REQUEST_PROTOCOL_VERSION) {
        return 0;
    }
    const uint32_t action = __atomic_load_n(&req->action, __ATOMIC_ACQUIRE);
    if (action == HMI_REQUEST_ACTION_NONE) {
        return 0;
    }
    if (action != HMI_REQUEST_ACTION_PROGRAM_START_LINE) {
        // MEM 下其它请求动作不处理：按正常启动走，不在 MEM 执行 MDI 类动作
        return 0;
    }

    // 读序号 → 参数 → 复查序号与发布标记（HMI 是先写数据、最后 RELEASE 写序号与 tag）
    const uint32_t seq1 = __atomic_load_n(&req->requestSequence, __ATOMIC_ACQUIRE);
    snap->seq = seq1;
    snap->startLine = req->params.programStart.startLine;
    snap->pathHash = req->params.programStart.pathHash;
    const uint32_t tag = __atomic_load_n(&req->publishTag, __ATOMIC_ACQUIRE);
    if (seq1 == 0U || tag != seq1
            || seq1 != __atomic_load_n(&req->requestSequence, __ATOMIC_ACQUIRE)) {
        return -1;
    }

    if (!HMI_REQUEST_IS_FRESH(req, hmi_request_monotonic_ms())) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_STALE;
        }
        hmi_request_ack(seq1, HMI_REQUEST_STATE_TIMEOUT, HMI_REQUEST_ERROR_STALE);
        return -2;
    }
    if (snap->startLine < 0) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
        }
        hmi_request_ack(seq1, HMI_REQUEST_STATE_REJECTED,
                        HMI_REQUEST_ERROR_PARAM_INVALID);
        return -2;
    }

    // 路径指纹必须与即将打开的 ncFilePath 一致，防止旧路径+新行号
    char path[NCFILEPATH_CHAR_LEN];
    memset(path, 0xFF, sizeof(path));
    if (getNcFilePath(path) != 0 || path[0] == '\0'
            || memchr(path, '\0', sizeof(path)) == NULL) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_PROGRAM;
        }
        hmi_request_ack(seq1, HMI_REQUEST_STATE_REJECTED, HMI_REQUEST_ERROR_PROGRAM);
        return -2;
    }
    if (hmiRequestPathHash32(path) != snap->pathHash) {
        if (errOut) {
            *errOut = HMI_REQUEST_ERROR_PARAM_INVALID;
        }
        hmi_request_ack(seq1, HMI_REQUEST_STATE_REJECTED,
                        HMI_REQUEST_ERROR_PARAM_INVALID);
        return -2;
    }

    // 行号/文件行数在 parser_set_content(MODE_MEM) 提交前校验：
    // 这里是 1ms 周期，不能做整文件行数统计
    memcpy(snap->path, path, sizeof(snap->path));
    return 1;
}

int hmi_request_program_start_decide(int32_t *errOut)
{
    if (errOut) {
        *errOut = HMI_REQUEST_ERROR_NONE;
    }
    // 每次循环启动重新决策：先丢弃上一次未提交的 pending
    s_program_start_pending_valid = false;
    memset(&s_program_start_pending, 0, sizeof(s_program_start_pending));

    HmiProgramStartSnapshot snap;
    memset(&snap, 0, sizeof(snap));
    int32_t reason = HMI_REQUEST_ERROR_NONE;
    const int rc = hmi_request_take_program_start(&snap, &reason);
    if (errOut) {
        *errOut = reason;
    }

    if (rc != 1) {
        if (rc == 0) {
            // 无起始行请求：释放本模块的确认状态；若报警是本模块挂的则清掉
            if (s_program_start_warning_active) {
                delAlarm(ALARM_CH, ALARM_WARNING, 3);
                s_program_start_warning_active = false;
            }
            s_program_start_confirmed_seq = 0;
            s_program_start_last_seq = 0;
        } else if (rc == -2) {
            if (s_program_start_warning_active) {
                delAlarm(ALARM_CH, ALARM_WARNING, 3);
                s_program_start_warning_active = false;
            }
            s_program_start_confirmed_seq = 0;
            s_program_start_last_seq = 0;
            LOG(LOG_LEVEL_WARN, "HMI起始行请求未执行 reason=%d", (int)reason);
        } else {
            LOG(LOG_LEVEL_WARN, "HMI起始行请求读取期间被改写，本轮不启动");
        }
        return rc;
    }

    // 序号变化：上一次的首按确认作废
    if (snap.seq != s_program_start_last_seq) {
        s_program_start_confirmed_seq = 0;
        s_program_start_last_seq = snap.seq;
    }
    if (snap.startLine == 0) {
        if (s_program_start_warning_active) {
            delAlarm(ALARM_CH, ALARM_WARNING, 3);
            s_program_start_warning_active = false;
        }
        s_program_start_confirmed_seq = 0;
        s_program_start_last_seq = 0;
        return 0;
    }
    if (snap.startLine > 0 && snap.seq != s_program_start_confirmed_seq) {
        // 首按确认：只报警、不启动；第二次按循环启动才执行

        setAlarm(ALARM_CH, ALARM_WARNING, 3);
        // 与断点再启动一致：把运行行对到目标行；复位清 0 后页面据此把光标回到开头
        setVarChanInt(0, VAR_CHAN_RUN_ROW, snap.startLine, 0);
        s_program_start_warning_active = true;
        s_program_start_confirmed_seq = snap.seq;
        LOG(LOG_LEVEL_WARN,
            "从指定行启动：请确认刀具安全后再次按循环启动 line=%d",
            (int)snap.startLine);
        return -1;
    }

    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    s_program_start_warning_active = false;
    s_program_start_confirmed_seq = 0;      // 本次启动已消费确认
    s_program_start_pending = snap;
    s_program_start_pending_valid = true;
    return 1;
}

bool hmi_request_program_start_pending(HmiProgramStartSnapshot *out)
{
    if (!s_program_start_pending_valid || out == NULL) {
        return false;
    }
    *out = s_program_start_pending;
    return true;
}

void hmi_request_program_start_commit(void)
{
    s_program_start_pending_valid = false;
    memset(&s_program_start_pending, 0, sizeof(s_program_start_pending));
    s_program_start_confirmed_seq = 0;
}

void hmi_request_program_start_cancel(void)
{
    s_program_start_pending_valid = false;
    memset(&s_program_start_pending, 0, sizeof(s_program_start_pending));
    s_program_start_confirmed_seq = 0;
    s_program_start_last_seq = 0;
    if (s_program_start_warning_active) {
        delAlarm(ALARM_CH, ALARM_WARNING, 3);
        s_program_start_warning_active = false;
    }
}

bool hmi_request_program_start_warning_active(void)
{
    return s_program_start_warning_active;
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
    tapreturn_clear_alarms();

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
    return len;
}

/* ---------------------------------------------------------------------------
 * todo：对刀启动的测量动作：**空桩，待实现**
 *
 * 当前固定返回 0，调用方（parser.cpp 的 MODE_MDI 分支）据此回执
 * REJECTED + HMI_REQUEST_ERROR_PROGRAM，界面提示"对刀启动被拒绝（错误码 5）"，
 * 机床不动作。测量动作与写刀补由实现者补齐（线索见 HMI 侧文档 12.5）；
 * 若实现时不是"生成程序串"而是直接合成运动，本函数签名与 parser.cpp 的分派要一起改。
 * --------------------------------------------------------------------------- */
int hmi_toolmeasure_build_program(const HmiRequestToolMeasureParams *toolMeasure, char *out,
                                  int outLen)
{
    (void)toolMeasure;
    (void)out;
    (void)outLen;
    return 0;   /* 空桩：待实现 */
}
