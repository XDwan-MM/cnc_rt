#include "hmi_request.h"

#include <math.h>
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

    // 1、版本闸门：不相等不消费（正常部署下不会发生，按无请求处理）
    if (__atomic_load_n(&req->protocolVersion, __ATOMIC_ACQUIRE)
            != HMI_REQUEST_PROTOCOL_VERSION) {
        return 0;
    }

    // 2、动作判定：NONE = 无请求；未定义的动作 = 界面已有新功能而本版 RT 不认识
    //    （必须拒绝，不能按错误语义执行；把序号带回，界面才能认出这条回执是自己的）
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
        // 全 0 生成不出程序（HMI 侧本就不会挂全 0，这里防御）
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
 * 攻丝回退程序串生成：**空桩，待实现**
 *
 * 当前固定返回 0，调用方（parser.cpp 的 MODE_MDI 分支）据此回执
 * REJECTED + HMI_REQUEST_ERROR_PROGRAM，界面提示"攻丝回退被拒绝（错误码 5）"，
 * 机床不动作。补这个函数时请一并核对下列各点（详见
 * hmi/docs/hmirequest-Instruction.md 第 11 节）：
 *
 *   1) 回退行程：设计取向"回到攻丝起点"。目前 RT 侧没有任何地方记录攻丝起点
 *      （SPINDLE_DATA / rt_data / PLAN_QUEUE 里都没有），要新加存储，填充点
 *      在 parser.cpp 中 PMC_G33_1（攻丝循环）那一支，或者取发起攻丝时的轴位置。
 *      在起点记录做出来之前，先用 HMI_TAPRET_RETRACT_DISTANCE_MM 写死一个
 *      回退量占位。
 *   2) 目标必须是**绝对位置**：目标 = 当前坐标 + 回退量（G33.2 的终点取自
 *      motion_data->position，即程序里的坐标字）。当前坐标可在 parser.cpp 里
 *      用 rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur 取（parser_init 里给
 *      ItpSetOriginMacPos 用的就是同一来源）。
 *   3) 必须带轴字：没有轴字时 end_pos == start_pos → length == 0 →
 *      auto_get_pos_cmd() 直接判 DO_NULL_CMD，块不动作。
 *   4) 进给：F = 螺距(mm/rev) × S(rpm) 得 mm/min（这样 parser 里
 *      screw_offset = F / S 正好等于螺距）。
 *   5) 指令选择待定。G33.2 是 RIGID_TAP/TAP_RETURN 路径（exec_cmd = DO_TAP_CMD），
 *      但当前 interpolate_tap()（motion/auto/interp/interpolation.c）有两个问题：
 *        a) 只写走刀轴（默认平面 PLANE_XY 下只写 z），另两轴是未初始化的栈值，
 *           会被 auto_execute_pos_cmd() 写进 X/Y 的 wcs_cmd_pos_cur；
 *        b) 恒按 start + accum_distance 走 **+ 方向**，不看目标符号。
 *      用 G33.2 之前先把这两点修掉（回退必须是 Z 正向）。
 *   6) M 值（3/4）对本运动没有影响：rigid_data.speed / screw_number 无读取点，
 *      reverse / screw_offset 只参与被编译器丢弃的运算。要让主轴真的转，
 *      得把 M04 之类的块单独发出去（走 MST 那套 REG_F 交握）。
 *   7) 一次性去重：本函数能真正执行之后，同一 seq 只能执行一次
 *      （HMI 收到回执后最长 150ms 才撤销），见文档 §11.5。
 * --------------------------------------------------------------------------- */
int hmi_tapret_build_program(const HmiRequestTapRetParams *tapret, char *out, int outLen)
{
    (void)tapret;
    (void)out;
    (void)outLen;
    return 0;   /* 空桩：待实现 */
}
