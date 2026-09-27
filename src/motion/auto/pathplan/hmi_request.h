#ifndef HMI_REQUEST_H
#define HMI_REQUEST_H

#include <stdint.h>
#include <hmirequestdata.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * HMI 请求区（M/S/T 下发、攻丝回退）RT 侧读取模块
 *
 * 文件分两段：
 *   1) 通用协议层：取指针、整块拷贝快照、按动作分派的判据、写回执；
 *   2) 各动作的程序串生成：
 *      TAPRET：    hmi_tapret_build_program
 *      TSM：       hmi_mst_build_program
 *      增加请求时增加对应的生成函数：hmi_xxx_build_program
 *===========================================================================*/

/*! MST 程序串缓冲区建议长度（T≤4位、S≤5位、M≤3位，最长一行约 20 字节） */
#define HMI_MST_PROGRAM_BUF_LEN 64

/*! 攻丝回退程序串缓冲区长度（与 MST 同量级留余量） */
#define HMI_TAPRET_PROGRAM_BUF_LEN 64

/*! todo: 攻丝回退量占位值（mm）：未确定是否需要，写死 */
#define HMI_TAPRET_RETRACT_DISTANCE_MM (10.0)

#define HMI_TAPRET_REJECT_PARAM (-1)

/*! 请求快照：只含 HMI 侧字段，一次整块拷贝出来，调用方在副本上工作 */
typedef struct {
    uint32_t seq;             /*!< 本次读取到的发布序号 */
    uint32_t action;          /*!< 动作（HmiRequestAction，由 take 判定后带回） */
    uint64_t timestampMs;     /*!< 快照时刻的时间戳（系统级单调毫秒） */
    HmiRequestParams params;  /*!< 动作参数（256 字节，整块拷贝） */
} HmiRequestSnapshot;

/**
 * @brief hmi_request_data 取请求区指针（首次调用时缓存）
 * @return 共享内存未初始化时返回 NULL
 */
HmiRequestData *hmi_request_data(void);

/**
 * @brief hmi_request_take 读取一条请求并按动作校验
 * @note  接受的动作：MST_DISPATCH、TAP_RETRACT（判定结果放在 snap->action，
 *        由调用方二次分派）；其它非 0 动作一律拒绝（UNKNOWN_ACTION）
 * @param[out] snap           请求快照
 * @param[out] errOut         返回 -2 时给出原因（HmiRequestError），可为 NULL
 * @return  1 = 取到可执行请求（snap 有效）
 *          0 = 无请求（调用方走原有 MDI 内容路径）
 *         -1 = 读取期间被 HMI 改写（本轮不执行、不回执）
 *         -2 = 有请求但拒绝执行（原因见 errOut）
 */
int hmi_request_take(HmiRequestSnapshot *snap, int32_t *errOut);

/**
 * @brief hmi_request_ack 回写回执
 * @note  只写 RT 自己的三个字段：acknowledgedSequence / errorCode / state
 */
void hmi_request_ack(uint32_t seq, uint32_t state, int32_t errorCode);

/**
 * @brief hmi_mst_build_program 生成 MST 程序串
 *        仅取非 0 项，顺序 T→S→M，项间单空格，结尾换行；M<100 用 M%02d
 * @return 生成的长度（含换行与结尾 '\0' 之前的字符数）；0 表示无有效内容或缓冲不足
 */
int hmi_mst_build_program(const HmiRequestMstParams *mst, char *out, int outLen);

/**
 * @brief hmi_tapret_build_program 生成攻丝回退程序串
 * @note  todo：未实现，固定返回 0：回退量与指令选择待定，由实现者补齐！
 *        返回 0 时调用方会回执 REJECTED + HMI_REQUEST_ERROR_PROGRAM，
 *        界面提示"攻丝回退被拒绝（错误码 5）"，机床不动作
 * @return 生成的长度（含换行与结尾 '\0' 之前的字符数）；0 表示未实现/无有效内容
 */
int hmi_tapret_build_program(const HmiRequestTapRetParams *tapret, char *out, int outLen);

#ifdef __cplusplus
}
#endif

#endif /* HMI_REQUEST_H */
