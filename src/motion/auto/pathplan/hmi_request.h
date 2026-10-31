#ifndef HMI_REQUEST_H
#define HMI_REQUEST_H

#include <stdbool.h>
#include <stdint.h>
#include <hmirequestdata.h>
#include <hmisysdata.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * HMI 请求区（M/S/T 下发、攻丝回退、从指定程序行启动）RT 侧读取模块
 *
 * 文件分三段：
 *   1) 通用协议层：取指针、整块拷贝快照、按动作分派的判据、写回执；
 *   2) 各动作的程序串生成：
 *      TAPRET：      hmi_tapret_build_program
 *      TSM：         hmi_mst_build_program
 *      TOOLMEASURE： hmi_toolmeasure_build_program
 *      增加请求时增加对应的生成函数：hmi_xxx_build_program
 *   3) PROGRAM_START_LINE（仅 MEM）：能力位发布、首按确认、pending 快照
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

/*! 从指定程序行启动的快照（仅 MODE_MEM 使用） */
typedef struct {
    uint32_t seq;                        /*!< 请求发布序号（成功时回执用） */
    int32_t  startLine;                  /*!< 0 基起始行号 */
    uint32_t pathHash;                   /*!< 程序路径指纹 */
    char     path[NCFILEPATH_CHAR_LEN];  /*!< 校验时读取的 ncFilePath */
} HmiProgramStartSnapshot;

/**
 * @brief hmi_request_data 取请求区指针（首次调用时缓存）
 * @return 共享内存未初始化时返回 NULL
 */
HmiRequestData *hmi_request_data(void);

/**
 * @brief hmi_request_take 读取一条请求并按动作校验（MDI 路径）
 * @note  接受的动作：MST_DISPATCH、TAP_RETRACT、TOOL_MEASURE（判定结果放在
 *        snap->action，由调用方二次分派）；PROGRAM_START_LINE 只在 MEM 由
 *        hmi_request_program_start_decide() 处理，本函数不认识它，会回
 *        UNKNOWN_ACTION 拒绝（MDI 下不得执行、不得回落）
 * @param[out] snap           请求快照
 * @param[out] errOut         返回 -2 时给出原因（HmiRequestError），可为 NULL
 * @return  1 = 取到可执行请求（snap 有效）
 *          0 = 无请求（调用方走原有 MDI 内容路径）
 *         -1 = 读取期间被 HMI 改写（本轮不执行、不回执）
 *         -2 = 有请求但拒绝执行（原因见 errOut）
 */
int hmi_request_take(HmiRequestSnapshot *snap, int32_t *errOut);

/**
 * @brief hmi_request_publish_capabilities 向 HMI 发布本版 RT 支持的能力位
 * @note  RT 初始化及每次 MEM 循环启动前调用；HMI 的 ResetAll 会清该位
 */
void hmi_request_publish_capabilities(void);

/**
 * @brief hmi_request_program_start_decide 读取并校验 PROGRAM_START_LINE 请求，
 *        并实现"非 0 行首按确认"
 * @note  只在 MODE_MEM 的循环启动事件处理（1ms 周期）里调用。本函数会清掉上一次
 *        的 pending 快照；拒绝时已写回执，首按报警/写入中不回执。
 *        为保证实时性，这里**不做整文件行数统计**：行号/文件行数在
 *        parser_set_content(MODE_MEM) 提交前校验。
 * @param[out] errOut 返回 -2 时的原因（HmiRequestError），可为 NULL
 * @return  1 = 允许启动，pending 快照有效（供 parser_set_content 使用）
 *          0 = 无起始行请求（调用方走原有 rerun/首行路径）
 *         -1 = 本轮不启动（首按确认已报警，或 HMI 正在写入；无回执）
 *         -2 = 请求被拒绝（已回执，本轮不启动，不得回落第 0 行）
 */
int hmi_request_program_start_decide(int32_t *errOut);

/**
 * @brief hmi_request_program_start_pending 取本次已确认的待提交快照
 * @return true = out 有效（由 hmi_request_program_start_decide 返回 1 设置）
 */
bool hmi_request_program_start_pending(HmiProgramStartSnapshot *out);

/**
 * @brief hmi_request_program_start_commit 提交完成：清 pending 与确认状态
 * @note  parser_set_content(MODE_MEM) 使用快照启动后调用
 */
void hmi_request_program_start_commit(void);

/**
 * @brief hmi_request_program_start_cancel 丢弃 pending 与首按确认状态
 * @note  复位/急停/模式切换时调用；不写 HMI 的请求区
 */
void hmi_request_program_start_cancel(void);

/**
 * @brief hmi_request_program_start_warning_active 首按确认提示是否还在显示
 * @return true = 已提示、还没按第二次启动
 */
bool hmi_request_program_start_warning_active(void);

/**
 * @brief hmi_request_program_start_validate_file 校验程序文件可读且行号在范围内
 * @param path      程序完整路径
 * @param startLine 0 基起始行号
 * @return true = 文件可读且 0 <= startLine < 行数
 */
bool hmi_request_program_start_validate_file(const char *path, int32_t startLine);

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

/**
 * @brief hmi_toolmeasure_build_program 执行对刀启动（单把刀具长度测量）
 * @note  todo：未实现，固定返回 0：测量动作与写刀补待定，由实现者补齐！
 *        返回 0 时调用方会回执 REJECTED + HMI_REQUEST_ERROR_PROGRAM，
 *        界面提示"对刀启动被拒绝（错误码 5）"，机床不动作
 * @return 生成的长度（含换行与结尾 '\0' 之前的字符数）；0 表示未实现/无有效内容
 */
int hmi_toolmeasure_build_program(const HmiRequestToolMeasureParams *toolMeasure, char *out,
                                  int outLen);

#ifdef __cplusplus
}
#endif

#endif /* HMI_REQUEST_H */
