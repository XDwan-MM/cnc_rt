#ifndef MASTER_WCF_H
#define MASTER_WCF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* =========================================================
 * Master 状态
 * ========================================================= */
typedef enum
{
    MASTER_STATE_INIT = 0,

    MASTER_STATE_STARTED,

    MASTER_STATE_OP,

    MASTER_STATE_ERROR,

    MASTER_STATE_CLOSED

} master_state_t;

/* =========================================================
 * Master 错误码
 * ========================================================= */
typedef enum
{
    MASTER_SUCCESS = 0,

    MASTER_ERR_INVALID_PARAM = -1,
    MASTER_ERR_NOT_READY = -2,
    MASTER_ERR_INIT = -3,
    MASTER_ERR_RECEIVE = -4,
    MASTER_ERR_SEND = -5,
    MASTER_ERR_SYNC = -6,
    MASTER_ERR_HARDWARE = -7,
    MASTER_ERR_RESET = -8,
    MASTER_ERR_CLOSE = -9

} master_retcode_t;

/* =========================================================
 * Master 上下文
 * ========================================================= */
typedef struct
{
    master_state_t state;

    int32_t error_code;

    uint32_t cycle_count;

    bool initialized;
    bool op_valid;

} master_context_t;

/* =========================================================
 * 全局变量
 * ========================================================= */

extern master_context_t g_master;

extern int master_error_code;

/*
 * reset_sem：
 *
 * 由其他线程/模块请求驱动器报警复位。
 *
 * Master PDO线程负责执行实际的 Fault Reset。
 */
extern bool reset_sem;

/* =========================================================
 * Master 生命周期
 * ========================================================= */

/**
 * @brief 启动 EtherCAT Master
 */
int Master_Start(void);

/**
 * @brief 关闭 EtherCAT Master
 */
int Master_Close(void);

/**
 * @brief 请求驱动器 Fault Reset
 *
 * 本函数不直接操作 EtherCAT PDO。
 *
 * 只设置 reset_sem。
 *
 * 真正的 Fault Reset 在 Master PDO线程中执行。
 */
void Master_Reset(void);

/* =========================================================
 * Master 周期
 * ========================================================= */

/**
 * @brief 等待 EtherCAT 周期同步
 *
 * 在实时模式下：
 *
 * 1. 等待 EtherCAT 周期
 * 2. 接收 PDO
 * 3. 更新 Driver HAL反馈缓存
 * 4. 唤醒 PLC
 * 5. 唤醒 Motion
 * 6. 唤醒 Path Plan
 */
void Master_Wait_Sync(void);

/**
 * @brief Master PDO线程主循环
 */
void Master_Run(void);

/**
 * @brief 接收 EtherCAT PDO
 *
 * 注意：
 *
 * 每个 EtherCAT 周期只在这里读取一次反馈 PDO。
 */
int Master_Receive(void);

/**
 * @brief 发送 EtherCAT PDO
 */
int Master_Send(void);

/* =========================================================
 * Master 状态
 * ========================================================= */

master_state_t Master_Get_State(void);

bool Master_Is_OP(void);

int Master_Error(void);

/* =========================================================
 * 驱动器报警
 * ========================================================= */

/**
 * @brief 检查驱动器报警
 *
 * 不直接读取 EtherCAT PDO。
 *
 * 只读取 Driver HAL 已经缓存的数据。
 */
void Check_Driver_Error(void);

/* =========================================================
 * Master 硬件错误
 * ========================================================= */

/**
 * @brief 检查 EtherCAT 硬件/通信错误
 */
void Master_Hardware_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_WCF_H */