#ifndef _ETHERCAT_H
#define _ETHERCAT_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"
#include "backend/greemaster/mapping/slave_list.h"
#include "backend/greemaster/export.h"

#define GM_MIN_CYCLE_US 250     /* GM_Master_Set_Cycle 支持的最小周期 */

/** 主站启动配置 —— ethercat_init() 的入参。 */
typedef struct {
    uint32_t cycle_us;        /* 通讯周期（us），下限 GM_MIN_CYCLE_US */
    uint32_t start_timeout_ms; /* 启动总预算；SDK 秒粒度等待向上取整 */
    uint32_t cycle_timeout_ms; /* 收帧等待超时（ms）；SDK 秒粒度等待向上取整 */
    int      dc_enable;       /* 是否使能 DC 信号（分布式时钟） */
} MasterConfig;

/* 默认配置：MasterConfig cfg = MASTER_CONFIG_DEFAULT; */
#define MASTER_CONFIG_DEFAULT                                       \
    { .cycle_us = 1000u, .start_timeout_ms = 120000u, .cycle_timeout_ms = 5000u, \
      .dc_enable = 1 }

#define MASTER_STOP_REQUESTED (-28673)
#define MASTER_START_TIMEOUT  (-28674)
#define MASTER_DEVICE_DICTIONARY_ERROR (-28675)

/**
 * @brief 取最近一次启动失败的原因文本。
 *
 * @return 静态缓冲中的中文原因；成功启动后内容无意义。
 */
const char* Master_StartupError(void);

/**
 * @brief 取已扫到的从站数；主站未就绪时恒为 0。
 *
 * @return 从站数。
 */
int Master_SlaveCount(void);
/* 已确认的零基位置；-1 未知，-2 为面板断联但位置需由配置确定。 */
int Master_LostSlave(void);

/**
 * @brief 执行一笔 SDO 传输（非实时线程）。
 *
 * @param[in]  slave_pos  从站位置号。
 * @param[in]  index      对象索引。
 * @param[in]  subindex   对象子索引。
 * @param[in,out] data    读写缓冲。
 * @param[in]  size       数据字节数，只接受 1/2/4。
 * @param[in]  write      非零表示写。
 * @param[in]  timeout_ms 总等待时间。
 * @return 0 成功；-1 一般失败；-2 超时；-3 SDO 应答错误；-4 主站停止。
 */
int Master_SdoTransfer(int slave_pos, uint16_t index, uint8_t subindex,
                       void* data, uint16_t size, int write, uint32_t timeout_ms);

/**
 * @brief 取打断标志地址，供 SDK 阻塞调用轮询。
 *
 * @return 打断标志地址。
 */
int* Master_InterruptFlag(void);

/**
 * @brief 是否已被要求打断。
 *
 * @return 非零表示已置位。
 */
int  Master_Interrupted(void);

/* 设备字典优先使用可执行文件目录下 ../lib/cnc_hal/lib/devices.json。
 * 文件不存在时取 CNC_HAL_DEVICES_JSON；访问或解析失败直接阻止启动。 */

#define PARAM_CHANG 1          /* 是否修改主站参数 */

/* 握手前的 REG/SDO 交互窗口（GM_Wait_Pilot_Data_Response）；尚未实现。 */
#define OP_PRE_REG_EN 0

typedef struct {
    int Model;      /* 报错模块 */
    int Fms;        /* 报错状态机 */
    int Code;       /* 报错索引码 */
    int errslave;
    int ErrorID;    /* 预定义的枚举 ID */
} ErrorContext;

/**
 * @brief 清空全局错误上下文，供每次启动前复位。
 */
void error_module_init(void);

typedef enum _EcatMasterError
{
    ECAT_INIT_ERROR = 0x1001,                   /* 主站初始化模块错误 */
    ECAT_CONTROL_PANEL_CONNECT_LOST = 0x1002,   /* 控制面板断连 */
    ECAT_IO_CONNECT_LOST = 0x1003,              /* IO 模块断连 */
    ECAT_SLAVE3_CONNECT_LOST = 0x1004,          /* 从站三断连 */
    ECAT_SLAVE4_CONNECT_LOST = 0x1005,          /* 从站四断连 */
    ECAT_SLAVE5_CONNECT_LOST = 0x1006,          /* 从站五断连 */
    ECAT_SLAVE6_CONNECT_LOST = 0x1007,          /* 从站六断连 */
    ECAT_TOPOLOGY_ERROR = 0x1008,               /* 主站拓扑模块错误 */
    ECAT_ACTIVE_ERROR = 0x1009,                 /* 主站激活模块错误 */
    ECAT_DC_ERROR = 0x100A,                     /* 主站 DC 模块错误 */
    ECAT_PDO_ERROR = 0x100B,                    /* 主站 PDO 模块错误 */
    ECAT_RT_ERROR = 0x100C,                     /* 主站 RT 模块错误 */
    UNKOWN_ERROR = 0xFFFF                       /* 未知错误 */
} EcatMasterError;

extern int slave_num;


/**
 * @brief 启动主站到「周期收发模式可用」。
 *
 * @param[in] cfg 主站配置（周期、毫秒超时、DC）。
 * @return 0 成功；< 0 失败。
 */
MASTER_API int ethercat_init(const MasterConfig* cfg);

/**
 * @brief 关闭主站并释放资源；幂等，调用方须先退出周期调用。
 *
 * @return 0 成功；< 0 失败。
 */
MASTER_API int ethercat_close(void);

/**
 * @brief 等本周期节拍并收一帧——周期里唯一的阻塞点；只在实时线程调用。
 *
 * @return 0 成功；< 0 等节拍超时或收帧失败。
 */
MASTER_API int Master_WaitCycle(void);

/**
 * @brief 下发本帧输出；只在实时线程调用。
 *
 * 标准周期顺序：WaitCycle → 读写 → CommitCycle。
 *
 * @return 0 成功；< 0 失败。
 */
MASTER_API int Master_CommitCycle(void);

/**
 * @brief 主站是否出过错或已停机。
 *
 * @return 0 正常；非 0 出错或已停机。
 */
MASTER_API int Master_StopFlag(void);

/**
 * @brief 非阻塞停止通知：等待中的周期调用会在 SDK 返回后报告 MASTER_STOP_REQUESTED。
 *
 * 不释放资源、不承诺唤醒 SDK；与周期调用可并发，生命周期调用仍须串行。
 */
MASTER_API void Master_RequestStop(void);

/** 总线健康快照 —— 主站自己累计的警告与计数，掉电清零。 */
typedef struct {
    int      pdo_warn;             /* 非零 = 有 PDO 警告 */
    int      pdo_warn_code;        /* PDO 警告号 */
    unsigned pdo_warn_para;        /* PDO 警告参数 */
    int      dc_warn;              /* 非零 = 有 DC（分布式时钟）警告 */
    int      dc_warn_code;
    unsigned dc_warn_para;
    unsigned crc_err_count;        /* CRC 错误累计次数 */
    unsigned frame_timeout_count;  /* 帧超时累计次数 */
    unsigned expect_wkc_tx;        /* 期望工作计数器（Tx 侧） */
    unsigned expect_wkc_rx;        /* 期望工作计数器（Rx 侧） */
} MasterBusHealth;

/**
 * @brief 取最近一次收帧的总线健康快照：只记录不判断。
 *
 * @param[out] out 带出快照；未收过帧时全零。
 * @return 0 成功；< 0 out 为空。
 */
MASTER_API int Master_BusHealth(MasterBusHealth* out);

#endif
