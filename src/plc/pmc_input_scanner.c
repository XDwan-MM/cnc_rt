#include "pmc_input_scanner.h"
#include "regapi.h"       /**< 寄存器访问 API */
#include "signal_utils.h"  /**< 边沿检测宏定义 */
#include <string.h>
#include "global.h"
#include "logapi.h"
#include "log.h"


/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static int pmc_input_fetch_snapshot(G_INPUT_MAP *local_snap);

/** @brief 将解析出的逻辑打包为 PMC_EVENT 并入队 */
static void pmc_dispatch_event(PMC_EVENT_ID id, int p1, int p2);

/**
 * @brief 模式解析逻辑实现
 * 涉及 MD1, MD2, MD4 (bit 0,1,2), DNCI (bit 5), ZRN (bit 7)
 */
static void process_mode_selection(unsigned char g43);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

static G_INPUT_MAP last_g;             /**< 上一周期快照（是指任务控制器线程，上一周期的快照） */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void pmc_input_scan_cycle(void) {
    if (event_queue == NULL) return;

    G_INPUT_MAP curr_g;
    /* 1. 抓取寄存器快照 */
    if (pmc_input_fetch_snapshot(&curr_g) != 0) {
        // 读取失败，跳过本周期
        return;
    }

    /* 2. 模式检测：调用内部解析函数处理复合位 (MD1, MD2, MD4, DNCI, ZRN), 输出模式 */
    if (curr_g.G0043 != last_g.G0043) {
        process_mode_selection(curr_g.G0043);
    }

    /* 3. 进给轴按键检测 (G100/G102) - 支持多轴同时扫描 */
    for (int i = 0; i < 3; i++) {
        /* 正向轴选按键检测 */
        // 检测按下(上升沿)
        if (SIGNAL_RISING(curr_g.G0100, last_g.G0100, i))
        {
            // i = 0, 1, 2  对应这个G100变量的第0, 1, 2位。SIGNAL_RISING()为真时，发生上升沿，表示对应进给轴按钮被按下。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 1);
        }
        // 检测释放(下降沿)
        if (SIGNAL_FALLING(curr_g.G0100, last_g.G0100, i))
        {
            // i = 0, 1, 2  对应这个G100变量的第0, 1, 2位。SIGNAL_FALLING()为真时，发生下降沿，表示对应进给轴按钮被释放。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 0);
        }
        /* 负向轴选按键检测 */
        // 检测按下(上升沿)
        if (SIGNAL_RISING(curr_g.G0102, last_g.G0102, i))
        {
            // i = 0, 1, 2  对应这个G102变量的第0, 1, 2位。SIGNAL_RISING()为真时，发生上升沿，表示对应进给轴按钮被按下。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, -1);
        }
        // 检测释放(下降沿)
        if (SIGNAL_FALLING(curr_g.G0102, last_g.G0102, i))
        {
            // i = 0, 1, 2  对应这个G102变量的第0, 1, 2位。SIGNAL_FALLING()为真时，发生下降沿，表示对应进给轴按钮被释放。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 0);
        }
    }

    /* 4. 倍率检测 (G12/G14) */
    // 进给倍率变化检测G12
    if (curr_g.G0012 != last_g.G0012)
    {
        pmc_dispatch_event(EV_MSG_OVERRIDE, 0, (int)curr_g.G0012);
    }
    // 快速进给倍率检测G14
    if ((curr_g.G0014 & 0x03) != (last_g.G0014 & 0x03)) {
        int rapid_lut[] = {100, 50, 25, 1}; /* 映射到实际百分比 */
        // curr_g.G0014 & 0x03 取低两位，G14.1 G14.0组合确定快速倍率
        // G14.1 = 0, G14.0 = 0；100%  (curr_g.G0014 & 0x03) == 0
        // G14.1 = 0, G14.0 = 1；50%   (curr_g.G0014 & 0x03) == 1
        // G14.1 = 1, G14.0 = 0；25%   (curr_g.G0014 & 0x03) == 2
        // G14.1 = 1, G14.0 = 1；1%    (curr_g.G0014 & 0x03) == 3  NOTE：暂定
        pmc_dispatch_event(EV_MSG_OVERRIDE, 1, rapid_lut[curr_g.G0014 & 0x03]);
    }
    // 主轴倍率检测G30
    if (curr_g.G0030 != last_g.G0030) {
        /* p1=2 代表主轴倍率类型 */
        pmc_dispatch_event(EV_MSG_OVERRIDE, 2, (int)curr_g.G0030);
    }

    /* 5. 自动运行控制 (G7.2 启动, G8.5 中止, G8.7 复位) */
    // 启动检测 (G7.2 上升沿 【暂定上升沿触发】)
    if (SIGNAL_RISING(curr_g.G0007, last_g.G0007, 2))
    {
        // G7.2 产生上升沿，开始自动运行
        pmc_dispatch_event(EV_MSG_AUTO_RUN, 1, 0);
    }
    // 中止检测 (G8.5 上升沿 中止，发送暂停事件；为0时，继续运行))
    if (SIGNAL_RISING(curr_g.G0008, last_g.G0008, 5))
    {
        pmc_dispatch_event(EV_MSG_AUTO_RUN, 0, 0);
    }

    /* 6. 复位 (G8.7 复位 上升沿触发) */
    // 复位检测 (G8.7 上升沿)
    if (SIGNAL_RISING(curr_g.G0008, last_g.G0008, 7))
    {
        pmc_dispatch_event(EV_MSG_RESET, 0, 0);
    }

    /* 7. 主轴正反信号检测(上升沿)  (G70.4 主轴反转信号, G70.5 主轴正转信号, G29.6 主轴停止信号) */
    if (SIGNAL_RISING(curr_g.G0070, last_g.G0070, 4))
    {
        // 产生主轴反转事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, -1, 0);
    }
    if (SIGNAL_RISING(curr_g.G0070, last_g.G0070, 5))
    {
        // 产生主轴正转事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, 1, 0);
    }
    if (SIGNAL_RISING(curr_g.G0029, last_g.G0029, 6))
    {
        // 产生主轴停止事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, 0, 0);
    }

    /* 8. 更新快照供下一周期使用 */
    last_g = curr_g;
}

/* ==========================================================================
 * 3. 内部函数实现 (Private Implementations)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static int pmc_input_fetch_snapshot(G_INPUT_MAP *local_snap) {
    int ret = 0;
    /* 逐个抓取 G 寄存器字节 */
    ret |= getRegValue8(REG_G, 7,   &local_snap->G0007); // 获取共享内存G7变量的值
    ret |= getRegValue8(REG_G, 8,   &local_snap->G0008); // 获取共享内存G8变量的值
    ret |= getRegValue8(REG_G, 12,  &local_snap->G0012); // 获取共享内存G12变量的值
    ret |= getRegValue8(REG_G, 14,  &local_snap->G0014); // 获取共享内存G14变量的值
    ret |= getRegValue8(REG_G, 29,  &local_snap->G0029); // 获取共享内存G29变量的值
    ret |= getRegValue8(REG_G, 30,  &local_snap->G0030); // 获取共享内存G30变量的值
    ret |= getRegValue8(REG_G, 43,  &local_snap->G0043); // 获取共享内存G43变量的值
    ret |= getRegValue8(REG_G, 70,  &local_snap->G0070); // 获取共享内存G70变量的值
    ret |= getRegValue8(REG_G, 100, &local_snap->G0100); // 获取共享内存G100变量的值
    ret |= getRegValue8(REG_G, 102, &local_snap->G0102); // 获取共享内存G102变量的值
    return (ret == 0) ? 0 : -1;
}

/** @brief 将解析出的逻辑打包为 PMC_EVENT 并入队 */
static void pmc_dispatch_event(PMC_EVENT_ID id, int p1, int p2)
{
    if (id == EV_MSG_NONE) return;
    PMC_EVENT event;
    switch (id) {
    case EV_MSG_MODE:
    {
        event.id = id;
        event.mode_payload.mode = p1; // 目标模式
        // log_system("[plc] 扫描到模式切换(%d)", id);
        // LOG_OUTPUT(INFO, "扫描到模式切换 %d", id);
        break;
    }
    case EV_MSG_FEEDAXIS_KEY:
    {
        event.id = id;
        event.feedaxis_key_payload.axis_num = p1;
        event.feedaxis_key_payload.direction = p2;
        // log_system("[plc] 扫描到手动 feedaxis_key: %d, axis: %d, action: %d ", event.id, p1, p2);
        // LOG_OUTPUT(INFO, "扫描到手动 feedaxis_key: %d, axis: %d, action: %d", event.id, p1, p2);
        break;
    }
    case EV_MSG_SPINDLE:
    {
        event.id = id;
        event.spindle_payload.direction = p1;
        // log_system("[plc] 扫描到主轴切换 sp:%d, direction: %d", event.id, p1);
        // LOG_OUTPUT(INFO, "扫描到主轴切换 sp:%d, direction: %d", event.id, p1);
        break;
    }
    case EV_MSG_OVERRIDE:
    {
        event.id = id;
        event.override_payload.override_type = p1;
        event.override_payload.val = p2;
        // log_system("[plc] 扫描到倍率切换 type: %d, override_type: %d, val: %d ", event.id, p1, p2);
        // LOG_OUTPUT(INFO, "扫描到倍率切换 type: %d, override_type: %d, val: %d", event.id, p1, p2);
        break;
    }
    case EV_MSG_AUTO_RUN:
    {
        event.id = id;
        event.run_payload.run_state = p1;
        // log_system("[plc] 扫描到运行状态切换 run:%d, state: %d", event.id, p1);
        // LOG_OUTPUT(INFO, "扫描到运行状态切换 run:%d, state: %d", event.id, p1);
        break;
    }
    case EV_MSG_RESET:
    {
        event.id = id;
        event.reset_payload.reset_state = p1;
        // log_system("[plc] 扫描到复位: %d, reset_state: %d", event.id, p1);
        // LOG_OUTPUT(INFO, "扫描到复位: %d, reset_state: %d", event.id, p1);
        break;
    }
    default:
        break;
    }
     /* 推送 PMC_EVENT 到线程安全的队列 */
    event_queue_push(event_queue, &event);
}

/**
 * @brief 模式解析逻辑实现
 * 涉及 MD1, MD2, MD4 (bit 0,1,2), DNCI (bit 5), ZRN (bit 7)
 */
static void process_mode_selection(unsigned char g43) {
    CNC_MODE_INDEX target_mode = CNC_MODE_INVALID; // 默认无效模式

    // 1. 判定高优先级信号
    if (GET_BIT(g43, 7)) {
        target_mode = CNC_MODE_ZRN;  // ZRN 置位进入回零模式
    } else if (GET_BIT(g43, 5)) {
        target_mode = CNC_MODE_DNC;  // DNCI 置位 
    } else {
        // 2. 判定MD4, MD2, MD1组合 (取低3位)
        unsigned char md_bits = g43 & 0x07;
        switch (md_bits) {
        case 0x01: target_mode = CNC_MODE_MEM;  break; // 001
        case 0x02: target_mode = CNC_MODE_MDI;  break; // 010
        case 0x03: target_mode = CNC_MODE_EDIT; break; // 011
        case 0x04: target_mode = CNC_MODE_MPG;  break; // 100
        case 0x05: target_mode = CNC_MODE_JOG;  break; // 101
        default:   target_mode = CNC_MODE_INVALID; break;
        }
    }

    pmc_dispatch_event(EV_MSG_MODE, (int)target_mode, 0);
}
