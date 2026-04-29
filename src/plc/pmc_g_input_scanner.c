#include "pmc_g_input_scanner.h"
#include "global.h"
#include <hmisysapi.h>
#include "regapi.h"       /**< 寄存器访问 API */
#include "signal_utils.h" /**< 边沿检测宏定义 */
#include <string.h>

/**
 * @brief 寄存器自动扫描注册表
 * * 作用：
 * 1. 集中管理所有需要从共享内存抓取到本地快照的 G 寄存器地址。
 * 2. 只有在此数组中注册过的索引，其对应的 g_regs[i] 才会包含有效的 共享内存G寄存器 数据。
 * 3. 以后若想增加 G19 或 G32，只需在此数组末尾添加数字 19, 32 即可，无需改动获取数据的函数。
 */
static const int SCAN_G_REGS[] =
    {
        4, 5, 6, 7, 8, 12, 14, 19, 29, 30, 32, 43, 44, 45, 61, 62, 70, 100, 102, 130, 132, 134 /* 已有信号的地址 */
                                                                                               /* <--- 您想新增的信号地址 */
};

/** 计算注册表中元素的个数，用于驱动循环扫描 */
#define SCAN_G_REGS_COUNT (sizeof(SCAN_G_REGS) / sizeof(SCAN_G_REGS[0]))

static G_INPUT_MAP
    last_g; /**< 上一周期快照（是指任务控制器线程，上一周期的快照） */

/* 首次扫描标志 */
static bool is_first_scan = true;

/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static void pmc_input_fetch_snapshot(G_INPUT_MAP *local_snap);

/** @brief 将解析出的逻辑打包为 PMC_EVENT 并入队 */
static void pmc_dispatch_event(PMC_EVENT_ID id, int p1, int p2);

/**
 * @brief 模式解析逻辑实现
 * 涉及 MD1, MD2, MD4 (bit 0,1,2), DNCI (bit 5), ZRN (bit 7)
 */
static void process_mode_selection(char g43, char g19);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void pmc_g_input_scan_cycle(void)
{
    if (event_queue == NULL)
        return;

    G_INPUT_MAP curr_g;
    memset(&curr_g, 0, sizeof(G_INPUT_MAP)); /* 初始化清零，防止脏数据 */
    /* step1. 抓取寄存器快照 */
    pmc_input_fetch_snapshot(&curr_g);

    /* step2. G信号检测 */

    /* G8.4紧急停止检测 (下降沿或常开逻辑触发)  G08.4 = 0急停拍下，G08.4 = 1急停拉起 */
    // 检测急停按下 1->0（下降沿，如果常开逻辑触发会造成事件堆积。）
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 4))
    {
        // 急停拍下
        rt_data->sys_state[SYS_IS_ESTOP_PRESS] = true;
        LOG(LOG_LEVEL_INFO, "扫描到急停: 拍下");
    }
    // 检测急停释放 0->1（上升沿）
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 4))
    {
        // 急停释放
        // 复位后实时内部急停标志rt_data->sys_state[SYS_IS_ESTOP_ON]置为false
        rt_data->sys_state[SYS_IS_ESTOP_PRESS] = false;
        LOG(LOG_LEVEL_INFO, "扫描到急停: 释放");
    }
    /* G43模式检测：调用内部解析函数处理复合位 (MD1, MD2, MD4, DNCI, ZRN), * 输出模式 */
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[43], last_g.g_regs[43]) ||
        SIGNAL_BIT_CHANGED(curr_g.g_regs[19], last_g.g_regs[19], 7))
    {
        process_mode_selection(curr_g.g_regs[43], curr_g.g_regs[19]);
    }

    /* (G100/G102)进给轴按键检测  - 支持多轴同时扫描 */
    for (int i = 0; i < 3; i++)
    {
        /* G100 正向轴选按键检测 */
        // 检测按下(上升沿)
        if (SIGNAL_RISING(curr_g.g_regs[100], last_g.g_regs[100], i))
        {
            // i = 0, 1, 2  对应这个G100变量的第0, 1,
            // 2位。SIGNAL_RISING()为真时，发生上升沿，表示对应进给轴按钮被按下。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 1);
        }
        // 检测释放(下降沿)
        if (SIGNAL_FALLING(curr_g.g_regs[100], last_g.g_regs[100], i))
        {
            // i = 0, 1, 2  对应这个G100变量的第0, 1,
            // 2位。SIGNAL_FALLING()为真时，发生下降沿，表示对应进给轴按钮被释放。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 0);
        }
        /* G102 负向轴选按键检测 */
        // 检测按下(上升沿)
        if (SIGNAL_RISING(curr_g.g_regs[102], last_g.g_regs[102], i))
        {
            // i = 0, 1, 2  对应这个G102变量的第0, 1, 2位。
            // SIGNAL_RISING()为真时，发生上升沿，表示对应进给轴按钮被按下。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, -1);
        }
        // 检测释放(下降沿)
        if (SIGNAL_FALLING(curr_g.g_regs[102], last_g.g_regs[102], i))
        {
            // i = 0, 1, 2  对应这个G102变量的第0, 1,
            // 2位。SIGNAL_FALLING()为真时，发生下降沿，表示对应进给轴按钮被释放。
            pmc_dispatch_event(EV_MSG_FEEDAXIS_KEY, i, 0);
        }
    }

    /* (G12/G14) 倍率检测 */
    // 进给倍率变化检测G12
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[12], last_g.g_regs[12]))
    {
        // 1、共享内存有符号版本代码逻辑
        int feed_override = 0;

        /* 核心修正：强制转换为无符号字节，避免符号位干扰位运算 */
        unsigned char raw_val = (unsigned char)curr_g.g_regs[12];

        /* * 倍率值 = (2^i * Vi)累加 [%]，i从0到7。
         * 根据文档原理：FVi为'1'时 Vi=0；FVi为'0'时 Vi=1。FVi对应G12变量中的第i位。
         * 这在逻辑上等同于对这 8 个位进行按位取反。
         */
        unsigned char inverted_val = (unsigned char)(~raw_val);

        /* * 特殊逻辑判断:
         * 1. 所有信号为 '0' (0x00) 时，视为 0%。
         * 2. 所有信号为 '1' (0xFF) 时，取反后为 0，自然也是 0%。
         */
        if (raw_val == 0x00 || raw_val == 0xFF)
        {
            feed_override = 0;
        }
        else
        {
            /* * 此时 inverted_val 的数值直接对应 1%~254% 的倍率。
             * 例如：若信号为 0b11111010 (0xFA)，取反后为 0b00000101 (5)，即 5%。
             */
            feed_override = (int)inverted_val;
        }
        /* p1=0 代表进给倍率类型 (EV_MSG_OVERRIDE) */
        pmc_dispatch_event(EV_MSG_OVERRIDE, 0, feed_override);

        // // 2、共享内存无符号版本的代码逻辑
        // pmc_dispatch_event(EV_MSG_OVERRIDE, 0, (int)curr_g.g_regs[12]);
    }
    // 快速进给倍率检测G14
    if (is_first_scan || ((curr_g.g_regs[14] & 0x03) != (last_g.g_regs[14] & 0x03)))
    {
        int rapid_lut[] = {100, 50, 25, 0}; /* 映射到实际百分比 */
        // curr_g.G0014 & 0x03 取低两位，G14.1 G14.0组合确定快速倍率
        // G14.1 = 0, G14.0 = 0；100%  (curr_g.G0014 & 0x03) == 0
        // G14.1 = 0, G14.0 = 1；50%   (curr_g.G0014 & 0x03) == 1
        // G14.1 = 1, G14.0 = 0；25%   (curr_g.G0014 & 0x03) == 2
        // G14.1 = 1, G14.0 = 1；0%    (curr_g.G0014 & 0x03) == 3  NOTE：暂定20260313
        pmc_dispatch_event(EV_MSG_OVERRIDE, 1, rapid_lut[curr_g.g_regs[14] & 0x03]);
    }
    // 主轴倍率检测G30
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[30], last_g.g_regs[30]))
    {
        /* p1=2 代表主轴倍率类型 */
        pmc_dispatch_event(EV_MSG_OVERRIDE, 2, (int)curr_g.g_regs[30]);
    }

    /* 自动运行控制 (G7.2 启动, G8.5 中止, G8.7 复位) */
    // 启动检测 (G7.2 上升沿 【暂定上升沿触发】)
    if (SIGNAL_RISING(curr_g.g_regs[7], last_g.g_regs[7], 2))
    {
        // G7.2 产生上升沿，开始自动运行
        pmc_dispatch_event(EV_MSG_AUTO_RUN, 1, 0);
    }
    // 中止检测 (G8.5 下降沿 中止，发送暂停事件；)
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 5))
    {
        pmc_dispatch_event(EV_MSG_AUTO_RUN, 0, 0);
    }

    /* 外部复位 (G8.7 复位 上升沿触发) */
    // 复位检测 (G8.7 上升沿)
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 7))
    {
        pmc_dispatch_event(EV_MSG_RESET, 1, 0);
    }

    /* 主轴正反信号检测 (G70.4 主轴反转, G70.5 主轴正转, G29.6 主轴停止信号) */
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 4))
    {
        // 产生主轴反转事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, -1, 0);
    }
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 5))
    {
        // 产生主轴正转事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, 1, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[29], last_g.g_regs[29], 6))
    {
        // 产生主轴停止事件 入队
        pmc_dispatch_event(EV_MSG_SPINDLE, 0, 0);
    }
    /* 主轴定向 G70.6 */
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G70.6 发生上升沿");
    }

    /* M代码FIN G4.3 */
    // if (SIGNAL_RISING(curr_g.g_regs[4], last_g.g_regs[4], 3))
    // {
    //     LOG(LOG_LEVEL_INFO, "扫描到G4.3 发生上升沿");
    // }

    /* 可选程序段跳过 G44.0 */
    if (SIGNAL_RISING(curr_g.g_regs[44], last_g.g_regs[44], 0))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, BLOCK_SKIP, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G44.0 发生上升沿");
    }
    if (SIGNAL_FALLING(curr_g.g_regs[44], last_g.g_regs[44], 0))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, BLOCK_SKIP, 0);
        LOG(LOG_LEVEL_INFO, "扫描到G44.0 发生下降沿");
    }
    /* 跳段 G45发生变化 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[45], last_g.g_regs[45]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[45], last_g.g_regs[45], i))
            {
                pmc_dispatch_event(EV_MSG_ENANLE, BLOCK_SKIP + i + 1, 1);
                LOG(LOG_LEVEL_INFO, "扫描到G45.%d 发生上升沿", i);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[45], last_g.g_regs[45], i))
            {
                pmc_dispatch_event(EV_MSG_ENANLE, BLOCK_SKIP + i + 1, 0);
                LOG(LOG_LEVEL_INFO, "扫描到G45.%d 发生下降沿", i);
            }
        }
    }

    /* 空运行 G46.7 */
    if (SIGNAL_RISING(curr_g.g_regs[46], last_g.g_regs[46], 7))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, DRY_RUN, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G46.7 发生上升沿");
    }
    if (SIGNAL_FALLING(curr_g.g_regs[46], last_g.g_regs[46], 7))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, DRY_RUN, 0);
        LOG(LOG_LEVEL_INFO, "扫描到G46.7 发生下降沿");
    }

    /* 单程序段 G46.1 */
    if (SIGNAL_RISING(curr_g.g_regs[46], last_g.g_regs[46], 1))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, SINGLE_BLOCK, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G46.1 发生上升沿");
    }
    if (SIGNAL_FALLING(curr_g.g_regs[46], last_g.g_regs[46], 1))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, SINGLE_BLOCK, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G46.1 发生下降沿");
    }

    /* 辅助功能锁住 G5.6 */
    if (SIGNAL_RISING(curr_g.g_regs[5], last_g.g_regs[5], 6))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, MSBT_LOCK, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G5.6 发生上升沿");
    }
    if (SIGNAL_FALLING(curr_g.g_regs[5], last_g.g_regs[5], 6))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, MSBT_LOCK, 0);
        LOG(LOG_LEVEL_INFO, "扫描到GG5.6 发生下降沿");
    }

    /* 机床锁住 G4.1 */
    if (SIGNAL_RISING(curr_g.g_regs[4], last_g.g_regs[4], 1))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, MAC_LOCK, 1);
        LOG(LOG_LEVEL_INFO, "扫描到G4.1 发生上升沿");
    }
    if (SIGNAL_FALLING(curr_g.g_regs[4], last_g.g_regs[4], 1))
    {
        pmc_dispatch_event(EV_MSG_ENANLE, MAC_LOCK, 0);
        LOG(LOG_LEVEL_INFO, "扫描到G4.1 发生下降沿");
    }
    /* 各轴机床锁住 G108 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[108], last_g.g_regs[108]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[108], last_g.g_regs[108], i))
            {
                pmc_dispatch_event(EV_MSG_ENANLE, MAC_LOCK + i + 1, 1);
                LOG(LOG_LEVEL_INFO, "扫描到G108.%d 发生上升沿", i);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[108], last_g.g_regs[108], i))
            {
                pmc_dispatch_event(EV_MSG_ENANLE, MAC_LOCK + i + 1, 0);
                LOG(LOG_LEVEL_INFO, "扫描到G108.%d 发生下降沿", i);
            }
        }
    }

    /*step3 更新快照供下一周期使用 */
    last_g = curr_g;

    /* 首次扫描结束，清除标志位 */
    is_first_scan = false;
}

/* ==========================================================================
 * 3. 内部函数实现 (Private Implementations)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体
 * 遍历注册表，集中抓取数据，数据安全且无遗漏。
 */
static void pmc_input_fetch_snapshot(G_INPUT_MAP *local_snap)
{
    /* 遍历注册表中的每一个索引 */
    for (int i = 0; i < SCAN_G_REGS_COUNT; i++)
    {
        // 1. 从数组中取出寄存器号（例如 i=0 时，G寄存器注册表数组SCAN_G_REGS[0]=7，reg_idx=7）
        int reg_idx = SCAN_G_REGS[i];
        // 2. 调用底层 API 获取该地址的值，并存入本地数组的对应位置
        // 注意：local_snap->g_regs[7] 对应 G0007
        getRegValue8(REG_G, reg_idx, &local_snap->g_regs[reg_idx]);
    }
}

/** @brief 将解析出的逻辑打包为 PMC_EVENT 并入队 */
static void pmc_dispatch_event(PMC_EVENT_ID id, int p1, int p2)
{
    if (id == EV_MSG_NONE)
        return;
    PMC_EVENT event;
    switch (id)
    {
    case EV_MSG_MODE:
    {
        event.id = id;
        event.mode_payload.mode = p1; // 目标模式
        LOG(LOG_LEVEL_INFO, "扫描到模式切换 %d", id);
        break;
    }
    case EV_MSG_FEEDAXIS_KEY:
    {
        event.id = id;
        event.feedaxis_key_payload.axis_num = p1;
        event.feedaxis_key_payload.direction = p2;
        LOG(LOG_LEVEL_INFO, "扫描到手动 feedaxis_key: %d, axis: %d, action: %d",
            event.id, p1, p2);
        break;
    }
    case EV_MSG_SPINDLE:
    {
        event.id = id;
        event.spindle_payload.direction = p1;
        LOG(LOG_LEVEL_INFO, "扫描到主轴切换 sp:%d, direction: %d", event.id, p1);
        break;
    }
    case EV_MSG_OVERRIDE:
    {
        event.id = id;
        event.override_payload.override_type = p1;
        event.override_payload.val = p2;
        LOG(LOG_LEVEL_INFO, "扫描到倍率切换 type: %d, override_type: %d, val: %d",
            event.id, p1, p2);
        break;
    }
    case EV_MSG_AUTO_RUN:
    {
        event.id = id;
        event.run_payload.run_state = p1;
        LOG(LOG_LEVEL_INFO, "扫描到运行状态切换 run:%d, state: %d", event.id, p1);
        break;
    }
    case EV_MSG_RESET:
    {
        event.id = id;
        event.reset_payload.reset_state = p1;
        LOG(LOG_LEVEL_INFO, "扫描到复位: %d, reset_state: %d", event.id, p1);
        break;
    }
    case EV_MSG_ENANLE:
        event.id = id;
        event.enable_payload.enable_key = p1;
        event.enable_payload.enable_state = p2;
        LOG(LOG_LEVEL_INFO, "扫描到使能: %d, enable_state: %d", p1, p2);
        break;
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
static void process_mode_selection(char g43, char g19)
{
    CNC_MODE_INDEX target_mode = CNC_MODE_INVALID; // 默认无效模式

    // 1. 判定高优先级信号
    if (GET_BIT(g43, 7)) // 判断第7位
    {
        target_mode = CNC_MODE_ZRN; // ZRN 置位进入回零模式
    }
    else if (GET_BIT(g43, 5)) // 判断第5位
    {
        target_mode = CNC_MODE_DNC; // DNCI 置位
    }
    else if (GET_BIT(g19, 7))
    {
        target_mode = CNC_MODE_RAPID; // 进入 RAPID 模式
    }
    else
    {
        // 2. 判定MD4, MD2, MD1组合 (取低3位)
        unsigned char md_bits = g43 & 0x07;
        switch (md_bits)
        {
        case 0x01:
            target_mode = CNC_MODE_MEM;
            break; // 001
        case 0x02:
            target_mode = CNC_MODE_MDI;
            break; // 010
        case 0x03:
            target_mode = CNC_MODE_EDIT;
            break; // 011
        case 0x04:
            target_mode = CNC_MODE_MPG;
            break; // 100
        case 0x05:
            target_mode = CNC_MODE_JOG;
            break; // 101
        default:
            target_mode = CNC_MODE_INVALID;
            break;
        }
        // todo：856梯形图，fanuc
        // switch (md_bits)
        // {
        // case 0x01:
        //     target_mode = CNC_MODE_MEM;
        //     break; // 001
        // case 0x00:
        //     target_mode = CNC_MODE_MDI;
        //     break; // 000
        // case 0x03:
        //     target_mode = CNC_MODE_EDIT;
        //     break; // 011
        // case 0x04:
        //     target_mode = CNC_MODE_MPG;
        //     break; // 100
        // case 0x05:
        //     target_mode = CNC_MODE_JOG;
        //     break; // 101
        // default:
        //     target_mode = CNC_MODE_INVALID;
        //     break;
        // }
    }

    /* 如果解析出有效模式，则发送事件 */
    if (target_mode != CNC_MODE_INVALID)
    {
        pmc_dispatch_event(EV_MSG_MODE, (int)target_mode, 0);
    }
}
