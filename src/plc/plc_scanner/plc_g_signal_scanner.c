#include "plc_g_signal_scanner.h"
#include "global.h"
#include "regapi.h"       /**< 寄存器访问 API */
#include "signal_utils.h" /**< 边沿检测宏定义 */
#include "spl_control.h"
#include <hmisysapi.h>
#include <string.h>

/**
 * @brief 寄存器自动扫描注册表
 * * 作用：
 * 1. 集中管理所有需要从共享内存抓取到本地快照的 G 寄存器地址。
 * 2. 只有在此数组中注册过的索引，其对应的 g_regs[i] 才会包含有效的 共享内存G寄存器 数据。
 * 3. 以后若想增加 G19 或 G32，只需在此数组末尾添加数字 19, 32 即可，无需改动获取数据的函数。
 */
static const int SCAN_G_REGS[] = {
    4,  5,  6,   7,   8,   12,  14,  19,  29,  30,  32,  33,  43,  44,  45,  46,  54,  55,  56,  61,
    62, 70, 100, 101, 102, 103, 107, 108, 109, 118, 120, 124, 130, 132, 134, 341, 342, 343, 344, /* 已有及外部减速信号地址
                                                                                                  */
};

/** 计算注册表中元素的个数，用于驱动循环扫描 */
#define SCAN_G_REGS_COUNT (sizeof(SCAN_G_REGS) / sizeof(SCAN_G_REGS[0]))

static G_INPUT_MAP last_g; /**< 上一周期快照（是指任务控制器线程，上一周期的快照） */

/* 首次扫描标志 */
static bool is_first_scan = true;

/* ==========================================================================
 * 1. 内部函数声明 (Private Prototypes)
 * ========================================================================== */

/** @brief 从寄存器系统同步数据到本地结构体 */
static void plc_input_fetch_snapshot(G_INPUT_MAP *local_snap);

/** @brief 将解析出的逻辑打包为 PLC_EVENT 并入队 */
static void plc_dispatch_event(PLC_EVENT_ID id, int p1, int p2, int p3, int p4);

/** @brief 更新 PLC 主轴速度指令及 G33.7 速度来源选择。 */
static void process_spindle_speed_command(const G_INPUT_MAP *curr_g);

/** @brief 从 G32/G33 读取 PLC 主轴速度指令原始值。 */
static uint16_t read_spindle_speed_command_raw(const G_INPUT_MAP *curr_g);

/** @brief 扫描外部减速 G 信号 */
static void process_external_deceleration(const G_INPUT_MAP *curr_g);

/** @brief 扫描一个外部减速功能正负方向的 G 信号 */
static void process_external_deceleration_function(const G_INPUT_MAP *curr_g,
                                                   int function,
                                                   int positive_reg,
                                                   int negative_reg);

/**
 * @brief 模式解析逻辑实现
 * 涉及 MD1, MD2, MD4 (bit 0,1,2), DNCI (bit 5), ZRN (bit 7)
 */
static void process_mode_selection(char g43);

/* ==========================================================================
 * 2. 外部函数定义 (Public Definitions)
 * ========================================================================== */

/**
 * @brief 周期性扫描任务
 * 建议 1ms 周期调用。内部完成：拉取寄存器 -> 逻辑判断 -> 推送队列
 */
void plc_g_signal_scan_cycle(void)
{
    if (event_queue == NULL)
        return;

    G_INPUT_MAP curr_g;
    memset(&curr_g, 0, sizeof(G_INPUT_MAP)); /* 初始化清零，防止脏数据 */
    /* step1. 抓取寄存器快照 */
    plc_input_fetch_snapshot(&curr_g);

    /* step2. G信号检测 */

    /* 外部减速信号检测：G 位为0时，对应逻辑轴方向减速生效 */
    process_external_deceleration(&curr_g);

    /* G8.4紧急停止检测 G08.4 = 0急停拍下，G08.4 = 1急停拉起 */
    // 急停按下检测 （状态检测）
    if (!GET_BIT(curr_g.g_regs[8], 4))
    {
        // 急停按键拍下
        rt_set_sys_state(SYS_ESTOP_INPUT, true);
        // 本周期急停8.4为0，上周期急停8.4为1，急停拍下
        if (is_first_scan || GET_BIT(last_g.g_regs[8], 4))
        {
            LOG(LOG_LEVEL_INFO, "扫描到急停G8.4信号为0, 急停拍下");
        }
    }
    // 急停释放检测 0->1（上升沿）,自动触发系统复位，清除内部急停标志和报警
    if (!is_first_scan && SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 4))
    {
        LOG(LOG_LEVEL_INFO, "扫描到急停G8.4信号由0变1, 系统自动复位");
        // 系统急停输入信号
        rt_set_sys_state(SYS_ESTOP_INPUT, false);
        // 自动触发复位事件（推送复位事件）
        plc_dispatch_event(PLC_EV_MSG_RESET, 1, 0, 0, 0);
    }
    /* G19.7 快速按键信号检测，0->1 上升沿有效，1->0 下降沿失效 */
    if (SIGNAL_RISING(curr_g.g_regs[19], last_g.g_regs[19], 7))
    {
        LOG(LOG_LEVEL_INFO, "扫描到快速按键: 按下");
        // 快速按键按下
        if (rt_data->current_mode == MODE_JOG)
        {
            rt_data->sys_state[SYS_IS_RAPID] = true;
        }
    }
    else if (SIGNAL_FALLING(curr_g.g_regs[19], last_g.g_regs[19], 7))
    {
        LOG(LOG_LEVEL_INFO, "扫描到快速按键: 释放");
        // 快速按键释放
        rt_data->sys_state[SYS_IS_RAPID] = false;
    }

    /* G43模式检测：调用内部解析函数处理复合位 (MD1, MD2, MD4, DNCI, ZRN), * 输出模式 */
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[43], last_g.g_regs[43]))
    {
        process_mode_selection(curr_g.g_regs[43]);
    }

    /* (G100/G102)进给轴按键检测 - 支持多轴同时扫描 */
    for (int i = 0; i < 3; i++)
    {
        /* 正常边沿检测 */
        // 正向按键检测
        if (SIGNAL_RISING(curr_g.g_regs[100], last_g.g_regs[100], i))
        {
            plc_dispatch_event(PLC_EV_MSG_FEEDAXIS_KEY, i, 1, 0, 0);
        }
        else if (SIGNAL_FALLING(curr_g.g_regs[100], last_g.g_regs[100], i))
        {
            plc_dispatch_event(PLC_EV_MSG_FEEDAXIS_KEY, i, -1, 0, 0);
        }
        // 负向按键检测
        if (SIGNAL_RISING(curr_g.g_regs[102], last_g.g_regs[102], i))
        {
            plc_dispatch_event(PLC_EV_MSG_FEEDAXIS_KEY, i, 2, 0, 0);
        }
        if (SIGNAL_FALLING(curr_g.g_regs[102], last_g.g_regs[102], i))
        {
            plc_dispatch_event(PLC_EV_MSG_FEEDAXIS_KEY, i, -2, 0, 0);
        }
    }

    /* (G12/G14) 倍率检测 */
    // 进给倍率变化检测G12
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[12], last_g.g_regs[12]))
    {
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
        /* p1=0 代表进给倍率类型 (PLC_EV_MSG_OVERRIDE) */
        plc_dispatch_event(PLC_EV_MSG_OVERRIDE, 0, feed_override, 0, 0);
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
        plc_dispatch_event(PLC_EV_MSG_OVERRIDE, 1, rapid_lut[curr_g.g_regs[14] & 0x03], 0, 0);
    }
    // 主轴倍率检测G30
    if (is_first_scan || SIGNAL_BYTE_CHANGED(curr_g.g_regs[30], last_g.g_regs[30]))
    {
        /* p1=2 代表主轴倍率类型 */
        plc_dispatch_event(PLC_EV_MSG_OVERRIDE, 2, (int)curr_g.g_regs[30], 0, 0);
    }

    /* 自动运行控制 (G7.2 启动, G8.5 中止, G8.7 复位) */
    // 启动检测 (G7.2 上升沿 【暂定上升沿触发】)
    if (SIGNAL_RISING(curr_g.g_regs[7], last_g.g_regs[7], 2))
    {
        LOG(LOG_LEVEL_INFO, "扫描到循环启动G7.2发生了上升沿");
        // G7.2 产生上升沿，发送自动运行事件
        plc_dispatch_event(PLC_EV_MSG_AUTO_RUN, 1, 0, 0, 0);
    }
    // 暂停检测 (G8.5 下降沿，发送暂停事件)
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 5))
    {
        LOG(LOG_LEVEL_INFO, "扫描到暂停G8.5发生了下降沿");
        plc_dispatch_event(PLC_EV_MSG_AUTO_RUN, 0, 0, 0, 0);
    }

    /* 外部复位M02M30 (G8.7 复位 上升沿触发) */
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 7))
    {
        LOG(LOG_LEVEL_INFO, "扫描到外部复位G8.7发生了上升沿");
        plc_dispatch_event(PLC_EV_MSG_RESET, 1, 1, 0, 0);
    }

    /* 主轴正反信号检测 (G70.4 主轴反转, G70.5 主轴正转, G29.6 主轴停止信号) */
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 5))
    {
        LOG(LOG_LEVEL_INFO, "扫描到主轴正转G70.5发生了上升沿");
        // 产生主轴正转事件 入队
        plc_dispatch_event(PLC_EV_MSG_SPINDLE, 1, 0, 0, 0);
    }
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 4))
    {
        LOG(LOG_LEVEL_INFO, "扫描到主轴反转G70.4发生了上升沿");
        // 产生主轴反转事件 入队
        plc_dispatch_event(PLC_EV_MSG_SPINDLE, -1, 0, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[29], last_g.g_regs[29], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到主轴停止G29.6发生了下降沿");
        // 产生主轴停止事件 入队
        plc_dispatch_event(PLC_EV_MSG_SPINDLE, 0, 0, 0, 0);
    }

    /* 主轴定向 G70.6 */
    if (SIGNAL_RISING(curr_g.g_regs[70], last_g.g_regs[70], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到主轴定向G70.6 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_SPD_ORI, 1, 0, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[70], last_g.g_regs[70], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到主轴定向G70.6 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_SPD_ORI, -1, 0, 0, 0);
    }

    /* 可选程序段跳过 G44.0 */
    if (SIGNAL_RISING(curr_g.g_regs[44], last_g.g_regs[44], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G44.0 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, BLOCK_SKIP, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[44], last_g.g_regs[44], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G44.0 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, BLOCK_SKIP, 0, 0, 0);
    }
    /* 跳段 G45发生变化 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[45], last_g.g_regs[45]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[45], last_g.g_regs[45], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G45.%d 发生上升沿", i);
                plc_dispatch_event(PLC_EV_MSG_ENANLE, BLOCK_SKIP + i + 1, 1, 0, 0);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[45], last_g.g_regs[45], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G45.%d 发生下降沿", i);
                plc_dispatch_event(PLC_EV_MSG_ENANLE, BLOCK_SKIP + i + 1, 0, 0, 0);
            }
        }
    }
    /* 读取 PLC 主轴速度指令，并解析 G33.7 的速度来源选择。 */
    process_spindle_speed_command(&curr_g);

    /* 程序再启动 G6.0 */
    if (SIGNAL_RISING(curr_g.g_regs[6], last_g.g_regs[6], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G6.0 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, PROG_RESTAR, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[6], last_g.g_regs[6], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G6.0 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, PROG_RESTAR, 0, 0, 0);
    }

    /* 空运行 G46.7 */
    if (SIGNAL_RISING(curr_g.g_regs[46], last_g.g_regs[46], 7))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G46.7 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, DRY_RUN, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[46], last_g.g_regs[46], 7))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G46.7 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, DRY_RUN, 0, 0, 0);
    }

    /* 单程序段 G46.1 */
    if (SIGNAL_RISING(curr_g.g_regs[46], last_g.g_regs[46], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G46.1 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, SINGLE_BLOCK, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[46], last_g.g_regs[46], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G46.1 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, SINGLE_BLOCK, 0, 0, 0);
    }

    /* 辅助功能锁住 G5.6 */
    if (SIGNAL_RISING(curr_g.g_regs[5], last_g.g_regs[5], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G5.6 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, MSBT_LOCK, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[5], last_g.g_regs[5], 6))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G5.6 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, MSBT_LOCK, 0, 0, 0);
    }

    /* 机床锁住 G4.1 */
    if (SIGNAL_RISING(curr_g.g_regs[4], last_g.g_regs[4], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G4.1 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, MAC_LOCK, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[4], last_g.g_regs[4], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G4.1 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, MAC_LOCK, 0, 0, 0);
    }

    /* 起动锁停 G7.1 1有效*/
    if (SIGNAL_RISING(curr_g.g_regs[7], last_g.g_regs[7], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G7.1 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, STLK, 1, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[7], last_g.g_regs[7], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G7.1 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, STLK, 0, 0, 0);
    }

    /* 所有互锁 G8.0 0有效*/
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.0 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, IT, 0, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 0))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.0 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, IT, 1, 0, 0);
    }

    /* 切削程序段开始互锁 G8.1 0有效*/
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.1 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, CSL, 0, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 1))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.1 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, CSL, 1, 0, 0);
    }

    /* 程序段开始互锁 G8.3 0有效*/
    if (SIGNAL_RISING(curr_g.g_regs[8], last_g.g_regs[8], 3))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.3 发生上升沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, BSL, 0, 0, 0);
    }
    if (SIGNAL_FALLING(curr_g.g_regs[8], last_g.g_regs[8], 3))
    {
        LOG(LOG_LEVEL_INFO, "扫描到G8.3 发生下降沿");
        plc_dispatch_event(PLC_EV_MSG_ENANLE, BSL, 1, 0, 0);
    }

    /* 各轴互锁 G130.0-130.4 0有效*/
    for (int i = 0; i < 5; i++)
    {
        if (SIGNAL_RISING(curr_g.g_regs[130], last_g.g_regs[130], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G130.%d 发生上升沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, IT1 + i, 0, 0, 0);
        }
        if (SIGNAL_FALLING(curr_g.g_regs[130], last_g.g_regs[130], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G130.%d 发生下降沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, IT1 + i, 1, 0, 0);
        }
    }

    /* 不同轴向互锁 G132.0-132.4 1有效*/
    for (int i = 0; i < 5; i++)
    {
        if (SIGNAL_RISING(curr_g.g_regs[132], last_g.g_regs[132], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G132.%d 发生上升沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, PMIT1 + i, 1, 0, 0);
        }
        if (SIGNAL_FALLING(curr_g.g_regs[132], last_g.g_regs[132], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G132.%d 发生下降沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, PMIT1 + i, 0, 0, 0);
        }
    }
    /* 不同轴向互锁 G134.0-134.4 1有效*/
    for (int i = 0; i < 5; i++)
    {
        if (SIGNAL_RISING(curr_g.g_regs[134], last_g.g_regs[134], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G134.%d 发生上升沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, NMIT1 + i, 1, 0, 0);
        }
        if (SIGNAL_FALLING(curr_g.g_regs[134], last_g.g_regs[134], i))
        {
            LOG(LOG_LEVEL_INFO, "扫描到G134.%d 发生下降沿", i);
            plc_dispatch_event(PLC_EV_MSG_ENANLE, NMIT1 + i, 0, 0, 0);
        }
    }

    /* G54 用户宏程序输入指令*/
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[54], last_g.g_regs[54]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[54], last_g.g_regs[54], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G54.%d 发生上升沿", i);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[54], last_g.g_regs[54], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G54.%d 发生下降沿", i);
            }
        }
    }
    /* G55 用户宏程序输入指令*/
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[55], last_g.g_regs[55]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[55], last_g.g_regs[55], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G55.%d 发生上升沿", i);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[55], last_g.g_regs[55], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G55.%d 发生下降沿", i);
            }
        }
    }
    /* G56 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[56], last_g.g_regs[56]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[56], last_g.g_regs[56], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G56.%d 发生上升沿", i);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[56], last_g.g_regs[56], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G56.%d 发生下降沿", i);
            }
        }
    }
    /* 各轴机床锁住 G108 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[108], last_g.g_regs[108]))
    {
        for (int i = 0; i < 8; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[108], last_g.g_regs[108], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G108.%d 发生上升沿", i);
                plc_dispatch_event(PLC_EV_MSG_ENANLE, MAC_LOCK + i + 1, 1, 0, 0);
            }
            if (SIGNAL_FALLING(curr_g.g_regs[108], last_g.g_regs[108], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G108.%d 发生下降沿", i);
                plc_dispatch_event(PLC_EV_MSG_ENANLE, MAC_LOCK + i + 1, 0, 0, 0);
            }
        }
    }
    /* 各轴屏蔽 G124 */
    if (SIGNAL_BYTE_CHANGED(curr_g.g_regs[124], last_g.g_regs[124]))
    {
        for (int i = 0; i < 5; i++)
        {
            if (SIGNAL_RISING(curr_g.g_regs[124], last_g.g_regs[124], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到轴屏蔽G124.%d发生上升沿 %d轴被屏蔽", i, i);
                rt_data->axis_data[i].status.plc_is_disable = 1;
            }
            if (SIGNAL_FALLING(curr_g.g_regs[124], last_g.g_regs[124], i))
            {
                LOG(LOG_LEVEL_INFO, "扫描到轴屏蔽G124.%d发生下降沿 %d轴解除屏蔽", i, i);
                rt_data->axis_data[i].status.plc_is_disable = 0;
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

static void process_spindle_speed_command(const G_INPUT_MAP *curr_g)
{
    const uint16_t raw_speed = read_spindle_speed_command_raw(curr_g);
    const double spindle_max_speed =
        par_axis[AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real;

    /* G32/G33 的模拟量编码按机床最大主轴速度换算为 rpm。 */
    spl_context.plc_speed = spindle_max_speed * raw_speed / 4095.0;

    /* G33.7=0：使用 CNC/M 代码速度；G33.7=1：使用 PLC 速度。 */
    if (GET_BIT(curr_g->g_regs[33], 7))
    {
        spl_context.speed_source = SPINDLE_SPEED_SOURCE_PLC;
    }
    else
    {
        spl_context.speed_source = SPINDLE_SPEED_SOURCE_CNC;
    }
}

static uint16_t read_spindle_speed_command_raw(const G_INPUT_MAP *curr_g)
{
    uint16_t low_bits = 0;
    uint16_t high_bits = 0;

    for (int bit = 0; bit < 8; bit++)
    {
        low_bits |= (uint16_t)(GET_BIT(curr_g->g_regs[32], bit) & 1) << bit;
        high_bits |= (uint16_t)(GET_BIT(curr_g->g_regs[33], bit) & 1) << bit;
    }

    /* G32/G33 的编码格式由 PLC 接口协议定义，保留原始位拼接方式。 */
    return (uint16_t)((high_bits & 0x0F) << 5) | (low_bits & 0xFF);
}

/** @brief 从寄存器系统同步数据到本地结构体
 * 遍历注册表，集中抓取数据，数据安全且无遗漏。
 */
static void plc_input_fetch_snapshot(G_INPUT_MAP *local_snap)
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

/** @brief 将解析出的逻辑打包为 PLC_EVENT 并入队 */
static void plc_dispatch_event(PLC_EVENT_ID id, int p1, int p2, int p3, int p4)
{
    if (id == PLC_EV_MSG_NONE)
        return;
    PLC_EVENT event;
    switch (id)
    {
    case PLC_EV_MSG_MODE:
    {
        event.id = id;
        event.mode_payload.mode = p1; // 目标模式
        // LOG(LOG_LEVEL_INFO, "扫描到模式切换 %d", id);
        break;
    }
    case PLC_EV_MSG_FEEDAXIS_KEY:
    {
        event.id = id;
        event.feedaxis_key_payload.axis_num = p1;
        event.feedaxis_key_payload.direction = p2;
        // LOG(LOG_LEVEL_INFO, "扫描到进给轴按键切换: %d, axis: %d, action: %d", event.id, p1, p2);
        break;
    }
    case PLC_EV_MSG_SPINDLE:
    {
        event.id = id;
        event.spindle_payload.direction = p1;
        // LOG(LOG_LEVEL_INFO, "扫描到主轴状态切换 sp:%d, direction: %d", event.id, p1);
        break;
    }
    case PLC_EV_MSG_OVERRIDE:
    {
        event.id = id;
        event.override_payload.override_type = p1;
        event.override_payload.val = p2;
        // LOG(LOG_LEVEL_INFO, "扫描到倍率切换 type: %d, override_type: %d, val: %d", event.id, p1, p2);
        break;
    }
    case PLC_EV_MSG_AUTO_RUN:
    {
        event.id = id;
        event.run_payload.run_state = p1;
        // LOG(LOG_LEVEL_INFO, "扫描到运行状态切换 run:%d, state: %d", event.id, p1);
        break;
    }
    case PLC_EV_MSG_RESET:
    {
        event.id = id;
        event.reset_payload.reset_state = p1;
        event.reset_payload.reset_type = p2;
        // LOG(LOG_LEVEL_INFO, "扫描到复位: %d, reset_state: %d", event.id, p1);
        break;
    }
    case PLC_EV_MSG_ENANLE:
        event.id = id;
        event.enable_payload.enable_key = p1;
        event.enable_payload.enable_state = p2;
        // LOG(LOG_LEVEL_INFO, "扫描到使能: %d, enable_state: %d", p1, p2);
        break;
    case PLC_EV_MSG_SPD_ORI:
        event.id = id;
        event.spd_ori_payload.p1 = p1;
        event.spd_ori_payload.p2 = p2;
        // LOG(LOG_LEVEL_INFO, "扫描到主轴定向: %d", event.id);
        break;
    case PLC_EV_MSG_EXTERNAL_DECEL:
        event.id = id;
        event.external_decel_payload.function = p1;
        event.external_decel_payload.axis_num = p2;
        event.external_decel_payload.direction = p3;
        event.external_decel_payload.active = p4;
        break;
    default:
        break;
    }
    /* 推送 PLC_EVENT 到线程安全的队列 */
    event_queue_push(event_queue, &event);
}

/** @brief 扫描外部减速 G 信号 */
static void process_external_deceleration(const G_INPUT_MAP *curr_g)
{
    /* 功能1：正向 G118，负向 G120 */
    process_external_deceleration_function(curr_g, PLC_EXTERNAL_DECEL_FUNCTION_1, 118, 120);

    /* 功能2：正向 G101，负向 G103 */
    process_external_deceleration_function(curr_g, PLC_EXTERNAL_DECEL_FUNCTION_2, 101, 103);

    /* 功能3：正向 G107，负向 G109 */
    process_external_deceleration_function(curr_g, PLC_EXTERNAL_DECEL_FUNCTION_3, 107, 109);

    /* 功能4：正向 G341，负向 G342 */
    process_external_deceleration_function(curr_g, PLC_EXTERNAL_DECEL_FUNCTION_4, 341, 342);

    /* 功能5：正向 G343，负向 G344 */
    process_external_deceleration_function(curr_g, PLC_EXTERNAL_DECEL_FUNCTION_5, 343, 344);
}

/** @brief 扫描一个外部减速功能正负方向的 G 信号 */
static void process_external_deceleration_function(const G_INPUT_MAP *curr_g,
                                                   int function,
                                                   int positive_reg,
                                                   int negative_reg)
{
    for (int axis = 0; axis < PLC_EXTERNAL_DECEL_AXIS_COUNT; axis++)
    {
        // 首次扫描
        if (is_first_scan)
        {
            int positive_active = 0;
            int negative_active = 0;

            positive_active = !GET_BIT(curr_g->g_regs[positive_reg], axis);
            if (positive_active)
            {
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, 1, 1);
            }

            negative_active = !GET_BIT(curr_g->g_regs[negative_reg], axis);
            if (negative_active)
            {
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, -1, 1);
            }
        }
        else
        {
            // 正向减速G信号扫描
            if (SIGNAL_FALLING(curr_g->g_regs[positive_reg], last_g.g_regs[positive_reg], axis))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G%d.%d 发生下降沿", positive_reg, axis);
                // 正向减速信号生效
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, 1, 1);
            }
            else if (SIGNAL_RISING(curr_g->g_regs[positive_reg], last_g.g_regs[positive_reg], axis))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G%d.%d 发生上升沿", positive_reg, axis);
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, 1, 0);
            }
            // 负向减速G信号扫描
            if (SIGNAL_FALLING(curr_g->g_regs[negative_reg], last_g.g_regs[negative_reg], axis))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G%d.%d 发生下降沿", negative_reg, axis);
                // 负向减速信号生效
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, -1, 1);
            }
            else if (SIGNAL_RISING(curr_g->g_regs[negative_reg], last_g.g_regs[negative_reg], axis))
            {
                LOG(LOG_LEVEL_INFO, "扫描到G%d.%d 发生上升沿", negative_reg, axis);
                plc_dispatch_event(PLC_EV_MSG_EXTERNAL_DECEL, function, axis, -1, 0);
            }
        }
    }
}

/**
 * @brief 模式解析逻辑实现
 * 涉及 MD1, MD2, MD4 (bit 0,1,2), DNCI (bit 5), ZRN (bit 7)
 */
static void process_mode_selection(char g43)
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
    else
    {
        // 2. 判定MD4, MD2, MD1组合 (取低3位)
        unsigned char md_bits = g43 & 0x07;
        switch (md_bits)
        {
        case 0x00:
            target_mode = CNC_MODE_MDI;
            break; // 000
        case 0x01:
            target_mode = CNC_MODE_MEM;
            break; // 001
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
    }

    /* 如果解析出有效模式，则发送事件 */
    if (target_mode != CNC_MODE_INVALID)
    {
        plc_dispatch_event(PLC_EV_MSG_MODE, (int)target_mode, 0, 0, 0);
    }
}
