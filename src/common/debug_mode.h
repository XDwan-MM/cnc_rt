#ifndef DEBUG_MODE_H
#define DEBUG_MODE_H

/**
 * @file debug_mode.h
 * @brief 调试模式配置头文件
 *
 * 用于在调试模式和实时模式之间切换：
 * - 调试模式：使用普通线程，可进行断点调试，无需硬件和root权限
 * - 实时模式：使用实时线程，需要EtherCAT硬件和root权限
 */

// ============ 调试模式开关 ============
// 编译时开关：定义此宏启用调试模式，注释掉使用实时模式
// 也可通过编译参数 -DDEBUG_MODE_ENABLED 来启用
// #define DEBUG_MODE_ENABLED

// ============ 模式检测宏 ============
#ifdef DEBUG_MODE_ENABLED
    #define IS_DEBUG_MODE 1
#else
    #define IS_DEBUG_MODE 0
#endif

// ============ 周期时间配置 ============
#define DEBUG_CYCLE_TIME_US 1000  // 调试模式周期: 1ms 
#define RT_CYCLE_TIME_US    1000   // 实时模式周期: 1ms

// ============ 调试模式线程配置 ============
#ifdef DEBUG_MODE_ENABLED
    #define DEBUG_THREAD_POLICY   SCHED_OTHER  // 普通调度策略
    #define DEBUG_THREAD_PRIORITY 0            // 普通优先级
    #define DEBUG_CORE_ID         -1           // 不绑定CPU核心
#endif

// ============ 调试输出宏 ============
#ifdef DEBUG_MODE_ENABLED
    #include <stdio.h>
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

#endif // DEBUG_MODE_H
