#ifndef SIGNAL_UTILS_H
#define SIGNAL_UTILS_H

/**
 * @file signal_utils.h
 * @brief 提供基础位运算、边沿检测以及基于快照对象的语义化宏。
 */

/* ==========================================================================
 * 第一层：基础位提取宏
 * ========================================================================== */

/* 提取 val 的第 bit 位 (0-7) */
#define GET_BIT(val, bit) (((val) >> (bit)) & 0x01)

/* ==========================================================================
 * 第二层：基础边沿检测宏 (需传入当前值和旧值)
 * ========================================================================== */

/* 信号上升沿：当前为1，上次为0 */
#define SIGNAL_RISING(curr, last, bit) (GET_BIT(curr, bit) && !GET_BIT(last, bit))

/* 信号下降沿：当前为0，上次为1 */
#define SIGNAL_FALLING(curr, last, bit) (!GET_BIT(curr, bit) && GET_BIT(last, bit))

/* ==========================================================================
 * 第三层：对象语义化宏 (桥接快照结构体)
 * 约定：调用环境中必须存在 curr_obj 和 last_obj 两个结构体实例
 * ========================================================================== */

/**
 * @brief 检查指定寄存器组(obj)中某个字节是否发生变化
 * @param obj 结构体实例名 (如 curr_g)
 * @param l_obj 上一次快照实例名 (如 last_g)
 * @param idx 寄存器偏移索引
 */
#define SNAP_CHANGED(obj, l_obj, idx) ((obj).g_regs[idx] != (l_obj).g_regs[idx])

/**
 * @brief 在指定的快照对中检测上升沿
 * @param obj 当前快照实例 (curr_g)
 * @param l_obj 历史快照实例 (last_g)
 * @param idx 寄存器偏移索引 (如 7)
 * @param bit 位偏移 (如 2)
 */
#define SNAP_RISING(obj, l_obj, idx, bit) \
    SIGNAL_RISING((obj).g_regs[idx], (l_obj).g_regs[idx], bit)

/**
 * @brief 在指定的快照对中检测下降沿
 */
#define SNAP_FALLING(obj, l_obj, idx, bit) \
    SIGNAL_FALLING((obj).g_regs[idx], (l_obj).g_regs[idx], bit)

/**
 * @brief 获取当前快照中特定寄存器的值
 */
#define SNAP_VAL(obj, idx) ((obj).g_regs[idx])

#endif