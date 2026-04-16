#ifndef SIGNAL_UTILS_H
#define SIGNAL_UTILS_H

/**
 * @file signal_utils.h
 * @brief 提供基础位运算及边沿检测宏。
 */

/* 第一层：基础位提取 */
#define GET_BIT(val, bit) (((val) >> (bit)) & 0x01)

/* 第二层：通用边沿检测 (需手动传入当前字节值和上一次字节值) */
/* 信号上升沿：当前为1，上次为0 */
#define SIGNAL_RISING(curr, last, bit) (GET_BIT(curr, bit) && !GET_BIT(last, bit))

/* 信号下降沿：当前为0，上次为1 */
#define SIGNAL_FALLING(curr, last, bit) (!GET_BIT(curr, bit) && GET_BIT(last, bit))

/* 信号变化：当前字节不等于上次字节 */
#define SIGNAL_BYTE_CHANGED(curr, last) ((curr) != (last))

/**
 * @brief 信号位变化检测
 * 只要该位发生了跳变（0->1 或 1->0），均返回真
 */
#define SIGNAL_BIT_CHANGED(curr, last, bit) (GET_BIT(curr, bit) != GET_BIT(last, bit))

#endif