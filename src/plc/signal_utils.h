#ifndef SIGNAL_UTILS_H
#define SIGNAL_UTILS_H

/**
 * @file signal_utils.h
 * @brief 提供基础位运算及边沿检测宏。
 */

/* 基础位提取：提取 val 的第 bit 位 */
#define GET_BIT(val, bit)         (((val) >> (bit)) & 0x01)

/* 信号上升沿：当前为1，上次为0 (按键按下) */
#define SIGNAL_RISING(curr, last, bit)   (GET_BIT(curr, bit) && !GET_BIT(last, bit))

/* 信号下降沿：当前为0，上次为1 (按键松开) */
#define SIGNAL_FALLING(curr, last, bit)  (!GET_BIT(curr, bit) && GET_BIT(last, bit))

/* 信号变化：当前值不等于上次值 */
#define SIGNAL_CHANGED(curr, last, bit)  (GET_BIT(curr, bit) != GET_BIT(last, bit))

/* 信号稳定：当前值等于指定电平 */
#define SIGNAL_STEADY(curr, bit, level)  (GET_BIT(curr, bit) == (level))

#endif
