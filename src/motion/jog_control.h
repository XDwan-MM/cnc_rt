/********************************************************************
 * @file        jog_control.h
 * @brief       JOG控制底层接口 - 负责参数配置与状态切换
 * @version     V1.00
 * @date        2026/03/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/
#ifndef JOG_CONTROL_H
#define JOG_CONTROL_H

#include <stdbool.h>
#include "simple_tp.h"
#include "pardata.h"

/* JOG方向定义 */
#define JOG_DIR_NONE      (0)
#define JOG_DIR_POSITIVE  (1)
#define JOG_DIR_NEGATIVE  (-1)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配置JOG/快速JOG 运动参数 (仅在按下按键瞬间或参数改变时调用)
 * @param axis_para   轴参数指针
 * @param jog_tp      规划器结构体
 * @param direction   方向 (1/-1)
 * @param feed_override 倍率 (0.0~1.5)
 * @param is_rapid    是否为快速点动
 */
void prepare_jog_params(const ParamUnit* axis_para, simple_tp_t *jog_tp, int direction, double feed_override, bool is_rapid);

/**
 * @brief 请求停止JOG
 * @param jog_tp      规划器结构体
 * @param immediate   是否立即硬停止（不走加减速）
 */
void request_jog_stop(simple_tp_t *jog_tp, bool immediate);

#ifdef __cplusplus
}
#endif

#endif /* JOG_CONTROL_H */

