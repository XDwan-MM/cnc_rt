/********************************************************************
 * @file        simple_tp.h
 * @brief       1、简单轨迹规划器
 * @version     V1.00
 * @date        2025/12/20
 * @author      伍臣发
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#include "simple_tp.h"
#include <stddef.h>
#include <math.h>

void simple_tp_update(simple_tp_t *tp, double period)
{
    double max_dv, tiny_dp, pos_err, vel_req;

    if (tp == NULL) return;

    tp->active = 0;
    
    /* 计算每个伺服周期的最大速度变化量 */
    max_dv = tp->max_acc * period;
    
    /* 计算微小位置阈值，在这个阈值内视为0 */
    tiny_dp = TINY_DP(tp->max_acc, period);
    
    /* 计算请求速度 */
    if (tp->enable) {
        /* 规划器使能，请求一个将位置误差驱动至零的速度，
           但允许停止时没有位置过冲 */
        pos_err = tp->pos_cmd - tp->curr_pos;
        
        /* 正负误差需要进行符号处理，以避免计算平方根时出现负数 */
        if (pos_err > tiny_dp) {
            vel_req = -max_dv + sqrt(2.0 * tp->max_acc * pos_err + max_dv * max_dv);
            tp->active = 1;
        } else if (pos_err < -tiny_dp) {
            vel_req = max_dv - sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
            tp->active = 1;
        } else {
            /* 位置误差在微小容差范围内，不需要移动 */
            vel_req = 0.0;
        }
    } else {
        /* 规划器未使能，请求一个零速度 */
        vel_req = 0.0;
        /* 设置命令位置为当前位置，以避免下次使能时的运动 */
        tp->pos_cmd = tp->curr_pos;
    }

    /* 限制请求速度 */
    if (vel_req > tp->max_vel) {
        vel_req = tp->max_vel;
    } else if (vel_req < -tp->max_vel) {
        vel_req = -tp->max_vel;
    }

    /* 在加速度限制下，将速度向请求值过渡 */
    if (vel_req > tp->curr_vel + max_dv) {
        tp->curr_vel += max_dv;
    } else if (vel_req < tp->curr_vel - max_dv) {
        tp->curr_vel -= max_dv;
    } else {
        tp->curr_vel = vel_req;
    }
    
    /* 检查是否仍在移动 */
    if (tp->curr_vel != 0.0) {
        tp->active = 1;
    }

    /* 对速度进行积分，得到新位置 */
    tp->curr_pos += tp->curr_vel * period;
}

