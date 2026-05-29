#ifndef COMP_COMMON_H
#define COMP_COMMON_H

#include <stdbool.h>
#include <math.h>
#include <stddef.h> /* 必须包含，确保 NULL 的定义 */

/* 定义物理安全边界：50um = 0.050mm，各项误差补偿（螺距误差，垂直度）最大不超过50um */
#define MAX_COMP_LIMIT_PER (0.050)
/* 定义发生运动判定的速度阈值 */
#define VEL_THRESHOLD (1e-7)

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 补偿通用状态机枚举
     * 用于处理补偿开启、关闭及参数变更时的平滑过渡
     */
    typedef enum
    {
        COMP_STATE_IDLE = 0, /* 待机状态：补偿关闭或上电未运动，输出为0 */
        COMP_STATE_RAMPING,  /* 过渡状态：入场、退场或类型切换时的平滑爬坡 */
        COMP_STATE_ACTIVE    /* 激活状态：正常加工中，指令位置直接叠加理想补偿值 */
    } CompState;

#ifdef __cplusplus
}
#endif

#endif
