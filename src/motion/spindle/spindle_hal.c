/**
 * @file spindle_hal.c
 * @brief 主轴控制模块访问 HAL 的唯一适配层。
 *
 * 本文件隔离 HAL 状态结构、模式切换请求和主轴力矩意愿。上层状态机只
 * 使用带业务含义的函数，不再散落状态字掩码、反馈读取失败默认值和模式
 * 切换闩锁。HAL 的反馈快照在一个 PDO 周期内保持不变，因此同一运动周期
 * 多次查询不会跨越硬件采样边界。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include <math.h>

spindle_hal_state_t g_spindle_hal = {
    .requested_mode = HAL_SPINDLE_CSV,
    .mode_switch_state = SPINDLE_MODE_SWITCH_IDLE,
    .mode_switch_result = HAL_OK
};
static int s_last_hal_error[7] = { HAL_OK };

/* 内部函数声明 */

static void spindle_hal_poll_mode_switch(void);
static int spindle_hal_log_hal_result(const char *api_name, int result, unsigned int call_site);

/* 外部函数定义 */

void spindle_hal_init(void)
{
    /* 初始化不仅清输出镜像，也必须丢弃上一次运行遗留的异步模式事务。 */
    g_spindle_hal.target_velocity = 0.0;
    g_spindle_hal.target_position = 0.0;
    g_spindle_hal.requested_mode = HAL_SPINDLE_CSV;
    g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_IDLE;
    g_spindle_hal.mode_switch_result = HAL_OK;
}

int spindle_hal_is_enabled(void)
{
    HalCAxisStatus status;
    int result;

    result = rt_hal_axis_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_axis_status", result, 0U) != HAL_OK)
    {
        return 0;
    }
    return status.enabled;
}

int spindle_hal_mode(void)
{
    HalCSpindleStatus status;
    int result;

    result = rt_hal_spindle_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_spindle_status", result, 1U) != HAL_OK)
    {
        return -1;
    }
    return status.mode;
}

double spindle_hal_actual_velocity(void)
{
    double velocity = 0.0;
    int result;

    result = rt_hal_spindle_speed(RT_HAL_AXIS_SP, &velocity);
    if (spindle_hal_log_hal_result("rt_hal_spindle_speed", result, 2U) != HAL_OK)
    {
        /* NaN 会使所有“已到速/已停稳”比较失败，避免通信失败被误判为零速。 */
        return NAN;
    }
    return velocity;
}

double spindle_hal_actual_position(void)
{
    HalCSpindleStatus status;
    int result;

    result = rt_hal_spindle_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_spindle_status", result, 3U) != HAL_OK)
    {
        /* 位置未知时不能通过 CSP 位置保持和 M29 完成判定。 */
        return NAN;
    }
    return status.position_deg;
}

uint16_t spindle_hal_status_word(void)
{
    HalCAxisStatus status;
    int result;

    result = rt_hal_axis_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_axis_status", result, 4U) != HAL_OK)
    {
        /* 读取失败按 Fault 处理，安全仲裁必须优先停止主轴。 */
        return 0x0008U;
    }
    return status.raw_status;
}

int spindle_hal_orientation_state(void)
{
    int32_t state = HAL_ORIENT_NONE;
    int result;

    result = rt_hal_spindle_orient_state(RT_HAL_AXIS_SP, &state);
    if (spindle_hal_log_hal_result("rt_hal_spindle_orient_state", result, 5U) != HAL_OK)
    {
        /* -1 不等于任何合法定向阶段，调用方不会误认为已经释放。 */
        return -1;
    }
    return (int)state;
}

bool spindle_hal_orientation_is_inactive(void)
{
    return spindle_hal_orientation_state() == HAL_ORIENT_NONE;
}

void spindle_hal_request_torque(void)
{
    /* 主轴不能直接调用 HAL enable。所有轴的使能意愿由保护模块统一合并，
     * 从而保证急停和轴屏蔽始终拥有更高优先级。 */
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = true;
}

void spindle_hal_release_torque(void)
{
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = false;
}

bool spindle_hal_mode_switch_is_pending(void)
{
    spindle_hal_poll_mode_switch();
    return g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_PENDING;
}

bool spindle_hal_mode_switch_is_done(void)
{
    spindle_hal_poll_mode_switch();
    return g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_DONE;
}

int32_t spindle_hal_mode_switch_result(void)
{
    return g_spindle_hal.mode_switch_result;
}

void spindle_hal_mode_switch_reset(void)
{
    /* HAL 内部请求无法从这里撤销；复位本地事务表示上层不再等待旧请求。 */
    g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_IDLE;
    g_spindle_hal.mode_switch_result = HAL_OK;
}

int spindle_hal_request_mode(int mode)
{
    int result;

    /* 即使实际模式已经匹配也继续提交请求，避免 HAL 中尚有反向模式请求。 */
    result = rt_hal_request_spindle_mode(RT_HAL_AXIS_SP, (HalSpindleMode)mode);
    g_spindle_hal.mode_switch_result = spindle_hal_log_hal_result(
        "rt_hal_request_spindle_mode", result, 6U);
    if (g_spindle_hal.mode_switch_result != HAL_OK)
    {
        g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_ERROR;
        return g_spindle_hal.mode_switch_result;
    }

    g_spindle_hal.requested_mode = mode;
    g_spindle_hal.mode_switch_state = spindle_hal_mode() == (int)mode ?
        SPINDLE_MODE_SWITCH_DONE : SPINDLE_MODE_SWITCH_PENDING;
    return HAL_OK;
}

/* 内部函数定义 */

static void spindle_hal_poll_mode_switch(void)
{
    if (g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_PENDING &&
        spindle_hal_mode() == g_spindle_hal.requested_mode)
    {
        g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_DONE;
    }
}

static int spindle_hal_log_hal_result(const char *api_name, int result, unsigned int call_site)
{
    if (result != HAL_OK)
    {
        if (s_last_hal_error[call_site] != result)
        {
            LOG(LOG_LEVEL_ERR, "主轴 HAL 调用失败：%s axis=%d result=%d",
                api_name, RT_HAL_AXIS_SP, result);
            s_last_hal_error[call_site] = result;
        }
    }
    else
    {
        s_last_hal_error[call_site] = HAL_OK;
    }
    return result;
}
