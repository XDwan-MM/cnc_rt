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

/*
 * 本地镜像的初值必须与主轴默认 CSV 工作方式一致。实际模式及运行状态
 * 仍以 HAL 反馈为准；这里仅记录本模块最近一次输出目标和异步请求进度。
 */
spindle_hal_state_t g_spindle_hal = {
    .requested_mode = HAL_SPINDLE_CSV,
    .mode_switch_state = SPINDLE_MODE_SWITCH_IDLE,
    .mode_switch_result = HAL_OK
};

/* 每类 HAL 调用独立记住最近错误码，避免固定周期内重复刷屏。 */
static int s_last_hal_error[7] = { HAL_OK };

/* 内部函数声明 */

/* 读取 HAL 模式反馈并推进本地异步模式事务。 */
static void spindle_hal_poll_mode_switch(void);
/* 记录 HAL 错误；同一调用点的相同错误只打印一次，成功后允许再次报告。 */
static int spindle_hal_log_hal_result(const char *api_name, int result, unsigned int call_site);

/* 外部函数定义 */

void spindle_hal_init(void)
{
    /*
     * 初始化本地输出镜像及异步事务状态。HAL 请求本身由 HAL 管理，
     * 此处不代表硬件已切换到 CSV；后续控制流程仍应读取实际反馈确认。
     */
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

    /* 使能状态来自轴状态快照；读取失败时返回未使能，按不可用处理。 */
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

    /* 返回 HAL 当前反馈模式；-1 专用于表示状态读取失败。 */
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

    /* 速度是驱动器实际反馈，不是 g_spindle_hal 中保存的规划目标。 */
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

    /* HAL 返回的位置以角度表示；读取错误用 NaN 保留“未知”语义。 */
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

    /* 状态字供安全仲裁检查驱动器故障位。 */
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

    /* 定向状态与普通轴状态分开读取；-1 表示反馈无效，而非某个阶段。 */
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
    /* 只有 HAL 明确报告 NONE 才认为定向已释放，读取失败不会放行。 */
    return spindle_hal_orientation_state() == HAL_ORIENT_NONE;
}

void spindle_hal_request_torque(void)
{
    /*
     * 这里只设置主轴的力矩意愿，不直接操作驱动使能。保护/使能仲裁模块
     * 汇总各轴意愿并应用急停、轴屏蔽等优先条件后，才决定是否真正使能。
     */
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = true;
}

void spindle_hal_release_torque(void)
{
    /* 撤销主轴力矩意愿；最终硬件使能状态由统一仲裁流程更新。 */
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = false;
}

bool spindle_hal_mode_switch_is_pending(void)
{
    /* 查询前先采样模式反馈，使事务状态与当前 HAL 快照同步。 */
    spindle_hal_poll_mode_switch();
    return g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_PENDING;
}

bool spindle_hal_mode_switch_is_done(void)
{
    /* DONE 仅表示反馈模式与最近请求一致，不代表其他运行条件已满足。 */
    spindle_hal_poll_mode_switch();
    return g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_DONE;
}

int32_t spindle_hal_mode_switch_result(void)
{
    /* 返回最近一次模式切换提交结果；异步等待状态由 mode_switch_state 表示。 */
    return g_spindle_hal.mode_switch_result;
}

void spindle_hal_mode_switch_reset(void)
{
    /*
     * HAL 内部请求无法从这里撤销。复位只清理本地等待状态，表示上层
     * 不再跟踪旧事务；若 HAL 仍执行旧请求，实际模式反馈仍会如实返回。
     */
    g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_IDLE;
    g_spindle_hal.mode_switch_result = HAL_OK;
}

int spindle_hal_request_mode(int mode)
{
    int result;

    /*
     * 每次都提交新请求，即使当前反馈已匹配目标模式；这样可覆盖 HAL
     * 内部可能尚未完成的相反请求。成功提交后再依据反馈初始化事务状态。
     */
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
    /* HAL 请求接口为异步语义；实际模式反馈匹配目标时才转入 DONE。 */
    if (g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_PENDING &&
        spindle_hal_mode() == g_spindle_hal.requested_mode)
    {
        g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_DONE;
    }
}

static int spindle_hal_log_hal_result(const char *api_name, int result, unsigned int call_site)
{
    /*
     * call_site 为各 HAL 调用类别分配的固定槽位。错误码改变时记录一次；
     * 同一错误持续存在期间抑制重复日志，恢复成功后清除该槽位以便未来
     * 再次出现相同故障时仍能报告。返回原结果，便于调用方继续判定。
     */
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
