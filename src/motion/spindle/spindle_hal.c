#include "rt_drive_control.h"
#include "emcmot.h"
/**
 * @file spindle_hal.c
 * @brief 主轴控制模块访问 HAL 的唯一适配层。
 *
 * 本文件隔离 HAL 状态结构与模式切换请求；使能需求交公共管理。上层状态机只
 * 使用带业务含义的函数，不再散落状态字掩码、反馈读取失败默认值和模式
 * 切换闩锁。HAL 的反馈快照在一个 PDO 周期内保持不变，因此同一运动周期
 * 多次查询不会跨越硬件采样边界。
 */

#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include <math.h>

/*
 * 本地镜像的初值必须与主轴默认 CSV 工作方式一致。实际模式及运行状态
 * 仍以 HAL 反馈为准；这里仅记录本模块最近一次输出目标和异步请求进度。
 */
spindle_hal_state_t g_spindle_hal = {
    .requested_mode = HAL_AXIS_MODE_CSV,
    .mode_switch_state = SPINDLE_MODE_SWITCH_IDLE,
    .mode_switch_result = HAL_OK
};

/* 每类 HAL 调用独立记住最近错误码，避免固定周期内重复刷屏。 */
static int s_last_hal_error[7] = { HAL_OK };

/* 内部函数声明 */

/* 读取 HAL 模式反馈并推进本地异步模式事务。 */
void spindle_hal_poll_mode_switch(void);
/* 记录 HAL 错误；同一调用点的相同错误只打印一次，成功后允许再次报告。 */
static int spindle_hal_log_hal_result(const char *api_name, int result, unsigned int call_site);

/* 外部函数定义 */

void spindle_hal_init(void)
{
    /*
     * 初始化本地输出镜像及异步事务状态。HAL 请求本身由 HAL 管理，
     * 此处不代表硬件已切换到 CSV；后续控制流程仍应读取实际反馈确认。
     */
    motion_spindle_release_position();
    (void)alarm_spindle_request_torque(false);
    g_spindle_hal.velocity_submitted=false;
    g_spindle_hal.target_velocity = 0.0;
    g_spindle_hal.target_position = 0.0;
    g_spindle_hal.requested_mode = HAL_AXIS_MODE_CSV;
    g_spindle_hal.mode_switch_state = SPINDLE_MODE_SWITCH_IDLE;
    g_spindle_hal.mode_switch_result = HAL_OK;
}

int spindle_hal_is_enabled(void)
{
    RtHalAxisStatus status;
    int result;

    /* 读取失败也返回 0，只能用于准入；确认实际下使能必须另查有效快照和驱动状态。 */
    result = rt_hal_axis_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_axis_status", result, 0U) != HAL_OK)
    {
        return 0;
    }
    return status.enabled;
}

int spindle_hal_mode(void)
{
    HalAxisSnapshot axis;
    if (spindle_hal_snapshot(&axis)!=HAL_OK || !(axis.feedback_valid&HAL_FEEDBACK_MODE))return -1;
    return axis.actual_mode;
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
    RtHalSpindleStatus status;
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
    RtHalAxisStatus status;
    int result;

    /* 状态字供安全仲裁检查驱动器故障位。 */
    result = rt_hal_axis_status(RT_HAL_AXIS_SP, &status);
    if (spindle_hal_log_hal_result("rt_hal_axis_status", result, 4U) != HAL_OK)
    {
        /* 反馈无效由控制入口统一处理；这里不要伪造成驱动器 Fault。 */
        return 0;
    }
    return status.raw_status;
}

int spindle_hal_orientation_state(void)
{
    int32_t state = RT_HAL_ORIENT_NONE;
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
    return spindle_hal_orientation_state() == RT_HAL_ORIENT_NONE;
}

bool spindle_hal_mode_switch_is_pending(void)
{
    /* 只查询本地业务阶段；由控制周期先调用 poll_mode_switch 更新。 */
    return g_spindle_hal.mode_switch_state == SPINDLE_MODE_SWITCH_PENDING;
}

bool spindle_hal_mode_switch_is_done(void)
{
    /* DONE 表示请求模式的准备已 READY；实际模式及使能仍须分别读有效反馈确认。 */
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
    /* 返回成功仅表示请求入缓冲；公共管理随后负责模式准备及明确使能。 */
    if(g_spindle_hal.mode_switch_state==SPINDLE_MODE_SWITCH_PENDING &&
       g_spindle_hal.requested_mode==mode)return HAL_OK;
    int rc=rt_hal_request_axis_mode(RT_HAL_AXIS_SP,(HalAxisMode)mode);
    g_spindle_hal.mode_switch_result=rc;
    g_spindle_hal.mode_switch_state=rc?SPINDLE_MODE_SWITCH_ERROR:SPINDLE_MODE_SWITCH_PENDING;
    if(!rc) {
        g_spindle_hal.requested_mode=mode;
        alarm_axis_mode_requested(RT_HAL_AXIS_SP,mode);
    }
    return rc;
}
int spindle_hal_snapshot(HalAxisSnapshot* out)
{
    return rt_hal_axis_snapshot(RT_HAL_AXIS_SP,out);
}
void spindle_hal_poll_mode_switch(void)
{
    if(g_spindle_hal.mode_switch_state!=SPINDLE_MODE_SWITCH_PENDING)return;
    HalAxisSnapshot axis;
    int rc=spindle_hal_snapshot(&axis);
    if(!rc && axis.request_rejection.category==HAL_ISSUE_MODE)rc=axis.request_rejection.code;
    if(!rc && axis.mode_setting_state==HAL_MODE_SETTING_FAILED)
        rc=axis.execution_error.category==HAL_ISSUE_MODE?axis.execution_error.code:HAL_ERROR_STATE;
    if(rc) {
        g_spindle_hal.mode_switch_result=rc;
        g_spindle_hal.mode_switch_state=SPINDLE_MODE_SWITCH_ERROR;
    } else if(axis.target_mode==g_spindle_hal.requested_mode &&
              axis.mode_setting_state==HAL_MODE_SETTING_READY) {
        /* 台达在这里可能仍然报不出实际模式；使能是另一件事。 */
        g_spindle_hal.mode_switch_state=SPINDLE_MODE_SWITCH_DONE;
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
