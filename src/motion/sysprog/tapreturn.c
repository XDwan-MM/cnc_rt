/********************************************************************
 * @file        tapreturn.c
 * @brief       刚性攻丝中断回退：断点参数记录 + G30 P99 回退程序生成
 ********************************************************************/

#include "tapreturn.h"
#include "sysprog.h"
#include "global.h"
#include "rt_data.h"
#include <stdio.h>
#include <string.h>
#include <tapreturnapi.h>
#include <stdint.h>
#include <hmirequestdata.h>
#include "alarmapi.h"

static TAP_RETURN_DATA tap_return_data;


static bool saved_record_is_sane(const TAP_RETURN_DATA *d)
{
    if (d->state < TAPRET_IDLE || d->state > TAPRET_DONE) return false;
    if (d->state == TAPRET_IDLE) return true;       // 无记录：其余字段无意义
    return (d->screw_offset > 0.0) && (d->spindle_speed > 0);
}

static void tapreturn_data_persist(void)
{
    if (setTapReturnData(&tap_return_data) != 0)
    {
        LOG(LOG_LEVEL_WARN, "断电块写入失败（共享内存未就绪），本次未落盘（记录仍在 RAM 里）");
        return;
    }
    LOG(LOG_LEVEL_DEBUG, "断电块已更新: state=%d", (int)tap_return_data.state);
}



void tapreturn_data_restore(void)
{
    TAP_RETURN_DATA restored;

    if (getTapReturnData(&restored) != 0)
    {
        LOG(LOG_LEVEL_WARN, "断电块不可用（共享内存未就绪）：按无记录启动");
        return;
    }

    if (!saved_record_is_sane(&restored))
    {
        LOG(LOG_LEVEL_WARN, "断电块内容不自洽(state=%d pitch=%.4f S=%d)，按无记录处理",
            (int)restored.state, restored.screw_offset, restored.spindle_speed);
        return;
    }

    tap_return_data = restored;

    LOG(LOG_LEVEL_INFO, "已恢复攻丝记录: state=%d R平面=%.3f 螺距=%.4f S=%d 攻进方向=%d",
        (int)tap_return_data.state, tap_return_data.tap_start_z,
        tap_return_data.screw_offset, tap_return_data.spindle_speed,
        tap_return_data.reverse);
}

void tapreturn_clear_ram_for_test(void)
{
    memset(&tap_return_data, 0, sizeof(tap_return_data));
}


void tapreturn_data_reset(void)
{
    memset(&tap_return_data, 0, sizeof(tap_return_data));
    tapreturn_data_persist();
}

static bool in_hole_by_feedback(double fb_z, double r_plane_z)
{
    const double tol=rt_data->axis_data[AXIS_Z].pos_tol;
    return fb_z < (r_plane_z-tol);
}




static bool block_has_no_type(const PLAN_BLOCK *block)
{
    return block->mode == PMC_MODE_M_CMD || block->mode == PMC_MODE_NULL || block->mode == PMC_MODE_PAUSE ;
}

/// 该块是不是刚性攻丝块（G33.1）。
static bool block_is_rigid_tap(const PLAN_BLOCK *block)
{
    if (block == NULL) return false;
    if (block_has_no_type(block)) return false;
    return block->type == RIGID_TAP;
}

/// 该块是不是本模块的回退段（G33.2，由 G30 P99 译码而来）。
static bool block_is_return_seg(const PLAN_BLOCK *block)
{
    if (block == NULL) return false;
    if (block_has_no_type(block)) return false;
    return block->type == TAP_RETURN;
}

bool tapreturn_record(const PLAN_BLOCK *block)
{
    if (!block_is_rigid_tap(block)) return false;

    // 新攻丝：覆盖记录（含上一次的历史数据）。

    tap_return_data.tap_start_z   = block->start_pos.z;   // R点平面
    tap_return_data.screw_offset  = block->rigid_data.screw_offset;
    tap_return_data.spindle_speed = block->rigid_data.speed;
    tap_return_data.reverse       = block->rigid_data.reverse;
    tap_return_data.state         = TAPRET_RECORDED;
    tapreturn_data_persist();
    return true;
}

void tapreturn_update(const PLAN_BLOCK *cur_block)
{
    // 参考点未建立 → 反馈位置不可信（上电/断电重启后坐标尚未建立）。
    if (!update_pos_flag) return;

    const double fb_z = rt_data->axis_data[AXIS_Z].fb_pos_no_comp;   // 实际反馈（未补偿机床坐标）
    const double r_z  = tap_return_data.tap_start_z;
    const bool in_hole = in_hole_by_feedback(fb_z, r_z);
    const TAP_RETURN_STATE state_before = tap_return_data.state;

    switch (tap_return_data.state)
    {
    case TAPRET_RECORDED:

        if (in_hole) tap_return_data.state = TAPRET_TAPPING;
        break;

    case TAPRET_TAPPING:
        if (!in_hole)
        {
            tap_return_data.state = TAPRET_DONE;
        }
        else if (block_is_return_seg(cur_block))
        {

            tap_return_data.state = TAPRET_RETURNING;
        }
        break;

    case TAPRET_RETURNING:
        if (!in_hole)
        {
            tap_return_data.state = TAPRET_DONE;
        }
        else if (!block_is_return_seg(cur_block))
        {

            // 记录保持可执行，再按一次回退键从当前位置继续沿同一螺旋退出来。
            tap_return_data.state = TAPRET_TAPPING;
        }
        break;

    case TAPRET_IDLE:

        break;

    case TAPRET_DONE:

        break;

    default:
        break;
    }
    if( tap_return_data.state!=state_before) tapreturn_data_persist();

}

TAP_RETURN_STATE tapreturn_state(void)
{
    return tap_return_data.state;
}

bool tapreturn_ready(void)
{
    // 只有"刀在孔里"的两个状态可回退
    if (tap_return_data.state != TAPRET_TAPPING &&
        tap_return_data.state != TAPRET_RETURNING)
    {
        return false;
    }

    if (!update_pos_flag) return false;

    double fb_z = rt_data->axis_data[AXIS_Z].fb_pos_no_comp;
    return in_hole_by_feedback(fb_z, tap_return_data.tap_start_z);
}

bool tapreturn_get_param(double *r_plane_z, double *pitch, int *reverse)
{
    if (!tapreturn_ready()) return false;
    if (r_plane_z) *r_plane_z = tap_return_data.tap_start_z;
    if (pitch)     *pitch     = tap_return_data.screw_offset;
    if (reverse)   *reverse   = tap_return_data.reverse ? 0 : 1;
    return true;
}


static bool tapreturn_build(char *buf, int buf_len)
{
    return tapreturn_generate(buf, buf_len) == GNC_OPERATOR_SUCCESS;
}

void tapreturn_refresh_return_param(void)
{
    double r_plane_z = 0.0, pitch = 0.0;
    int reverse = 0;

    if (tapreturn_get_param(&r_plane_z, &pitch, &reverse))
    {
        rt_set_rigid_return_param(r_plane_z, pitch, reverse, 1);
    }
    else
    {
        rt_clear_rigid_return_param();
    }
}

#define TAPRET_PITCH_TOL 1e-4

bool tapreturn_page_pitch_matches(double pitch_mm)
{
    if (!tapreturn_ready()) return false;
    const double diff = pitch_mm - tap_return_data.screw_offset;
    if (!(diff > -TAPRET_PITCH_TOL && diff < TAPRET_PITCH_TOL))
    {
        LOG(LOG_LEVEL_WARN, "页面螺距 %.4f 与记录 %.4f 不符，拒绝回退（回退必须走记录的螺距）",
            pitch_mm, tap_return_data.screw_offset);
        return false;
    }
    return true;
}

bool tapreturn_page_direction_matches(int m_value)
{
    if (!tapreturn_ready()) return false;
    const int expect_m = (tap_return_data.reverse != 0) ? 3 : 4;
    if (m_value != expect_m)
    {
        LOG(LOG_LEVEL_WARN, "页面转向 M%d 与记录不符：本次攻进为 %s，回退应为 M%d，拒绝回退",
            m_value, (tap_return_data.reverse != 0) ? "M4(反转)" : "M3(正转)", expect_m);
        return false;
    }
    return true;
}

bool tapreturn_page_params_match(double pitch_mm, int m_value)
{
    // "两项都对"的便捷判据（两条各自会打日志；这里只是不让调用方写两遍）
    return tapreturn_page_pitch_matches(pitch_mm)
           && tapreturn_page_direction_matches(m_value);
}

void tapreturn_clear_alarms(void)
{
    // 号与级别必须与上报时一致，否则清不掉（报警按 (type, level, id) 记）
    delAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_NO_RECORD);
    delAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_PITCH);
    delAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_SPEED);
    delAlarm(ALARM_PS, ALARM_WARNING, TAPRET_ALARM_PAGE_DIR);
}
static int tapreturn_speed_ceiling(void)
{
    const double max_spdl = rt_data->axis_data[AXIS_SP].spdl_max_speed;
    if (max_spdl > 0.0) return (int)max_spdl;

    if (tap_return_data.spindle_speed <= 0)
    {
        LOG(LOG_LEVEL_WARN, "主轴最大转速未配置(PAR_SPDL_MAX_SPEED)，且当前没有攻丝记录：页面转速无上限可比");
    }
    else
    {
        LOG(LOG_LEVEL_WARN, "主轴最大转速未配置(PAR_SPDL_MAX_SPEED)，页面转速上限按攻丝转速 %d 计",
            tap_return_data.spindle_speed);
    }
    return tap_return_data.spindle_speed;
}

bool tapreturn_page_speed_ok(int page_speed)
{
    if (page_speed <= 0)
    {
        LOG(LOG_LEVEL_WARN, "页面转速 %d 非法（必须为正），拒绝回退", page_speed);
        return false;
    }
    const int ceiling = tapreturn_speed_ceiling();
    if (page_speed > ceiling)
    {
        // ceiling == 0 表示"没有可比的量"（参数没配 + 没记录），不是"高于 0"
        if (ceiling > 0) LOG(LOG_LEVEL_WARN, "页面转速 %d 高于主轴最大转速 %d，拒绝回退", page_speed, ceiling);
        else             LOG(LOG_LEVEL_WARN, "页面转速 %d 无法校验（没有上限可比），拒绝回退", page_speed);
        return false;
    }
    return true;
}

const SYS_PROG_DESC tapreturn_sysprog_desc = {
    .id      = SYS_PROG_RIGID_RETURN,
    .name    = "攻丝回退",
    .guard   = tapreturn_ready,
    .build   = tapreturn_build,
    .prepare = tapreturn_refresh_return_param,   // 由 sysprog_arm() 保证早于内容被取用
    .cleanup = rt_clear_rigid_return_param,

};

static GNC_RES tapreturn_build_line(int speed, char *buf, int buf_len)
{
    if (buf == NULL || buf_len <= 0) return GNC_OPERATOR_ERROR;
    if (tap_return_data.screw_offset <= 0.0 ||        // 螺距必须为正
        speed <= 0 ||                                 // **生效**转速必须为正
        tap_return_data.tap_start_z <= -9000.0 ||     // 起点Z异常（未有效记录）
        tap_return_data.tap_start_z >= 9000.0)
    {
        LOG(LOG_LEVEL_WARN, "攻丝回退参数异常: 螺距%.4f 转速%d 起点Z%.4f，拒绝回退",
            tap_return_data.screw_offset, speed, tap_return_data.tap_start_z);
        return GNC_OPERATOR_ERROR;
    }

    int n = snprintf(buf, buf_len, "G90 M29 S%d G30 P99", speed);
    if (n <= 0 || n >= buf_len) return GNC_OPERATOR_ERROR;
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES tapreturn_generate(char *buf, int buf_len)
{
    if (!tapreturn_ready()) return GNC_OPERATOR_ERROR;

    return tapreturn_build_line(tap_return_data.spindle_speed, buf, buf_len);
}

GNC_RES tapreturn_generate_with_speed(int speed, char *buf, int buf_len)
{
    if (!tapreturn_ready()) return GNC_OPERATOR_ERROR;
    // 转速由**页面请求**直接给（HMI 请求区）：合法性判据只有一份（tapreturn_page_speed_ok）
    // if (!tapreturn_page_speed_ok(speed)) return GNC_OPERATOR_ERROR;
    LOG(LOG_LEVEL_INFO, "本次回退转速用页面下发值 %d（攻丝转速 %d，螺距不变，同步比不变）",
        speed, tap_return_data.spindle_speed);
    return tapreturn_build_line(speed, buf, buf_len);
}

double cal_rigid_pos_sync_error(){
    double drill_axis_pos = rt_data->axis_data[AXIS_Z].motor_act_pos - rt_data->axis_data[AXIS_Z].motor_act_pos_pre;
    double sp_axis_pos = rt_data->axis_data[AXIS_SP].motor_act_pos - rt_data->axis_data[AXIS_SP].motor_act_pos_pre;
    return drill_axis_pos / tap_return_data.screw_offset - sp_axis_pos / 360;
}