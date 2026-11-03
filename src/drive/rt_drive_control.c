/* RT 侧驱动管理：反馈诊断、故障复位等待、模式与使能业务时序。
 *
 * 这里是"某个轴该不该出力"的唯一仲裁点：
 *   业务模块只能表达需求（alarm_spindle_request_torque），
 *   真正的 enable/disable 请求全部由 hal_axis_intent_poll() 每拍重算后提交。
 *
 * 它明确不做的三件事（越界会破坏分层）：
 *   - 不推 DS402 设备阶梯，那是 HAL 的事；
 *   - 不承担整次 NC 复位，那是 reset.c 的事；
 *   - 不给适配层添加策略，适配层只负责搬运。
 */
#include "rt_drive_control.h"
#include "global.h"
#include "rt_data.h"
#include "rt_hal_adapter.h"
#include "reset.h"
#include "rt_hal_config.h"
#include "emcmot.h"
#include "../common/debug_mode.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include <time.h>
#include <math.h>

/* 下面这组 include 与文件开头重复（靠 include guard 兜住）。
 * 整理时请以文件开头那组为准，这里的副本不要单独维护。 */
#include "reset.h"
#include "rt_hal_config.h"
#include <alarmapi.h>
#include <hmisysapi.h>
#include "mpg.h"
#include <time.h>
#include"plc_g_signal_scanner.h"

/* HAL 错误码 → 报警号。RT 直接把 HAL 的返回值映射成报警号上报。
 * 每个号都必须在 HMI 的报警文本表里有对应条目，否则界面只会显示未知报警。
 * 120..130 与 HAL 公共返回码一一对应；131 专给复位超时；139 兜底未归类错误。 */
#define HAL_ALARM_BUS            120  /* 总线错误（含闭锁） */
#define HAL_ALARM_TIMEOUT        121  /* 启动预算耗尽或周期超时 */
#define HAL_ALARM_STOPPED        122  /* 已请求停止 */
#define HAL_ALARM_NOT_RUNNING    123  /* 尚未就绪 */
#define HAL_ALARM_STATE          124  /* 调用顺序错误 */
#define HAL_ALARM_CONFIG         125  /* 配置无效 */
#define HAL_ALARM_ABI            126  /* ABI 不匹配 */
#define HAL_ALARM_ARGUMENT       127  /* 参数无效 */
#define HAL_ALARM_MEMORY         128  /* 内存分配失败 */
#define HAL_ALARM_UNSUPPORTED    129  /* 设备不支持该功能 */
#define HAL_ALARM_BUSY           130  /* 主站已被占用 */
#define HAL_ALARM_RESET_TIMEOUT  131  /* 故障复位超时 */
#define HAL_ALARM_UNKNOWN        139  /* 未归类的 HAL 错误 */

/* 轴持续未使能不算驱动故障，只作诊断。
 * 启动和故障复位之后的 DS402 使能过渡期间本来就会短暂未使能，
 * 这个拍数阈值就是用来滤掉这种正常过渡的。 */
#define AXIS_NOT_ENABLED_LOG_CYCLES 5000u

static int hal_code_to_alarm(int32_t rc) {
    switch (rc) {
    case HAL_ERROR_BUS:         return HAL_ALARM_BUS;
    case HAL_ERROR_TIMEOUT:     return HAL_ALARM_TIMEOUT;
    case HAL_ERROR_STOPPED:     return HAL_ALARM_STOPPED;
    case HAL_ERROR_NOT_RUNNING: return HAL_ALARM_NOT_RUNNING;
    case HAL_ERROR_STATE:       return HAL_ALARM_STATE;
    case HAL_ERROR_CONFIG:      return HAL_ALARM_CONFIG;
    case HAL_ERROR_ABI:         return HAL_ALARM_ABI;
    case HAL_ERROR_ARGUMENT:    return HAL_ALARM_ARGUMENT;
    case HAL_ERROR_MEMORY:      return HAL_ALARM_MEMORY;
    case HAL_ERROR_UNSUPPORTED: return HAL_ALARM_UNSUPPORTED;
    case HAL_ERROR_BUSY:        return HAL_ALARM_BUSY;
    default:                    return HAL_ALARM_UNKNOWN;
    }
}

/* 把 HAL 错误码（正数）报成 RT 报警。
 *
 * 注意不能沿用旧 RT 的 `rc < 0` 判断——HAL 的错误码是正数。
 * 本函数公开给 PDO 线程直接调用：周期循环一退出，运动线程就卡在 sem_wait 上，
 * alarm_poll_hal 不再运行，只能在发现错误的地方就地把它报出去。 */
void alarm_report_hal_error(int32_t rc) {
    if (rc == HAL_OK) return;
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    const int id = hal_code_to_alarm(rc);
    LOG(LOG_LEVEL_ERR, "HAL 错误 rc=%d（%s）-> 报警 %d", rc, text, id);
    setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, id);
}

/* 是否有一笔故障复位正在等收口。
 * 由 rt_drive_request_fault_reset() 置起，收口或超时后清掉。 */
static int s_reset_waiting;
static uint32_t s_reset_axis_mask;    /* 本次已请求复位的轴 */
static uint32_t s_reset_unknown_mask; /* 状态暂不可读，仍需重新判定 */
static bool s_reset_request_failed;
/* 复位未确认成功时的使能锁定：为 1 期间禁止重新上力矩，
 * 只有下一次显式复位（alarm_reset_latch_clear）才解除。 */
static int s_reset_failed;
static struct timespec s_reset_started;
/* 唯一的正常业务力矩需求。业务侧只改这里；
 * rt_data 里的 spindle_wants_torque 是只读投影，不参与任何决策。 */
static bool s_spindle_torque_demand;
/* 业务侧出现过"撤力矩→再要力矩"的边沿。只靠每拍比较 intent 会漏掉同一拍内的
 * 翻转（停车收口撤力矩、紧接着 M19 又要力矩），导致使能管理停在 COMPLETE 而
 * 硬件其实已经下使能。该标志强制下一次 poll 重新走一次完整序列。 */
static bool s_spindle_torque_rearm;

bool alarm_spindle_request_torque(bool needed)
{

    /*===1. 撤销无条件生效：任何一处想撤，都能立刻撤下来===*/
    if (!needed) {
        s_spindle_torque_demand = false;
        return true;
    }

    /*===2. 授予要过闸：复位状态、急停、轴屏蔽、轴故障，缺一不可===*/
    if (!reset_spindle_business_allowed() || s_reset_waiting || s_reset_failed ||
        rt_data->sys_state[SYS_ESTOP_INPUT] || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] ||
        rt_data->axis_data[AXIS_SP].status.is_disable ||
        rt_data->axis_data[AXIS_SP].status.is_error) return false;
    /* 真正授予时记下"撤力矩→再要力矩"边沿，防止使能管理漏看同一拍内的翻转。 */
    if (!s_spindle_torque_demand) s_spindle_torque_rearm = true;
    s_spindle_torque_demand = true;
    return true;
}

bool alarm_fault_reset_pending(void) {
    return s_reset_waiting || s_reset_failed;
}

/* 任何一次新的复位动作都会调用：解除上一次"故障复位未收口"造成的使能锁定。
 * 没有这一步，收口失败一次就只能重启软件才能恢复。 */
static void reset_enable_business(void);
void alarm_reset_latch_clear(void) {
    (void)alarm_spindle_request_torque(false);
    reset_enable_business();
    if (!s_reset_failed) return;
    s_reset_failed = 0;
    LOG(LOG_LEVEL_INFO, "新的复位动作：解除上次故障复位失败的使能锁定，允许重试");
}

/* 复位收口失败时把每个轴的真实状态打全。
 * 目的是区分两种完全不同的失败：请求根本没被受理，还是受理了但驱动器仍停在 Fault。 */
static void alarm_log_reset_axes(const char* why) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        int32_t rst = -1;
        RtHalAxisStatus st;
        const int rst_ok = rt_hal_fault_reset_state((RtHalAxisSlot)i, &rst) == HAL_OK;
        const int st_ok = rt_hal_axis_status((RtHalAxisSlot)i, &st) == HAL_OK;
        LOG(LOG_LEVEL_ERR, "%s：轴槽 %d reset=%d%s enabled=%d status=0x%04X 603F=0x%08X%s%s",
            why, i, (int)rst, rst_ok ? "" : "(读失败)",
            st_ok ? (int)st.enabled : -1,
            st_ok ? (unsigned)st.raw_status : 0u,
            st_ok ? (unsigned)st.error_code : 0u,
            (st_ok && st.error_code_valid) ? "" : "(无故障码)",
            st_ok ? "" : "[状态读不到]");
    }
}

/* 四轴反馈是否都已干净：无 Fault 位、无故障码。
 * 只用于复位收口之后的提示，不再作为收口条件（原因见 alarm_poll_hal 里的说明）。 */
static int drive_feedback_clear(void) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        RtHalAxisStatus st;
        if (rt_hal_axis_status((RtHalAxisSlot)i, &st) != HAL_OK ||
            (st.raw_status & 0x0008) ||
            (st.error_code_valid && st.error_code != 0)) return 0;
    }
    return 1;
}

/* 每轴使能状态机：准备模式 → 显式使能 → 观察结果。
 *
 * 这里只做业务时序，不推 DS402 阶梯、不记硬件超时、也不重试被拒的请求。
 * ENABLE_FAILED 是粘滞态：意图仍然是"使能"也不会自动重来，必须等一次复位、
 * 一次新的主轴模式业务，或一次下使能/上使能的意图翻转。 */
enum { ENABLE_IDLE, ENABLE_MODE_NEXT, ENABLE_MODE_WAIT, ENABLE_WAIT, ENABLE_COMPLETE, ENABLE_FAILED };
static int s_enable_stage[RT_HAL_AXIS_COUNT];
static int s_enable_intent[RT_HAL_AXIS_COUNT]={-1,-1,-1,-1};
static HalAxisMode s_enable_mode[RT_HAL_AXIS_COUNT];
/* ENABLE_WAIT 连续无进展拍数。HAL 每轴只保留"最新一条"拒绝，同一拍里后续的
 * 定向/位置请求会把 ENABLE 的拒绝记录覆盖掉，使能管理就永远等不到结果。
 * 连续这么多拍既没有使能动作、也没有 ENABLE 拒绝时，重新提交一次。 */
#define ENABLE_WAIT_RETRY_CYCLES 10U
static unsigned s_enable_wait_cycles[RT_HAL_AXIS_COUNT];
/* 该轴曾因报警/急停被强制下使能：重新上使能前必须先把命令位置基准
 * 对齐到实际位置，否则会用下使能前的旧命令位置把轴“拉回”去，
 * 表现为复位/松开急停时抖一下。 */
static uint8_t s_pos_realign[RT_HAL_AXIS_COUNT];
/* 进入粘滞失败时留一条日志；否则现场只能看到"轴一直不使能"，不知道卡在哪一步。 */
static void enable_mark_failed(int i, const char* why, int code) {
    if(s_enable_stage[i]!=ENABLE_FAILED)
        LOG(LOG_LEVEL_ERR,"使能失败：轴槽 %d 阶段=%d 原因=%s code=%d",
            i,s_enable_stage[i],why!=NULL?why:"未说明",code);
    s_enable_stage[i]=ENABLE_FAILED;
    s_enable_wait_cycles[i]=0;
}
#ifdef UNIT_TEST
/* 测试入口：把主轴使能管理强制放到"已提交、但拒绝记录被覆盖"的等待态，
 * 用于验证 ENABLE_WAIT 无进展自愈。 */
void rt_test_force_spindle_enable_wait(void) {
    /* 需求已授予、但使能请求没有生效：模拟"拒绝记录被其它类别覆盖"后的等待态。
     * 同时把需求边沿标志清掉，避免再次触发"重走序列"。 */
    s_spindle_torque_demand=true;
    s_spindle_torque_rearm=false;
    s_enable_intent[RT_HAL_AXIS_SP]=1;
    s_enable_stage[RT_HAL_AXIS_SP]=ENABLE_WAIT;
    s_enable_wait_cycles[RT_HAL_AXIS_SP]=0;
}
#endif
void alarm_axis_mode_requested(int slot,int mode) {
    if(slot<0||slot>=RT_HAL_AXIS_COUNT)return;
    s_enable_mode[slot]=mode;
    s_enable_stage[slot]=ENABLE_MODE_NEXT;
}
static void reset_enable_business(void) {
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        s_enable_intent[i]=-1;s_enable_stage[i]=ENABLE_IDLE;s_enable_wait_cycles[i]=0;
    }
}
void hal_axis_intent_poll(void) {

    /*===1. 急停闸门：按下急停、以及松开后尚未复位期间，都不许任何驱动器动作。
     * SYS_IS_ESTOP_ACTIVE 只有显式复位（handle_alarm_cmd 的 HMI_RESET 分支）才清除，
     * 所以松开急停后轴保持停止/下使能，重新使能完全交给复位流程。否则会在松开
     * 瞬间用急停前的命令位置重新上使能，表现为“抖一下”。===*/
    const int estop=rt_data->sys_state[SYS_ESTOP_INPUT] ||
                    rt_data->sys_state[SYS_IS_ESTOP_ACTIVE];
    if(estop) s_spindle_torque_demand=false;

    /*===2. 复位期允许量：受控停车过程中必须保留力矩才能减速===*/
    const bool spindle_wanted=reset_spindle_torque_allowed(s_spindle_torque_demand);

    /*===3. 把最终结果投影给 HMI（只读字段，不参与决策）===*/
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque = spindle_wanted &&
        !estop && !rt_data->sys_state[SYS_IS_ESTOP_ACTIVE] && !s_reset_waiting &&
        !s_reset_failed && !rt_data->axis_data[AXIS_SP].status.is_disable &&
        !rt_data->axis_data[AXIS_SP].status.is_error &&
        s_enable_stage[RT_HAL_AXIS_SP]!=ENABLE_FAILED;

    /*===4. 故障复位在途：本拍不发任何使能请求，只处理急停打断===*/
    if(estop&&s_reset_waiting)s_reset_waiting=0;
    if(s_reset_waiting)return;
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        RtHalAxisSlot slot=(RtHalAxisSlot)i;

        /*===5. 逐轴求三态意图：2 急停 / 0 去使能 / 1 使能===*/
        const int disabled=rt_data->axis_data[i].status.is_disable||s_reset_failed;
        const int off=disabled||(slot==RT_HAL_AXIS_SP&&
            !spindle_wanted);

        const int intent=estop?2:off?0:1;
        int changed=intent!=s_enable_intent[i];
        if(slot==RT_HAL_AXIS_SP&&s_spindle_torque_rearm) {
            /* 业务明确重新要力矩：即使 intent 没变，也重走一次使能序列。 */
            changed=1;
            s_spindle_torque_rearm=false;
        }
        s_enable_intent[i]=intent;

        /*===6. 急停：走 HAL 安全路径，不进入使能状态机===*/
        if(estop) {
            s_enable_stage[i]=ENABLE_IDLE;
            s_pos_realign[i]=1;          /* 急停下使能，恢复前要重新对齐位置 */
            (void)rt_hal_request_estop(slot);
            continue;
        }

        /*===7. 反馈报故障：下使能；只有真的 Fault 位才锁粘滞失败===*/
        if(rt_data->axis_data[i].status.is_error && !off) {
            /* 区分两种“报错”：
             *   a) 状态字 Fault 位仍置位 → 真故障，锁 ENABLE_FAILED，等显式复位；
             *   b) 驱动已回 READY、只剩 0x603F 残留码 → 只保持下使能，不锁失败。
             * 现场“要按两次复位才能上使能”就是 b) 被误当成 a)：故障复位
             * 刚收口那一拍 0x603F 还没清掉，使能状态机被锁死，只能再复位一次。 */
            RtHalAxisStatus st;
            const int unreadable=(rt_hal_axis_status(slot,&st)!=HAL_OK);
            const int real_fault=unreadable||((st.raw_status&0x0008U)!=0);
            if(s_enable_stage[i]!=ENABLE_FAILED)(void)rt_hal_request_axis_enable(slot,0);
            if(real_fault) {
                s_pos_realign[i]=1;          /* 故障强制下使能，恢复前要重新对齐位置 */
                enable_mark_failed(i,"轴反馈故障",0);
            } else {
                /* 残留故障码：保持下使能，等它清掉后自动重新走使能序列。 */
                s_enable_stage[i]=ENABLE_IDLE;
                s_enable_intent[i]=-1;       /* 清意图，故障码清掉后 changed 重新成立 */
            }
            continue;
        }

        /*===8. 关断分支：意图翻转、或复位要求收力矩时才下使能===*/
        if(off) {
            if(changed || (slot==RT_HAL_AXIS_SP && reset_spindle_disable_needed())) {
                const int rc=rt_hal_request_axis_enable(slot,0);
                if(slot==RT_HAL_AXIS_SP && reset_spindle_disable_needed())
                    reset_spindle_disable_submitted(rc);
            }
            /* 无力矩状态下可以先做模式准备：新一次的主轴模式业务从这里进入。 */
            if(changed)s_enable_stage[i]=ENABLE_IDLE;
            continue;
        }

        /*===8.5 完成态复核：COMPLETE 只是业务镜像。若 HAL 已经登记了停止约束，
         * 说明使能结论已过期（停车收口在 COMPLETE 之后才生效），必须重新走序列；
         * 否则 stop_goal 会一直留着，轴永远停在 READY。===*/
        if(slot==RT_HAL_AXIS_SP&&s_enable_stage[i]==ENABLE_COMPLETE&&
           reset_spindle_business_allowed()) {
            HalAxisSnapshot chk;
            if(rt_hal_axis_snapshot(slot,&chk)==HAL_OK &&
               (chk.motion_block_reasons&HAL_BLOCK_STOP_REQUIRED)) {
                s_enable_stage[i]=ENABLE_MODE_WAIT;
            }
        }

        /*===9. 使能第一步：先把模式准备好（进给轴固定 CSP，主轴等业务指定）===*/
        if(changed && s_enable_stage[i]!=ENABLE_MODE_NEXT && s_enable_stage[i]!=ENABLE_MODE_WAIT) {
            if(slot!=RT_HAL_AXIS_SP) {
                s_enable_mode[i]=HAL_AXIS_MODE_CSP;
                if(rt_hal_request_axis_mode(slot,HAL_AXIS_MODE_CSP)==HAL_OK)
                    s_enable_stage[i]=ENABLE_MODE_NEXT;
                else
                    enable_mark_failed(i,"CSP模式请求被拒",HAL_ERROR_BUSY);
            } else {
                /* 主轴要准备哪个模式由业务决定（CSV 运转或 CSP 定向），这里只等它给出。 */
                s_enable_mode[i]=HAL_AXIS_MODE_UNSET;
                s_enable_stage[i]=ENABLE_MODE_WAIT;
            }
        }
        if(s_enable_stage[i]==ENABLE_MODE_NEXT) {
            s_enable_stage[i]=ENABLE_MODE_WAIT;
            continue; /* 本拍提交出去的模式请求，要等下一拍屏障之后才被真正消费，本拍不重复申请 */
        }

        /*===10. 读一次快照：后面两个等待阶段都靠它判别进展===*/
        HalAxisSnapshot axis;
        if(rt_hal_axis_snapshot(slot,&axis)!=HAL_OK) {
            enable_mark_failed(i,"轴快照不可读",0);continue;
        }

        /*===11. 等待阶段：模式就绪后才发上使能，再确认实际使能到位才算完成===*/
        if(s_enable_stage[i]==ENABLE_MODE_WAIT) {
            if(s_enable_mode[i]==HAL_AXIS_MODE_UNSET)s_enable_mode[i]=axis.target_mode;
            if(axis.mode_setting_state==HAL_MODE_SETTING_FAILED ||
               (axis.request_rejection.category==HAL_ISSUE_MODE&&axis.request_rejection.code)) {
                enable_mark_failed(i,"模式准备失败",(int)axis.request_rejection.code);continue;
            }
            if(axis.target_mode==HAL_AXIS_MODE_UNSET||axis.target_mode!=s_enable_mode[i]||
               axis.mode_setting_state!=HAL_MODE_SETTING_READY)continue;
            /* 同模式定向退出期间已经有有效使能，不再提交会被动作占用拒绝的重复使能。 */
            if((axis.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE))==
                   (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE) &&
               axis.drive_state==HAL_DRIVE_OPERATION_ENABLED && axis.actual_mode==s_enable_mode[i] &&
               !(axis.motion_block_reasons&HAL_BLOCK_NOT_ENABLED)) {
                s_enable_stage[i]=ENABLE_COMPLETE;continue;
            }
            /* 真正提交使能字之前才做位置对齐（之前放在序列启动那拍太早）。
             * 不用额外等拍：HAL 的 DS402 使能阶梯本身要经过
             * SWITCH_ON → ENABLE_OP 好几拍，而且每一拍都会先把 0x607A
             * 预置成当前实际位置再发控制字，目标天然先于使能生效。 */
            if(s_pos_realign[i] && slot!=RT_HAL_AXIS_SP) {
                const double cmd=rt_data->axis_data[i].motor_cmd_pos;
                const double act=rt_data->axis_data[i].motor_act_pos;
                emcmot_rebuild_axis_position_from_feedback(i);
                s_pos_realign[i]=0;
                LOG(LOG_LEVEL_INFO,
                    "轴槽 %d 上使能前对齐命令位置：cmd=%.4f act=%.4f 偏差=%.4f",
                    i,cmd,act,cmd-act);
            }
            if(rt_hal_request_axis_enable(slot,1)==HAL_OK) {
                s_enable_stage[i]=ENABLE_WAIT;s_enable_wait_cycles[i]=0;
            } else enable_mark_failed(i,"使能请求提交失败",0);
        } else if(s_enable_stage[i]==ENABLE_WAIT) {
            if(axis.request_rejection.category==HAL_ISSUE_ENABLE&&axis.request_rejection.code) {
                /* BUSY = 轴上一笔停止/动作还没收口，属瞬态；等它收口后重新提交使能。
                 * 停车后紧接 M19/M3 时必然遇到，不能粘连成 ENABLE_FAILED。 */
                if(axis.request_rejection.code==HAL_ERROR_BUSY) {
                    s_enable_stage[i]=ENABLE_MODE_WAIT;s_enable_wait_cycles[i]=0;continue;
                }
                enable_mark_failed(i,"使能请求被拒绝",(int)axis.request_rejection.code);continue;
            }
            if((axis.feedback_valid&(HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE))==
                (HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE) &&
                axis.drive_state==HAL_DRIVE_OPERATION_ENABLED&&axis.actual_mode==s_enable_mode[i]) {
                s_enable_stage[i]=ENABLE_COMPLETE;s_enable_wait_cycles[i]=0;
            } else if(axis.control_state!=HAL_CONTROL_ENABLING &&
                axis.execution_error.category==HAL_ISSUE_ENABLE&&axis.execution_error.code) {
                enable_mark_failed(i,"使能执行错误",(int)axis.execution_error.code);
            } else if(axis.control_state==HAL_CONTROL_ENABLING) {
                /* 使能动作在途，正常等待。 */
                s_enable_wait_cycles[i]=0;
            } else if(++s_enable_wait_cycles[i]>=ENABLE_WAIT_RETRY_CYCLES) {
                /* 没有使能动作、没有 ENABLE 拒绝、也没有执行错误：请求多半被执行过但被
                 * 拒，而拒绝记录被同一拍后续的定向/位置请求类别覆盖。重新提交一次。 */
                s_enable_stage[i]=ENABLE_MODE_WAIT;s_enable_wait_cycles[i]=0;
            }
        }
        /* COMPLETE / FAILED 都不会凭一个一直存在的旧意图自动重新使能。
         * 恢复只有三条路：一次显式复位、一次新的主轴模式业务、或一次下使能/上使能
         * 的意图翻转；而且每条路都要重新过全部准入条件。
         * 单纯 Fault 消失不构成恢复授权。 */
    }
}

/* 恢复原版轴报警 30..33（X30 / Y31 / Z32 / SP33）。
 * 清库之后仍然存在的故障必须重新发布，不能只报一次边沿；
 * 屏蔽轴只抑制报警，不伪造健康反馈；读失败时保留上一次的故障状态。 */
static void alarm_poll_drive_errors(void) {
    static const char* const kAxisName[RT_HAL_AXIS_COUNT] = {"X", "Y", "Z", "SP"};
    static uint32_t s_last_code[RT_HAL_AXIS_COUNT] = {0, 0, 0, 0};
    static int s_last_fault[RT_HAL_AXIS_COUNT];

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        RtHalAxisStatus st;

        /*===1. 读本轴状态；读不到时沿用上一次的故障判断===*/
        const int readable = (rt_hal_axis_status((RtHalAxisSlot)i, &st) == HAL_OK);
        const int masked = rt_data->axis_data[i].status.is_disable ? 1 : 0;
        if (!readable) {
            if (!masked && rt_data->axis_data[i].status.is_error)
                setAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
            continue;
        }

        /*===2. 判故障：状态字 Fault 位或 0x603F 非零都算===*/
        const int has_err = (st.raw_status & 0x0008) ||
                            (st.error_code_valid && st.error_code != 0);
        const uint32_t code = st.error_code_valid ? st.error_code : 0;

        rt_data->axis_data[i].status.is_error = has_err ? true : false;
        if (has_err && !masked)
            setAlarm(ALARM_AX, ALARM_ERROR, 30 + i);
        else
            delAlarm(ALARM_AX, ALARM_ERROR, 30 + i);

        /*===3. 只在故障码或故障态变化时打日志，避免每拍刷屏===*/
        if (code != s_last_code[i] || has_err != s_last_fault[i]) {
            s_last_code[i] = code;
            s_last_fault[i] = has_err;
            if (has_err)
                LOG(LOG_LEVEL_ERR, "[HAL] %s 轴驱动器报错：status_word=0x%04X 0x603F=0x%08X valid=%d",
                    kAxisName[i], (unsigned)st.raw_status, (unsigned)code, (int)st.error_code_valid);
            else
                LOG(LOG_LEVEL_INFO, "[HAL] %s 轴驱动器报警已清除", kAxisName[i]);
        }
    }
}

/* 轴持续未使能只作诊断，不升级成报警，也不触发复位循环。
 * 连续 AXIS_NOT_ENABLED_LOG_CYCLES 拍都不满足期望时，才打一条 WARN。 */
static void alarm_poll_axis_not_enabled(void) {
    static uint32_t s_age[RT_HAL_AXIS_COUNT];
    static int s_logged[RT_HAL_AXIS_COUNT];

    /*===1. 静默期：急停、切模式、周期错误、复位在途，一律不做诊断===*/
    const int quiescent = !!(rt_data->sys_state[SYS_ESTOP_INPUT] ||
                             rt_data->sys_state[SYS_IS_SWITCHING_MODE] ||
                             rt_hal_cycle_error() != 0 ||
                             alarm_fault_reset_pending());
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        RtHalAxisStatus st;
        const int readable = rt_hal_axis_status((RtHalAxisSlot)i, &st) == HAL_OK;

        /*===2. 只有"本该使能却没使能"的轴，才算异常===*/
        const int expected = !quiescent &&
            !rt_data->axis_data[i].status.is_disable &&
            !rt_data->axis_data[i].status.is_error &&
            (!rt_hal_config_is_spindle((RtHalAxisSlot)i) ||
             reset_spindle_torque_allowed(s_spindle_torque_demand));
        if (!expected || !readable || st.enabled) {
            s_age[i] = 0;
            s_logged[i] = 0;
            continue;
        }

        /*===3. 连续超过阈值才打一条 WARN；打过就不再重复===*/
        if (s_logged[i] || ++s_age[i] < AXIS_NOT_ENABLED_LOG_CYCLES) continue;
        s_logged[i] = 1;
        int32_t reset_state = -1;
        (void)rt_hal_fault_reset_state((RtHalAxisSlot)i, &reset_state);
        HalAxisSnapshot snap;
        const int snap_ok = rt_hal_axis_snapshot((RtHalAxisSlot)i, &snap) == HAL_OK;
        LOG(LOG_LEVEL_WARN,
            "HAL 轴槽 %d 持续未使能：status_word=0x%04X 0x603F=0x%08X valid=%d reset=%d stage=%d intent=%d ctl=%d mode_set=%d act/tgt=%d/%d block=0x%x",
            i, (unsigned)st.raw_status, (unsigned)st.error_code,
            (int)st.error_code_valid, (int)reset_state,
            s_enable_stage[i], s_enable_intent[i],
            snap_ok ? (int)snap.control_state : -1,
            snap_ok ? (int)snap.mode_setting_state : -1,
            snap_ok ? (int)snap.actual_mode : -1,
            snap_ok ? (int)snap.target_mode : -1,
            snap_ok ? (unsigned)snap.motion_block_reasons : 0u);
    }
}

/* 轴在“已使能”状态下意外退出 Operation Enabled：HAL 记为
 * AXIS_CONDITION / NOT_ENABLED（典型是硬件急停直接切 STO/快停，驱动器先掉使能）。
 * 这类事件必须锁成“需复位”：否则松开急停后可能直接重新上使能 → 飞车风险。
 * 报警用现有的 ETHCAT_SV 132（轴未使能或运动许可未就绪）。 */
static void alarm_poll_oe_loss(void) {
    static uint8_t s_reported[RT_HAL_AXIS_COUNT];
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        HalAxisSnapshot a;
        if(rt_hal_axis_snapshot((RtHalAxisSlot)i,&a)!=HAL_OK) continue;
        const int oe_lost = (a.execution_error.category==HAL_ISSUE_AXIS_CONDITION &&
                             a.execution_error.code==HAL_ERROR_NOT_ENABLED);
        if(!oe_lost) { s_reported[i]=0; continue; }
        if(s_reported[i]) continue;
        s_reported[i]=1;
        s_enable_stage[i]=ENABLE_FAILED;   /* 粘滞：只有显式复位能解除 */
        s_pos_realign[i]=1;
        setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, 132);
        LOG(LOG_LEVEL_ERR,
            "轴槽 %d 意外退出使能（sw=0x%04X 0x603F=0x%08X）：已锁为需复位，松开急停不会自动上使能",
            i,(unsigned)a.raw_status,(unsigned)a.drive_error_code);
    }
}

/* 每拍检查 HAL 侧状态并推进复位等待。由运动线程调用：
 * 只读适配层快照，不直接碰 HalContext。 */
void alarm_poll_hal() {
    /*===1. 周期循环异常退出：主站已经跑不动了===*/
    const int32_t ce = rt_hal_cycle_error();
    if (ce) alarm_report_hal_error(ce);

    /*===2. 驱动器故障码（0x603F）→ is_error===*/
    alarm_poll_drive_errors();

    /*===2.5 轴意外退出使能（OE 丢失）：锁成需复位并报警===*/
    alarm_poll_oe_loss();

    /*===3. 持续未使能只做诊断，不把正常使能过渡当报警===*/
    alarm_poll_axis_not_enabled();

    /*===4. 只等本次有故障的轴；状态不可读的轴恢复后重新判定===*/
    if (!s_reset_waiting) return;
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const uint32_t bit = 1u << i;
        RtHalAxisStatus st;
        if (!(s_reset_unknown_mask & bit) || rt_hal_axis_status((RtHalAxisSlot)i, &st) != HAL_OK)
            continue;
        s_reset_unknown_mask &= ~bit;
        if ((st.raw_status & 0x0008U) || (st.error_code_valid && st.error_code != 0)) {
            const int rc = rt_hal_request_fault_reset((RtHalAxisSlot)i);
            if (rc == HAL_OK) s_reset_axis_mask |= bit;
            else { s_reset_request_failed = true; alarm_report_hal_error(rc); }
        }
    }
    if (s_reset_request_failed) {
        s_reset_waiting = 0;
        s_reset_failed = 1;
        alarm_log_reset_axes("故障复位请求被拒绝");
        return;
    }
    int timed_out = 0;
    const int settled = !s_reset_unknown_mask && rt_hal_fault_reset_settled_mask(s_reset_axis_mask, &timed_out);
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    /* 外层比 HAL 多留 1 秒，仿真调试跟随放宽，正式运行仍为 6 秒。
     * 还要覆盖三种拖尾——
     * 请求没被受理、快照缺失、以及驱动器残留 0x603F 的情况。 */
#ifdef DEBUG_MODE_ENABLED
    const double reset_budget = (double)DEBUG_HAL_TIMEOUT_MS / 1000.0 + 1.0;
#else
    const double reset_budget = 6.0;
#endif
    const double elapsed = (double)(now.tv_sec - s_reset_started.tv_sec) +
                           (double)(now.tv_nsec - s_reset_started.tv_nsec) / 1e9;
    /* 收口条件只认 HAL 的复位终态：原先还要求"四轴反馈全干净"，任一轴残留 0x603F
     * 就超时并把整机锁成未使能；残留故障由 alarm_poll_drive_errors() 按轴报 30..33。 */
    if (!settled) {
        if (elapsed < reset_budget) return;
        timed_out = 1;
    }

    /*===5. 收口：超时则锁定并使能需求失效，正常则恢复正常使能意图管理===*/
    s_reset_waiting = 0;
    if (timed_out) {
        s_reset_failed = 1;
        alarm_log_reset_axes("故障复位未收口");
        /* 边沿发了、驱动器没在配置时间内离开 Fault：与"没复位"是两回事，单独报号。 */
        LOG(LOG_LEVEL_ERR, "故障复位超时：复位未结束或快照不可读");
        setAlarm(ALARM_ETHCAT_SV, ALARM_ERROR, HAL_ALARM_RESET_TIMEOUT);
    } else {
        if (!drive_feedback_clear())
            LOG(LOG_LEVEL_WARN, "故障复位已收口，但仍有轴反馈未清（按轴报警 30..33 处理，不锁整机）");
        LOG(LOG_LEVEL_INFO, "故障复位完成：已请求的轴均已收口，恢复轴使能意图管理");
    }
}

/* 报警事件驱动的故障复位入口：保留原有处理顺序，
 * 这里只接管"发请求"和"记录等待"两件事。 */
void rt_drive_request_fault_reset(void)
{

    /*===1. 重新计时，并解除上一次的失败锁定===*/
    clock_gettime(CLOCK_MONOTONIC, &s_reset_started);
    s_reset_failed = 0;
    s_reset_axis_mask = s_reset_unknown_mask = 0;
    s_reset_request_failed = false;

    /*===2. 清业务力矩需求与使能状态机，避免旧意图在复位后被沿用===*/
    (void)alarm_spindle_request_torque(false);
    reset_enable_business(); /* 一次新的显式 RT 恢复（含 D66 自动复位）：清空使能状态机与力矩需求 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        /* 没绑定 HAL 轴的槽位不能发请求：适配层不会取走它，收口判定永远不成立。 */
        if (rt_hal_config_hal_axis((RtHalAxisSlot)i) < 0) continue;

        /* 清软件执行错误与驱动器故障复位是两件事。 */
        alarm_report_hal_error(rt_hal_clear_execution_error((RtHalAxisSlot)i));
        RtHalAxisStatus st;
        if (rt_hal_axis_status((RtHalAxisSlot)i, &st) != HAL_OK) {
            s_reset_unknown_mask |= 1u << i;
            continue;
        }
        if ((st.raw_status & 0x0008U) || (st.error_code_valid && st.error_code != 0)) {
            const int rc = rt_hal_request_fault_reset((RtHalAxisSlot)i);
            if (rc == HAL_OK) s_reset_axis_mask |= 1u << i;
            else { s_reset_request_failed = true; alarm_report_hal_error(rc); }
        }
    }

    /*===4. 进入等待收口，并立刻刷一次驱动故障===*/
    s_reset_waiting = s_reset_axis_mask || s_reset_unknown_mask || s_reset_request_failed;
    alarm_poll_drive_errors();
}
