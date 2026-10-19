/* 适配层实现；线程契约见 rt_hal_adapter.h 头部。 */

/* 严格 ISO 模式下不声明 clock_gettime/sem_timedwait，必须显式要，且必须在 include 前 */
#define _POSIX_C_SOURCE 200809L

#include "rt_hal_adapter.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "hal_c_api.h"   /* create/start/stop/destroy、hal_error_text */
#include "rt_sem.h"      /* 信号量、thread_exit_flag */

/* ---- context 与运行标志 ---- */
static HalContext* s_ctx;                 /* 唯一句柄；只有 PDO 线程碰（create/start/stop 例外） */
static int         s_running;             /* 1 = 已 start 且未 shutdown */
static uint32_t    s_cycle_timeout_ms;    /* 周期超时：等两个手的等待上限 */

/* ---- 快照：PDO 线程 begin 后填一次，本拍内不再改 ---- */
static HalCAxisStatus   s_snap_status[RT_HAL_AXIS_COUNT];    /* 轴状态 */
static HalCSpindleStatus s_snap_spindle[RT_HAL_AXIS_COUNT];  /* 转速侧；非主轴槽无意义 */
static int              s_snap_valid;                        /* 0 = 本拍没采过，查询接口要拒 */

/* ---- 请求：其他线程写，PDO 线程在 commit 前取走 ---- */
static double s_req_pos[RT_HAL_AXIS_COUNT];        /* 目标位置（用户单位） */
static double s_req_setpos[RT_HAL_AXIS_COUNT];     /* 当前位置重定义为该值（设零/对刀） */
static int    s_req_setpos_dirty[RT_HAL_AXIS_COUNT];
static double s_req_turn[RT_HAL_AXIS_COUNT];       /* 主轴单圈刻度（度）；同拍优先于 pos */
static int    s_req_pos_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_turn_dirty[RT_HAL_AXIS_COUNT];
static double s_req_speed[RT_HAL_AXIS_COUNT];      /* 目标转速 rpm */
static int    s_req_speed_dir[RT_HAL_AXIS_COUNT];  /* 转向单独传：rpm 不许为负 */
static int    s_req_speed_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_estop[RT_HAL_AXIS_COUNT];      /* 1 = 请求急停。一次性事件 */
static int    s_req_reset[RT_HAL_AXIS_COUNT];      /* 1 = 请求故障复位。一次性事件 */
static int     s_req_enable[RT_HAL_AXIS_COUNT];          /* 期望使能：1 = 要力矩，0 = 去使能；按拍声明 */
static int     s_req_enable_dirty[RT_HAL_AXIS_COUNT];    /* 只有值变了才下发；失败保留，下拍重试 */
/* 每轴每类请求上次的失败码；只在失败码变化时报一次，避免逐拍刷屏 */
enum { RT_REQ_POS = 0, RT_REQ_SPEED, RT_REQ_MODE, RT_REQ_ENABLE, RT_REQ_ESTOP, RT_REQ_RESET,
       RT_REQ_ORIENT, RT_REQ_SETPOS, RT_REQ_KINDS };
static int32_t s_req_fail_rc[RT_HAL_AXIS_COUNT][RT_REQ_KINDS];
/* 预期内失败已连着失败的拍数；到点才报一行，见 report_req() */
static uint32_t s_req_fail_age[RT_HAL_AXIS_COUNT][RT_REQ_KINDS];
static int32_t s_snap_reset[RT_HAL_AXIS_COUNT];   /* 复位进展快照 */
static int32_t s_snap_orient[RT_HAL_AXIS_COUNT];  /* 定向状态快照 */
static uint8_t s_snap_fresh[RT_HAL_AXIS_COUNT];   /* 该轴本拍反馈读取成功 */
static uint8_t s_snap_spindle_fresh[RT_HAL_AXIS_COUNT];
static int32_t s_cycle_error;                     /* 周期循环异常退出的原因；0 = 无 */
static HalSpindleMode s_req_mode[RT_HAL_AXIS_COUNT];      /* 期望的 CSV / CSP */
static int    s_req_mode_dirty[RT_HAL_AXIS_COUNT];        /* 只有变了才下发 */
/* 定向请求/释放：运动线程写，PDO 线程消费，一次性，消费即清 */
static int    s_req_orient[RT_HAL_AXIS_COUNT];       /* 1 = 请求定向 */
static int    s_req_orient_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_orient_rel[RT_HAL_AXIS_COUNT];   /* 1 = 释放定向 */
static int    s_req_orient_rel_dirty[RT_HAL_AXIS_COUNT];
/* 模式切换握手上限：连续这么多拍驱动器实际模式（0x6061）还没变成要的，报一次 */
#define RT_HAL_MODE_WAIT_CYCLES 1000u
/* "轴尚未使能"等多久才值得报：DS402 要几拍才走到使能 */
#define RT_HAL_NOT_ENABLED_WAIT_CYCLES 1000u
static uint32_t s_mode_wait[RT_HAL_AXIS_COUNT];      /* 本次请求已等拍数；请求变了清零 */
static int      s_mode_warned[RT_HAL_AXIS_COUNT];    /* 超上限报过一次就不再刷 */
static uint8_t  s_mode_requested[RT_HAL_AXIS_COUNT]; /* 发过模式请求没；去使能时清掉 */

/* IO 映像，长度与 REG_X_NUM/REG_Y_NUM 一致 */
static uint8_t s_x_image[RT_HAL_IO_IMAGE_MAX];   /* 输入映像 */
static uint8_t s_y_image[RT_HAL_IO_IMAGE_MAX];   /* 输出映像 */
static int     s_io_ready;                 /* 1 = 本拍输入映像取到了；取不到连输出也不发 */
static uint32_t s_x_size, s_y_size;        /* 映像段长（字节），start 时向 HAL 问一次并缓存 */
static int     s_in_cycle;                 /* 已 begin 未 commit：停机帧只能在这个窗口发 */

/* 停机要下发的 Y；默认未配置——不能假设全零安全，宁可不写 */
static uint8_t s_stop_image[RT_HAL_IO_IMAGE_MAX];   /* 停机时要写出去的 Y 映像 */
static int     s_stop_image_valid;            /* 0 = 没配过，就什么都不写 */
static int slot_ok(int slot) {
    return slot >= 0 && slot < (int)RT_HAL_AXIS_COUNT;
}

/* 台账挡住运动请求时提示一次（被挡的请求软件侧照跑、只有驱动器收不到）；
 * 台账清掉后重新允许提示 */
static int s_ledger_warned;
static void warn_ledger_blocked(const char* what, int slot) {
    if (rt_hal_config_motion_allowed()) { s_ledger_warned = 0; return; }
    if (s_ledger_warned) return;
    s_ledger_warned = 1;
    fprintf(stderr, "[HAL] 运动台账未清，轴槽%d 的 %s 被丢弃"
                    "（软件侧看起来正常，但驱动器收不到）\n", slot, what);
    fflush(stderr);
}

/* HAL 错误码是正数；带上轴槽，否则四轴同码看起来只报了一次。 */
static void log_hs(const char* what, int slot, int32_t rc) {
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    if (slot >= 0)
        fprintf(stderr, "[HAL] 轴槽%d %s 失败: rc=%d (%s)\n", slot, what, rc, text);
    else
        /* 生命周期/周期循环出错没有具体轴槽，传 -1 */
        fprintf(stderr, "[HAL] %s 失败: rc=%d (%s)\n", what, rc, text);
}

/* 每轴每类只报一次，成功才清；HAL_ERROR_NOT_ENABLED 是预期内过渡态，连着失败到
 * 第 N 拍才报一行，其它错误码当场报一次 */
static void report_req(const char* what, int slot, int kind, int32_t rc) {
    if (rc == HAL_OK) {
        s_req_fail_rc[slot][kind] = 0;
        s_req_fail_age[slot][kind] = 0;
        return;
    }

    if (s_req_fail_rc[slot][kind] != rc) {   /* 换了错误码就重新计数 */
        s_req_fail_rc[slot][kind] = rc;
        s_req_fail_age[slot][kind] = 0;
    }

    if (rc == HAL_ERROR_NOT_ENABLED) {
        /* 不报也不计数的两种情形：没在请求使能；或复位在飞（使能本就被压着不发）。
         * 请求了使能且无复位在飞、连着 N 拍被同一个码拒，才值得报。 */
        if (!s_req_enable[slot] || s_snap_reset[slot] == HAL_RESET_PENDING) {
            s_req_fail_age[slot][kind] = 0;
            return;
        }
        if (s_req_fail_age[slot][kind] >= RT_HAL_NOT_ENABLED_WAIT_CYCLES) return;  /* 报过了 */
        if (++s_req_fail_age[slot][kind] < RT_HAL_NOT_ENABLED_WAIT_CYCLES) return; /* 还没到点 */
    } else if (s_req_fail_age[slot][kind] != 0) {
        return;                              /* 这个码已经报过一次 */
    }

    log_hs(what, slot, rc);
    if (rc != HAL_ERROR_NOT_ENABLED) s_req_fail_age[slot][kind] = 1;  /* 报过了 */
}

/* ---- 生命周期 ---- */

int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len) {
    if (!params) return RT_HAL_ERR_NULL_ARG;
    if (s_ctx) return HAL_ERROR_BUSY;   /* 重复 create；当前只支持一个实例 */

    if (err && err_len) err[0] = '\0';

    /* 装配 profile（占位换算量） */
    HalCConfig cfg;
    int rc = rt_hal_config_build(&cfg, err, err_len);
    if (rc) return rc;

    /* 逐轴换机床真值；缺参数的轴不调 set_scaling，保持占位值并挂台账 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        if (!params[i].loaded) continue;
        rc = rt_hal_config_set_scaling(&cfg, (RtHalAxisSlot)i, &params[i].scaling);
        if (rc) {
            if (err && err_len)
                snprintf(err, err_len, "第 %d 根轴的机床参数非法（当量/enc_off/每转计数）", i);
            return rc;
        }
    }

    /* 主轴转速上限；缺参数保持占位值 */
    if (params[RT_HAL_AXIS_SP].loaded && params[RT_HAL_AXIS_SP].max_speed > 0.0) {
        rc = rt_hal_config_set_spindle_max(&cfg, params[RT_HAL_AXIS_SP].max_speed);
        if (rc) {
            if (err && err_len)
                snprintf(err, err_len, "主轴转速上限非法（%.6g rpm）",
                         params[RT_HAL_AXIS_SP].max_speed);
            return rc;
        }
    } else {
        printf("[HAL] 主轴转速上限未从参数域装载（参数为空），保持占位值——"
               "超过占位上限的转速会被 HAL 拒收\n");
    }

    /* 校验 + 打印：没硬件也能先看配置对不对 */
    rc = rt_hal_config_finish(&cfg, err, err_len);
    if (rc) return rc;

    s_cycle_timeout_ms = cfg.cycle_timeout_ms;

    /* 创建 context */
    HalContext* ctx = NULL;
    rc = hal_context_create(&cfg, &ctx, err, err_len);
    if (rc) { log_hs("hal_context_create", -1, rc); return rc; }
    if (!ctx) {
        if (err && err_len) snprintf(err, err_len, "hal_context_create 返回成功但句柄为空");
        return HAL_ERROR_STATE;
    }

    s_ctx = ctx;
    s_snap_valid = s_io_ready = 0;
    s_in_cycle = 0;
    memset(s_req_pos_dirty, 0, sizeof(s_req_pos_dirty));
    memset(s_req_setpos_dirty, 0, sizeof(s_req_setpos_dirty));
    memset(s_req_turn_dirty, 0, sizeof(s_req_turn_dirty));
    memset(s_req_speed_dirty, 0, sizeof(s_req_speed_dirty));
    memset(s_req_estop, 0, sizeof(s_req_estop));
    memset(s_req_reset, 0, sizeof(s_req_reset));
    memset(s_req_enable_dirty, 0, sizeof(s_req_enable_dirty));
    memset(s_req_fail_rc, 0, sizeof(s_req_fail_rc));
    memset(s_req_fail_age, 0, sizeof(s_req_fail_age));
    memset(s_snap_reset, 0, sizeof(s_snap_reset));
    memset(s_snap_orient, 0, sizeof(s_snap_orient));
    memset(s_snap_fresh, 0, sizeof(s_snap_fresh));
    memset(s_snap_spindle_fresh, 0, sizeof(s_snap_spindle_fresh));
    s_cycle_error = 0;
    memset(s_req_mode_dirty, 0, sizeof(s_req_mode_dirty));
    __atomic_store_n(&plc_cycle_target, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&plc_done_cycle, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&motion_cycle_target, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&motion_done_cycle, 0, __ATOMIC_RELEASE);
    return HAL_OK;
}

int rt_hal_start(char* err, uint32_t err_len) {
    if (!s_ctx) {
        if (err && err_len) snprintf(err, err_len, "尚未 rt_hal_create");
        return RT_HAL_ERR_ORDER;
    }
    if (err && err_len) err[0] = '\0';

    const int32_t rc = hal_context_start(s_ctx, err, err_len);
    if (rc) { log_hs("hal_context_start", -1, rc); return rc; }

    /* 问一次映像段长并缓存；失败不致命（按 0 处理 = 不下发 IO） */
    s_x_size = s_y_size = 0;
    (void)hal_io_image_size(s_ctx, &s_x_size, &s_y_size);
    printf("映像段长（按设备 Entry 实算）：X=%u 字节, Y=%u 字节\n",
           (unsigned)s_x_size, (unsigned)s_y_size);
    /* 段长超本适配层缓冲必须在这里拒绝启动：否则 IO 无声失效、线程层按 HAL 段长写
     * 自己的 512 字节缓冲会越界写。RT_HAL_IO_IMAGE_MAX 三处一致（REG_X/REG_Y、线程层）。 */
    if (s_x_size > RT_HAL_IO_IMAGE_MAX || s_y_size > RT_HAL_IO_IMAGE_MAX) {
        if (err && err_len)
            snprintf(err, err_len,
                     "IO 映像段长超上限：X=%u Y=%u 字节 > %u（要放大就改 RT_HAL_IO_IMAGE_MAX，"
                     "并同步 REG_X/REG_Y 与线程层缓冲）",
                     (unsigned)s_x_size, (unsigned)s_y_size, (unsigned)RT_HAL_IO_IMAGE_MAX);
        log_hs("IO 映像段长超上限", -1, HAL_ERROR_CONFIG);
        return HAL_ERROR_CONFIG;
    }

    s_running = 1;
    /* emcmotTrajPlan() 拿 op_flag 当门控；"主站已进 OP"只有这里知道 */
    op_flag = 1;
    return HAL_OK;
}

void rt_hal_shutdown(void) {
    if (!s_ctx) return;

    op_flag = 0;   /* 先关门控 */

    /* 调用方须先保证 PDO 线程已退出；本函数不 join */
    if (s_running) {
        (void)hal_context_request_stop(s_ctx);
        (void)hal_context_stop(s_ctx);
        s_running = 0;
    }
    hal_context_destroy(s_ctx);
    s_ctx = NULL;

    s_snap_valid = s_io_ready = 0;
    memset(s_snap_fresh, 0, sizeof(s_snap_fresh));
    memset(s_snap_spindle_fresh, 0, sizeof(s_snap_spindle_fresh));
    s_in_cycle = 0;
    s_x_size = s_y_size = 0;
}

int rt_hal_running(void) { return s_running; }

int rt_hal_sdo_read(int slave_pos, uint16_t index, uint8_t subindex,
                    void* data, uint32_t size, uint32_t timeout_ms) {
    if (!s_ctx || !s_running) return HAL_ERROR_NOT_RUNNING;
    return hal_sdo_read(s_ctx, slave_pos, index, subindex, data, size, timeout_ms);
}

int rt_hal_sdo_write(int slave_pos, uint16_t index, uint8_t subindex,
                     const void* data, uint32_t size, uint32_t timeout_ms) {
    if (!s_ctx || !s_running) return HAL_ERROR_NOT_RUNNING;
    return hal_sdo_write(s_ctx, slave_pos, index, subindex, data, size, timeout_ms);
}

int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size) {
    if (!x_size || !y_size) return HAL_ERROR_ARGUMENT;
    if (!s_running) return HAL_ERROR_NOT_RUNNING;
    *x_size = s_x_size;
    *y_size = s_y_size;
    return HAL_OK;
}

/* ---- 周期（仅 PDO 线程） ---- */

/* 取本拍快照；只读，不触发总线访问 */
static void publish_snapshot(void) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        s_snap_fresh[i] = 0;
        s_snap_spindle_fresh[i] = 0;
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;
        /* 读成功才覆盖快照；失败保留上一拍——过期的真值比新鲜的零安全 */
        HalCAxisStatus st;
        HalCSpindleStatus sp;
        if (hal_rt_axis_read_status(s_ctx, (HalAxisId)hal_axis, &st) != HAL_OK) continue;
        memcpy(&s_snap_status[i], &st, sizeof(st));
        s_snap_fresh[i] = 1;
        if (hal_rt_spindle_read_status(s_ctx, (HalAxisId)hal_axis, &sp) == HAL_OK) {
            memcpy(&s_snap_spindle[i], &sp, sizeof(sp));
            s_snap_spindle_fresh[i] = 1;
        }
        /* 复位进展进快照：别的线程用它判断收没收口，但不许碰 context */
        int32_t rst = HAL_RESET_NONE;
        if (hal_rt_axis_fault_reset_state(s_ctx, (HalAxisId)hal_axis, &rst) != HAL_OK)
            rst = HAL_RESET_NONE;
        s_snap_reset[i] = rst;
        /* 定向状态进快照：公开查询接口给别的线程，但不能碰 context */
        int32_t ori = HAL_ORIENT_NONE;
        if (hal_rt_spindle_orient_state(s_ctx, (HalAxisId)hal_axis, &ori) != HAL_OK)
            ori = HAL_ORIENT_NONE;
        s_snap_orient[i] = ori;
    }
    s_snap_valid = 1;
}

/* 取走本拍命令下发给 HAL，须在 begin 之后 commit 之前调用；
 * 急停优先且排他：有急停意图时该轴普通请求一条都不下发 */
static void take_requests(void) {
    const int motion_ok = rt_hal_config_motion_allowed();

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;

        /* 设坐标零点：只平移软件基准、不写驱动器，故不受台账阻挡，放在急停分支之前；
         * 必须在位置写之前消费（同拍后面的 write_pos 要按新零点算）。 */
        if (s_req_setpos_dirty[i]) {
            s_req_setpos_dirty[i] = 0;
            report_req("hal_rt_axis_set_pos", i, RT_REQ_SETPOS,
                       hal_rt_axis_set_pos(s_ctx, (HalAxisId)hal_axis, s_req_setpos[i]));
        }

        if (s_req_estop[i]) {
            s_req_estop[i] = 0;
            /* 急停不受台账阻挡 */
            report_req("hal_rt_axis_estop", i, RT_REQ_ESTOP,
                       hal_rt_axis_estop(s_ctx, (HalAxisId)hal_axis));
            s_req_reset[i] = 0;   /* 急停覆盖同拍的复位请求 */
            /* 急停顺带撤掉使能意图；留着旧值 1 会让之后每拍的"轴尚未使能"被当成失败 */
            s_req_enable[i] = 0;
            s_req_enable_dirty[i] = 0;
            /* 暂存的位置/转速/模式/刻度本拍作废即丢 dirty，否则下拍会被当新意图消费 */
            s_req_pos_dirty[i] = s_req_speed_dirty[i] = 0;
            s_req_mode_dirty[i] = s_req_turn_dirty[i] = 0;
            continue;             /* 本拍该轴的其余请求全部作废 */
        }

        /* 故障复位同样不受台账阻挡 */
        if (s_req_reset[i]) {
            s_req_reset[i] = 0;
            /* 同急停：复位期间暂存目标已过期，作废本拍 */
            s_req_pos_dirty[i] = s_req_speed_dirty[i] = 0;
            s_req_mode_dirty[i] = s_req_turn_dirty[i] = 0;
            report_req("hal_rt_axis_fault_reset", i, RT_REQ_RESET,
                       hal_rt_axis_fault_reset(s_ctx, (HalAxisId)hal_axis));
            continue;   /* 复位与运动互斥，本拍不再下发运动指令 */
        }

        /* 使能/去使能不过台账，失败保留 dirty 下拍重试。复位在飞时必须忍住：HAL 契约是
         * "急停/去使能取消待复位"，而 RT 停车路径每拍都下去使能，漏下去复位永不收口；
         * 判据看快照里 HAL 的复位状态，收口后照常下发。 */
        if (s_req_enable_dirty[i] && s_snap_reset[i] != HAL_RESET_PENDING) {
            const int32_t erc = hal_rt_axis_enable(s_ctx, (HalAxisId)hal_axis, s_req_enable[i]);
            report_req("hal_rt_axis_enable", i, RT_REQ_ENABLE, erc);
            if (erc == HAL_OK) s_req_enable_dirty[i] = 0;   /* 失败保留，下一拍重试 */
        }

        /* 定向的请求与释放放在台账闸门之前：定向不产生位移，不该被"运动范围未批准"拦住，
         * 否则台账没清时 M19 毫无反应。一次性、失败也清；释放放在请求之前，同拍以释放为准。 */
        if (s_req_orient_rel_dirty[i]) {
            s_req_orient_rel_dirty[i] = 0;
            s_req_orient_dirty[i] = 0;
            report_req("hal_rt_spindle_release_orient", i, RT_REQ_ORIENT,
                       hal_rt_spindle_release_orient(s_ctx, (HalAxisId)hal_axis));
        } else if (s_req_orient_dirty[i]) {
            s_req_orient_dirty[i] = 0;
            report_req("hal_rt_spindle_request_orient", i, RT_REQ_ORIENT,
                       hal_rt_spindle_request_orient(s_ctx, (HalAxisId)hal_axis));
        }

        if (!motion_ok) continue;

        /* 位置/转速/模式是一次性意图：报失败但不重试，留着 dirty 会在急停后、
         * 轴重新使能时把过期目标下下去 */
        if (s_req_pos_dirty[i]) {
            s_req_pos_dirty[i] = 0;
            report_req("hal_rt_axis_write_pos", i, RT_REQ_POS,
                       hal_rt_axis_write_pos(s_ctx, (HalAxisId)hal_axis, s_req_pos[i]));
        }
        /* 刻度命令放在位置写之后：同拍都置时以刻度为准（HAL 后写覆盖先写），
         * 一次性意图，失败也清 */
        if (s_req_turn_dirty[i]) {
            s_req_turn_dirty[i] = 0;
            report_req("hal_rt_spindle_write_pos", i, RT_REQ_POS,
                       hal_rt_spindle_write_pos(s_ctx, (HalAxisId)hal_axis, s_req_turn[i]));
        }
        if (s_req_speed_dirty[i]) {
            s_req_speed_dirty[i] = 0;
            report_req("hal_rt_spindle_write_speed", i, RT_REQ_SPEED,
                       hal_rt_spindle_write_speed(s_ctx, (HalAxisId)hal_axis,
                                                  s_req_speed[i], s_req_speed_dir[i]));
        }

        if (s_req_mode_dirty[i]) {
            s_req_mode_dirty[i] = 0;
            report_req("hal_rt_spindle_request_mode", i, RT_REQ_MODE,
                       hal_rt_spindle_request_mode(s_ctx, (HalAxisId)hal_axis, s_req_mode[i]));
            s_mode_requested[i] = 1;  /* 从这一刻起才值得跟驱动器对模式 */
            s_mode_wait[i] = 0;      /* 新请求，重新计时 */
            s_mode_warned[i] = 0;
        }

        /* 模式切换握手：比驱动器实际模式（0x6061）与最近一次请求的模式。两个前提不满足
         * 就不握手，否则是假警报（启动时没发过模式，拿 s_req_mode 初值去比永远对不上）：
         * ① 真的发过一次模式请求；② 该轴当前被请求使能（未使能时驱动器不切模式）。
         * 被去使能就清掉"发过请求"的记号。 */
        {
            const int want_enable = s_req_enable[i] != 0;
            if (!want_enable) s_mode_requested[i] = 0;
            const int handshake =
                s_mode_requested[i] && want_enable && (RtHalAxisSlot)i == RT_HAL_AXIS_SP;

            HalCSpindleStatus sst;
            int mode_ok = 1;
            if (handshake &&
                rt_hal_spindle_status((RtHalAxisSlot)i, &sst) == HAL_OK &&
                sst.mode != s_req_mode[i]) {
                mode_ok = 0;
            }
            if (!handshake || mode_ok) {
                s_mode_wait[i] = 0;
                s_mode_warned[i] = 0;
            } else if (++s_mode_wait[i] >= RT_HAL_MODE_WAIT_CYCLES && !s_mode_warned[i]) {
                s_mode_warned[i] = 1;
                const char* want = s_req_mode[i] == HAL_SPINDLE_CSP ? "CSP" : "CSV";
                fprintf(stderr,
                        "[HAL] 轴槽%d 主轴模式切换超时：已等 %u 拍，实际模式仍不是 %s\n",
                        i, s_mode_wait[i], want);
            }
        }
    }
}

/* 本拍业务等待截止时刻；PLC 与运动共用，各自等一个完整周期会把一拍拖成两拍 */
static void make_deadline(struct timespec* ts) {
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec  += (time_t)(s_cycle_timeout_ms / 1000u);
    ts->tv_nsec += (long)(s_cycle_timeout_ms % 1000u) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) { ts->tv_sec += 1; ts->tv_nsec -= 1000000000L; }
}

/* 等一个带周期号的完成信号：信号量只证明有人 post 过，不证明 post 的是本拍；
 * 对不上周期号就继续等。 */
static int32_t wait_done(const char* what, volatile uint32_t* done, sem_t* sem,
                         uint32_t cycle, const struct timespec* deadline) {
    while (__atomic_load_n(done, __ATOMIC_ACQUIRE) != cycle) {
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) return HAL_ERROR_STOPPED;
        if (sem_timedwait(sem, deadline) != 0) {
            if (errno == EINTR) continue;
            if (errno == ETIMEDOUT) {
                fprintf(stderr, "[HAL] 第 %u 拍等 %s 完成超时（上限 %u ms）\n",
                        cycle, what, s_cycle_timeout_ms);
                return HAL_ERROR_TIMEOUT;
            }
            return HAL_ERROR_STATE;
        }
    }
    return HAL_OK;
}

/* 超时/中止时的停机帧：全轴急停 + 停机映像 + 提交一帧；只能 begin 与 commit 之间调 */
static void stop_frame(void) {
    if (!s_in_cycle) return;

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;
        (void)hal_rt_axis_estop(s_ctx, (HalAxisId)hal_axis);
    }

    if (s_stop_image_valid) {
        (void)hal_rt_io_flush_outputs(s_ctx, s_stop_image, RT_HAL_IO_IMAGE_MAX);
    } else {
        /* 不写 Y，已下发的电平保持原状 */
        fprintf(stderr, "[HAL] 停机帧：未配置停机输出映像，本帧不写 Y（计划书 §8）\n");
    }

    (void)hal_rt_commit_cycle(s_ctx);   /* 把上面的急停控制字发出去 */
    s_in_cycle = 0;
}

/* 只读快照转储：CNC_RT_DUMP=1 打第 100 拍、=N 打第 N 拍，支持逗号列表；不参与控制 */
static void dump_snapshot_once(uint32_t cycle) {
    const char* on = getenv("CNC_RT_DUMP");
    if (!on || !*on) return;
    /* =1 打第 100 拍（等采样稳定）；要看晚态传更大的 N */
    int hit = 0;
    for (const char* p = on; *p; ) {
        char* end = NULL;
        const long v = strtol(p, &end, 10);
        if (end == p) break;                        /* 不是数字就停 */
        if (v > 0 && cycle == (uint32_t)(v == 1 ? 100 : v)) { hit = 1; break; }
        if (*end != ',') break;
        p = end + 1;                                /* 跳过逗号 */
    }
    if (!hit) return;

    printf("[HAL] ===== 只读快照 @第 %u 拍（未使能）=====\n", cycle);
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;
        HalCIdentity id;
        HalCAxisStatus st;
        const int ok_id = hal_device_identity(s_ctx, (HalAxisId)hal_axis, &id) == HAL_OK;
        const int ok_st = hal_rt_axis_read_status(s_ctx, (HalAxisId)hal_axis, &st) == HAL_OK;
        printf("  HAL轴%-2d 身份=%-12s vendor=0x%04x product=0x%04x | 使能=%d 位置=%.6f 速度=%.3f%s 状态字=0x%04x%s\n",
               hal_axis,
               ok_id ? id.name : "(读不到)",
               ok_id ? id.vendor_id : 0u,
               ok_id ? id.product_code : 0u,
               ok_st ? st.enabled : -1,
               ok_st ? st.actual_pos : 0.0,
               ok_st ? st.actual_speed : 0.0,
               (ok_st && st.speed_valid) ? "" : "(无速度PDO)",
               ok_st ? st.raw_status : 0,
               ok_st ? "" : "  [读取失败]");
    }
    if (s_io_ready) {
        printf("  输入映像 X 前 12 字节:");
        for (uint32_t i = 0; i < 12u && i < RT_HAL_IO_IMAGE_MAX; ++i) printf(" %02x", s_x_image[i]);
        printf("\n");
    } else {
        printf("  输入映像：本拍没取到\n");
    }
    printf("[HAL] =====================================\n");
    fflush(stdout);
}

int rt_hal_pdo_loop(void) {
    if (!s_ctx || !s_running) return RT_HAL_ERR_ORDER;

    uint32_t cycle = 0;
    int32_t rc = HAL_OK;

    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        ++cycle;

        rc = hal_rt_wait_cycle(s_ctx);            /* 等节拍、收帧 */
        if (rc) break;

        rc = hal_rt_begin_cycle(s_ctx);           /* 采样、推 DS402、准备输入映像 */
        if (rc) break;
        s_in_cycle = 1;

        publish_snapshot();
        s_io_ready = (hal_rt_io_snapshot_inputs(s_ctx, s_x_image, RT_HAL_IO_IMAGE_MAX) == HAL_OK);

        /* 周期号先写再放行，业务线程才知道自己算第几拍 */
        __atomic_store_n(&plc_cycle_target, cycle, __ATOMIC_RELEASE);
        __atomic_store_n(&motion_cycle_target, cycle, __ATOMIC_RELEASE);
        sem_post(&plc_sem);
        sem_post(&motion_sem);
        sem_post(&path_plan_sem);

        struct timespec deadline;
        make_deadline(&deadline);

        rc = wait_done("PLC", &plc_done_cycle, &plc_done_sem, cycle, &deadline);
        if (rc) { stop_frame(); break; }
        rc = wait_done("运动", &motion_done_cycle, &moton_sem_exec_done, cycle, &deadline);
        if (rc) { stop_frame(); break; }

        take_requests();

        if (s_io_ready && hal_rt_io_flush_outputs(s_ctx, s_y_image, RT_HAL_IO_IMAGE_MAX) != HAL_OK) {
            /* IO 下发失败不让整圈失败：commit 会再报同一个总线错误 */
            s_io_ready = 0;
        }

        rc = hal_rt_commit_cycle(s_ctx);          /* 本拍唯一发送出口 */
        s_in_cycle = 0;
        if (rc) break;

        dump_snapshot_once(cycle);
    }

    if (rc && !__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        /* 记下来给 RT 侧报警用，不能只打 stderr */
        s_cycle_error = rc;
        log_hs("周期循环", -1, rc);
    }
    return rc;
}

int32_t rt_hal_cycle_error(void) { return s_cycle_error; }

int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len) {
    if (!y) return HAL_ERROR_ARGUMENT;
    if (len < RT_HAL_IO_IMAGE_MAX) return HAL_ERROR_ARGUMENT;
    memcpy(s_stop_image, y, RT_HAL_IO_IMAGE_MAX);
    s_stop_image_valid = 1;
    return HAL_OK;
}

/* ---- 反馈快照（任意线程，只读快照） ---- */

int rt_hal_axis_pos(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].position_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_pos;
    return HAL_OK;
}

int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_status[slot];
    return HAL_OK;
}

/* speed_valid=0 → UNSUPPORTED，让上层区分"没速度 PDO"和"停着" */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].speed_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_speed;
    return HAL_OK;
}

/* 与 rt_hal_axis_speed() 同一读数，换成 rpm 口径（HAL 已按 speed_unit 换算） */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].speed_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_speed_rpm;
    return HAL_OK;
}

int rt_hal_axis_speed_mm_min(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].linear_speed_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_speed_mm_min;
    return HAL_OK;
}

int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    /* 非主轴槽 */
    if (!rt_hal_config_is_spindle(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_spindle_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_spindle[slot];
    return HAL_OK;
}

int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    /* 非主轴槽 */
    if (!rt_hal_config_is_spindle(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid || !s_snap_spindle_fresh[slot]) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_spindle[slot].actual_speed;
    return HAL_OK;
}

int rt_hal_request_fault_reset(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    /* 有意不做台账检查：驱动器已故障时不该被"参数未确认"拦住 */
    s_req_reset[slot] = 1;
    return HAL_OK;
}

int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_reset[slot];
    return HAL_OK;
}

int rt_hal_fault_reset_settled(int* any_timeout) {
    if (!s_snap_valid) return 0;
    if (any_timeout) *any_timeout = 0;
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        /* 请求还没被 PDO 线程取走 → 没收口 */
        if (s_req_reset[i]) return 0;
        if (s_snap_reset[i] == HAL_RESET_DONE) continue;
        if (s_snap_reset[i] == HAL_RESET_TIMEOUT) {
            if (any_timeout) *any_timeout = 1;
            continue;
        }
        return 0;   /* NONE 或 PENDING 都算没收口 */
    }
    return 1;
}

double rt_hal_spindle_rpm(void) {
    double rpm = 0.0;
    if (rt_hal_spindle_speed(RT_HAL_AXIS_SP, &rpm) != HAL_OK) return 0.0;
    return rpm;
}

/* ---- 命令请求（任意线程） ---- */

/* 请求主轴定向；不受台账阻挡（不产生位移，只是让驱动器对准） */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    s_req_orient[slot] = 1;
    s_req_orient_dirty[slot] = 1;
    return HAL_OK;
}

/* 只读快照，任意线程可调，不碰 HalContext */
int rt_hal_spindle_orient_state(RtHalAxisSlot slot, int32_t* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!rt_hal_config_is_spindle(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_orient[slot];
    return HAL_OK;
}

int rt_hal_spindle_release_orient(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    s_req_orient_rel[slot] = 1;
    s_req_orient_rel_dirty[slot] = 1;
    return HAL_OK;
}

/* 主轴单圈刻度：与 axis_pos 同一条台账闸门（同样会让轴动） */
int rt_hal_request_spindle_turn(RtHalAxisSlot slot, double deg) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    if (!rt_hal_config_motion_allowed()) {
        warn_ledger_blocked("主轴单圈刻度", (int)slot);
        return HAL_ERROR_NOT_RUNNING;
    }
    s_req_turn[slot] = deg;
    s_req_turn_dirty[slot] = 1;
    return HAL_OK;
}

/* 把该轴当前位置重新定义为 pos（用户单位）；攻丝起点、回零/对刀用。
 * 不写驱动器、不动电机，故不过台账闸门 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    s_req_setpos[slot] = pos;
    s_req_setpos_dirty[slot] = 1;
    return HAL_OK;
}

int rt_hal_axis_set_pos_result(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx || !s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    if (s_req_setpos_dirty[slot]) return HAL_ERROR_BUSY;
    return s_req_fail_rc[slot][RT_REQ_SETPOS];
}

int rt_hal_request_axis_pos(RtHalAxisSlot slot, double pos) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    if (!rt_hal_config_motion_allowed()) {
        warn_ledger_blocked("位置指令", (int)slot);
        return HAL_ERROR_NOT_RUNNING;
    }
    s_req_pos[slot] = pos;   /* 同拍后者覆盖前者，命令按拍消费不排队 */
    s_req_pos_dirty[slot] = 1;
    return HAL_OK;
}

int rt_hal_request_spindle_speed(RtHalAxisSlot slot, double rpm, int dir) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (dir < -1 || dir > 1) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    if (!rt_hal_config_motion_allowed()) {
        warn_ledger_blocked("主轴转速", (int)slot);
        return HAL_ERROR_NOT_RUNNING;
    }
    s_req_speed[slot] = rpm;
    s_req_speed_dir[slot] = dir;
    s_req_speed_dirty[slot] = 1;
    return HAL_OK;
}

int rt_hal_request_spindle_mode(RtHalAxisSlot slot, HalSpindleMode mode) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (mode != HAL_SPINDLE_CSV && mode != HAL_SPINDLE_CSP) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    if (!rt_hal_config_motion_allowed()) {
        warn_ledger_blocked("主轴模式", (int)slot);
        return HAL_ERROR_NOT_RUNNING;
    }
    /* 期望模式没变就不重复提交 */
    if (s_req_mode_dirty[slot] && s_req_mode[slot] == mode) return HAL_OK;
    s_req_mode[slot] = mode;
    s_req_mode_dirty[slot] = 1;
    return HAL_OK;
}

/* 期望使能状态，主轴也用这一个；值没变就不重复下发。
 * 有意不做台账检查（台账只关位置/转速/模式）。 */
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (on != 0 && on != 1) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    if (s_req_enable_dirty[slot] && s_req_enable[slot] == on) return HAL_OK;
    s_req_enable[slot] = on;
    s_req_enable_dirty[slot] = 1;
    return HAL_OK;
}

int rt_hal_request_estop(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    /* 有意不做台账检查：急停是安全路径 */
    s_req_estop[slot] = 1;
    return HAL_OK;
}

/* ---- IO 映像 ---- */

int rt_hal_io_fetch_inputs(uint8_t* out, uint32_t len) {
    if (!out) return HAL_ERROR_ARGUMENT;
    if (len < RT_HAL_IO_IMAGE_MAX) return HAL_ERROR_ARGUMENT;
    if (!s_io_ready) return HAL_ERROR_NOT_RUNNING;
    memcpy(out, s_x_image, RT_HAL_IO_IMAGE_MAX);
    return HAL_OK;
}

int rt_hal_io_publish_outputs(const uint8_t* y, uint32_t len) {
    if (!y) return HAL_ERROR_ARGUMENT;
    if (len < RT_HAL_IO_IMAGE_MAX) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    memcpy(s_y_image, y, RT_HAL_IO_IMAGE_MAX);
    return HAL_OK;
}
