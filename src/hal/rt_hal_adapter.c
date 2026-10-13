/* 适配层实现。线程契约见 rt_hal_adapter.h 头部那张表，改之前先读它。 */

/* 严格 ISO 模式下不声明 clock_gettime/sem_timedwait，必须显式要，且必须在 include 前 */
#define _POSIX_C_SOURCE 200809L

#include "rt_hal_adapter.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>   /* getenv/strtol（诊断转储），漏了会隐式声明截断指针 */
#include <string.h>
#include <time.h>

#include "hal_c_api.h"   /* create/start/stop/destroy、hal_error_text */
#include "rt_sem.h"      /* 信号量、thread_exit_flag */

/* ---- PDO 线程之外不准碰的 context ---- */
static HalContext* s_ctx;                 /* 唯一句柄；只有 PDO 线程用它（create/start/stop 例外） */
static int         s_running;             /* 1 = start 成功且未 shutdown；查询接口据此拒绝 */
static uint32_t    s_cycle_timeout_ms;    /* 缓存配置里的周期超时：等两个手的等待上限 */

/* ---- 快照：PDO 线程 begin 之后填一次，本拍内不再改 ---- */
static HalCAxisStatus   s_snap_status[RT_HAL_AXIS_COUNT];    /* 每轴一份轴状态 */
static HalCSpindleStatus s_snap_spindle[RT_HAL_AXIS_COUNT];  /* 转速侧快照；非主轴槽无意义 */
static int              s_snap_valid;                        /* 0 = 本拍还没采过，查询接口要拒 */

/* ---- 请求：其他线程写，PDO 线程在 commit 前取走 ---- */
static double s_req_pos[RT_HAL_AXIS_COUNT];        /* 目标位置（用户单位）；同拍后者覆盖前者 */
static double s_req_setpos[RT_HAL_AXIS_COUNT];     /* 把当前位置重定义为该值（设零/对刀）*/
static int    s_req_setpos_dirty[RT_HAL_AXIS_COUNT];
static double s_req_turn[RT_HAL_AXIS_COUNT];       /* 主轴单圈刻度（度）；同拍优先于 pos */
static int    s_req_pos_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_turn_dirty[RT_HAL_AXIS_COUNT];
static double s_req_speed[RT_HAL_AXIS_COUNT];      /* 目标转速 rpm */
static int    s_req_speed_dir[RT_HAL_AXIS_COUNT];  /* 转向与 rpm 分开传：rpm 本身不许为负 */
static int    s_req_speed_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_estop[RT_HAL_AXIS_COUNT];      /* 1 = 请求急停。一次性事件 */
static int    s_req_reset[RT_HAL_AXIS_COUNT];      /* 1 = 请求故障复位。一次性事件 */
/* 期望使能状态，不是事件：每拍重新声明，与基线每拍推 DS402 梯子对应 */
static int     s_req_enable[RT_HAL_AXIS_COUNT];          /* 1 = 要力矩，0 = 去使能 */
static int     s_req_enable_dirty[RT_HAL_AXIS_COUNT];    /* 只有值变了才下发；失败**保留**，下拍重试 */
/* 每轴每类请求上次的失败码；只在失败码变化时报一次，避免逐拍刷屏 */
enum { RT_REQ_POS = 0, RT_REQ_SPEED, RT_REQ_MODE, RT_REQ_ENABLE, RT_REQ_ESTOP, RT_REQ_RESET,
       RT_REQ_ORIENT, RT_REQ_SETPOS, RT_REQ_KINDS };
static int32_t s_req_fail_rc[RT_HAL_AXIS_COUNT][RT_REQ_KINDS];
/* "轴尚未使能"这类预期内失败已经连着失败了多少拍；到点才报一行，见 report_req()。 */
static uint32_t s_req_fail_age[RT_HAL_AXIS_COUNT][RT_REQ_KINDS];
static int32_t s_snap_reset[RT_HAL_AXIS_COUNT];   /* 复位进展快照 */
static int32_t s_snap_orient[RT_HAL_AXIS_COUNT];  /* 主轴定向状态快照 */
static int32_t s_cycle_error;                     /* 周期循环异常退出的原因；0 = 无 */
static HalSpindleMode s_req_mode[RT_HAL_AXIS_COUNT];      /* 期望的 CSV / CSP */
static int    s_req_mode_dirty[RT_HAL_AXIS_COUNT];        /* 同上：只有变了才下发 */
/* 定向请求/释放也是按拍意图：运动线程写，PDO 线程在相位 2 里调 HAL。
 * 一次性，消费即清（不像使能那样重试）。 */
static int    s_req_orient[RT_HAL_AXIS_COUNT];       /* 1 = 请求定向 */
static int    s_req_orient_dirty[RT_HAL_AXIS_COUNT];
static int    s_req_orient_rel[RT_HAL_AXIS_COUNT];   /* 1 = 释放定向 */
static int    s_req_orient_rel_dirty[RT_HAL_AXIS_COUNT];
/* 模式切换握手：请求下发后连续多少拍驱动器实际模式（0x6061）还没变成我们要的。
 * 超过上限报一次——HAL 是按拍推进的（模式不对不上使能），没有握手就只能看到
 * 一堆被丢掉的返回值，分不清"正在切"和"永远切不过去"。 */
#define RT_HAL_MODE_WAIT_CYCLES 1000u
/* "轴尚未使能"等多久才值得报：使能只是请求，DS402 要几拍才生效 */
#define RT_HAL_NOT_ENABLED_WAIT_CYCLES 1000u
static uint32_t s_mode_wait[RT_HAL_AXIS_COUNT];      /* 本次请求已等了多少拍；请求变了清零 */
static int      s_mode_warned[RT_HAL_AXIS_COUNT];    /* 超上限报过一次就不再刷 */
static uint8_t  s_mode_requested[RT_HAL_AXIS_COUNT]; /* 发过模式请求没；去使能时清掉 */

/* IO 单缓冲，512 与 REG_X_NUM/REG_Y_NUM 一致；不引 cncshm 头 */
static uint8_t s_x_image[RT_HAL_IO_IMAGE_MAX];   /* 输入映像：PDO 线程从 HAL 取，别的线程读 */
static uint8_t s_y_image[RT_HAL_IO_IMAGE_MAX];   /* 输出映像：别的线程写，PDO 线程下发 */
static int     s_io_ready;                 /* 1 = 本拍输入映像取到了；取不到连输出也不发 */
/* 映像段长，start 后向 HAL 问一次并缓存；不要自己推 */
static uint32_t s_x_size, s_y_size;        /* X / Y 段长（字节） */
static int     s_in_cycle;                 /* 已 begin 未 commit：停机帧只能在这个窗口发 */

/* 超时停机要下发的 Y。默认未配置：不能假设全零安全，宁可不写 */
static uint8_t s_stop_image[RT_HAL_IO_IMAGE_MAX];   /* 停机时要写出去的 Y 映像 */
static int     s_stop_image_valid;            /* 0 = 没配过，那就什么都不写 */

/* 入参一律是 RT 轴槽；跨到 HAL 时统一走 rt_hal_config_hal_axis()。 */
static int slot_ok(int slot) {
    return slot >= 0 && slot < (int)RT_HAL_AXIS_COUNT;
}

/* 台账挡住运动请求时提示一次。被挡的请求是**静默丢弃**的——软件侧一切正常
 * （主轴斜坡照爬、日志照打"速度到达"），只有驱动器收不到、HMI 显示露馅。
 * 2026-10-09 查"S 显示恒 0"花了十几轮就栽在这上面，所以必须让它出声。
 * 台账清掉之后重新允许提示。 */
static int s_ledger_warned;
static void warn_ledger_blocked(const char* what, int slot) {
    if (rt_hal_config_motion_allowed()) { s_ledger_warned = 0; return; }
    if (s_ledger_warned) return;
    s_ledger_warned = 1;
    fprintf(stderr, "[HAL] 运动台账未清，轴槽%d 的 %s 被丢弃"
                    "（软件侧看起来正常，但驱动器收不到）\n", slot, what);
    fflush(stderr);
}

/* HAL 的错误码是正数，不能沿用旧 RT 的 <0 判断。
 * 带上轴槽：不带的话四根轴同一个码失败，日志里看起来只报了一次。 */
static void log_hs(const char* what, int slot, int32_t rc) {
    char text[96];
    hal_error_text(rc, text, sizeof(text));
    if (slot >= 0)
        fprintf(stderr, "[HAL] 轴槽%d %s 失败: rc=%d (%s)\n", slot, what, rc, text);
    else
        /* 生命周期/周期循环出错没有具体轴槽，槽位传 -1。 */
        fprintf(stderr, "[HAL] %s 失败: rc=%d (%s)\n", what, rc, text);
}

/* 每轴每类只报一次，避免逐拍刷屏。原来是 (void) 丢返回值，失败了没人知道。
 * 去掉某个轴的失败记录只靠它自己成功，别的轴不影响它。
 *
 * 例外：HAL_ERROR_NOT_ENABLED（"轴尚未使能"）是**预期内**的过渡态 —— 使能只是请求，
 * DS402 要几拍才走到 Operation Enabled，紧挨着使能请求发出的那几拍（定向、转速、
 * 位置）都会被拒，上层下一拍重发就过了。所以它先不吭声，**连着失败到第 N 拍才报一行**
 * —— 那才是真的使不上能。其它错误码不会自己好，照旧当场报一次。 */
static void report_req(const char* what, int slot, int kind, int32_t rc) {
    if (rc == HAL_OK) {
        s_req_fail_rc[slot][kind] = 0;
        s_req_fail_age[slot][kind] = 0;
        return;
    }

    if (s_req_fail_rc[slot][kind] != rc) {   /* 换了错误码：新一轮，重新计数 */
        s_req_fail_rc[slot][kind] = rc;
        s_req_fail_age[slot][kind] = 0;
    }

    if (rc == HAL_ERROR_NOT_ENABLED) {
        /* 什么情况下"轴尚未使能"是**预期内**的（不报也不计数）：
         *   - 本轴没在请求使能：没使能就是不能写位置/转速/定向，上层下一拍重发；
         *   - 故障复位在飞：使能本来就被我们压着不发（见上面 enable 那段的闸门），
         *     这几秒写不进去是设计使然（estopmid 那 3 条就是这么来的）。
         * 反过来才值得说：请求了使能、也没复位在飞，连着 N 拍还被同一个码拒。 */
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
    if (rc != HAL_ERROR_NOT_ENABLED) s_req_fail_age[slot][kind] = 1;  /* 标记：报过了 */
}

/* ============================================================================
 * 生命周期
 * ========================================================================== */

int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len) {
    if (!params) return RT_HAL_ERR_NULL_ARG;
    if (s_ctx) return HAL_ERROR_BUSY;   /* 重复 create；当前只支持一个实例 */

    if (err && err_len) err[0] = '\0';

    /* 1. 装配 profile（占位换算量） */
    HalCConfig cfg;
    int rc = rt_hal_config_build(&cfg, err, err_len);
    if (rc) return rc;

    /* 2. 逐轴换成机床真值。缺参数的轴不调用 set_scaling：保持占位值并挂台账 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        if (!params[i].loaded) continue;
        rc = rt_hal_config_set_scaling(&cfg, (RtHalAxisSlot)i, &params[i].scaling);
        if (rc) {
            if (err && err_len)
                snprintf(err, err_len, "第 %d 根轴的机床参数非法（当量/enc_off/每转计数）", i);
            return rc;
        }
    }

    /* 2b. 主轴转速上限：同上，缺参数就保持占位值（print 里能看出来）。
     * 不接这一步的后果是"S 超过占位上限时 HAL 拒收"，而 RT 侧只有一行 stderr。 */
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

    /* 3. 校验 + 打印。打印放 create 里，没硬件时也能先看配置对不对 */
    rc = rt_hal_config_finish(&cfg, err, err_len);
    if (rc) return rc;

    s_cycle_timeout_ms = cfg.cycle_timeout_ms;

    /* 4. 创建 context */
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

    /* 问一次映像段长并缓存。失败不致命：周期里按 0 处理，等于不下发 IO。 */
    s_x_size = s_y_size = 0;
    (void)hal_io_image_size(s_ctx, &s_x_size, &s_y_size);
    printf("映像段长（按设备 Entry 实算）：X=%u 字节, Y=%u 字节\n",
           (unsigned)s_x_size, (unsigned)s_y_size);
    /* 段长超过本适配层缓冲就必须在这里挡住，不能往下跑：
     *   - HAL 侧只会在"缓冲太短"时返回 ARGUMENT，而周期里那两处把结果
     *     当成"没就绪" —— IO 会**无声失效**（输入恒 0、输出不下发）；
     *   - 线程层按 HAL 给的段长循环写自己的 512 字节缓冲 → **越界写**。
     * 这两条都比"启动失败"糟得多。改大就得三处一起改（见头文件的注释）。 */
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
    /* emcmotTrajPlan() 拿 op_flag 当门控，"主站已进 OP"这个事实只有这里知道。 */
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
    s_in_cycle = 0;
    s_x_size = s_y_size = 0;
}

int rt_hal_running(void) { return s_running; }

int rt_hal_io_image_size(uint32_t* x_size, uint32_t* y_size) {
    if (!x_size || !y_size) return HAL_ERROR_ARGUMENT;
    if (!s_running) return HAL_ERROR_NOT_RUNNING;
    *x_size = s_x_size;
    *y_size = s_y_size;
    return HAL_OK;
}

/* ============================================================================
 * 周期（仅 PDO 线程）
 * ========================================================================== */

/* 取本拍快照。只读，不触发总线访问。 */
static void publish_snapshot(void) {
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;
        /* 读成功才覆盖快照；失败保留上一拍的值——过期的真值比新鲜的零安全 */
        HalCAxisStatus st;
        HalCSpindleStatus sp;
        if (hal_rt_axis_read_status(s_ctx, (HalAxisId)hal_axis, &st) != HAL_OK) continue;
        memcpy(&s_snap_status[i], &st, sizeof(st));
        if (hal_rt_spindle_read_status(s_ctx, (HalAxisId)hal_axis, &sp) == HAL_OK)
            memcpy(&s_snap_spindle[i], &sp, sizeof(sp));
        /* 复位进展进快照：其他线程要用它判断收没收口，但不许碰 context */
        int32_t rst = HAL_RESET_NONE;
        if (hal_rt_axis_fault_reset_state(s_ctx, (HalAxisId)hal_axis, &rst) != HAL_OK)
            rst = HAL_RESET_NONE;
        s_snap_reset[i] = rst;
        /* 定向状态也进快照：这个查询是公开接口，别的线程会调，
         * 不能从它们那儿碰 context（契约：只有 PDO 线程碰）。 */
        int32_t ori = HAL_ORIENT_NONE;
        if (hal_rt_spindle_orient_state(s_ctx, (HalAxisId)hal_axis, &ori) != HAL_OK)
            ori = HAL_ORIENT_NONE;
        s_snap_orient[i] = ori;
    }
    s_snap_valid = 1;
}

/* 取走本拍命令下发给 HAL，须在 begin 之后 commit 之前调用。
 * 急停优先且排他：有急停意图时该轴普通请求一条都不下发，不靠 HAL 逐个拒。 */
static void take_requests(void) {
    const int motion_ok = rt_hal_config_motion_allowed();

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;

        /* 设坐标零点：只平移软件基准，**不写驱动器、不动电机** —— 所以不受台账阻挡，
         * 也放在急停分支之前（急停中设零同样安全）。必须在位置写之前消费：同一拍里
         * 后面的 write_pos 要按新零点来算。 */
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
            /* 急停 = 不再请求使能。不清的话 s_req_enable 会留着旧值 1，
             * 之后每拍被拒的"轴尚未使能"看起来就像失败（report_req 用的就是它）。 */
            s_req_enable[i] = 0;
            s_req_enable_dirty[i] = 0;
            /* 光 continue 不够：暂存的位置/转速/模式/刻度会留到**下一拍**
             * （急停解除后）被当成新意图消费掉。这几类都是一次性意图，
             * 作废就是丢 dirty —— 上层要继续运动自然会重发。 */
            s_req_pos_dirty[i] = s_req_speed_dirty[i] = 0;
            s_req_mode_dirty[i] = s_req_turn_dirty[i] = 0;
            continue;             /* 本拍该轴的其余请求全部作废 */
        }

        /* 故障复位同样不受台账阻挡 */
        if (s_req_reset[i]) {
            s_req_reset[i] = 0;
            /* 复位同样作废本拍暂存：跟急停一个道理（复位期间这些目标已过期）。 */
            s_req_pos_dirty[i] = s_req_speed_dirty[i] = 0;
            s_req_mode_dirty[i] = s_req_turn_dirty[i] = 0;
            report_req("hal_rt_axis_fault_reset", i, RT_REQ_RESET,
                       hal_rt_axis_fault_reset(s_ctx, (HalAxisId)hal_axis));
            continue;   /* 复位与运动互斥，本拍不再下发运动指令 */
        }

        /* 使能/去使能不受台账阻挡，台账只管位置、转速、模式。
         * 它是状态不是事件，失败保留 dirty 下一拍重试。 */
        /* **复位在飞时要忍住**：HAL 的契约是"急停/去使能会取消待复位"
         * （见 hal_rt_api.h 的 hal_rt_axis_fault_reset 注释），而下发去使能的
         * 恰恰是 RT 自己的停车路径（spl_drv_disable 每拍一次）。漏下去就等于每拍
         * 把复位取消掉 —— 复位永远收不了口，上层（alarm.c 的 s_reset_waiting）
         * 也就永远不放开，四轴再也上不了力矩。
         * 判据看快照里 HAL 自己的复位状态，不看请求意图；dirty 保留，收口后照常下发。 */
        if (s_req_enable_dirty[i] && s_snap_reset[i] != HAL_RESET_PENDING) {
            const int32_t erc = hal_rt_axis_enable(s_ctx, (HalAxisId)hal_axis, s_req_enable[i]);
            report_req("hal_rt_axis_enable", i, RT_REQ_ENABLE, erc);
            if (erc == HAL_OK) s_req_enable_dirty[i] = 0;   /* 失败保留，下一拍重试 */
        }

        /* 定向的请求与释放：**放在运动台账闸门之前**。
         * 定向不产生位移（只是让驱动器对准），不受"运动范围未批准"拦；
         * 放在闸门后面的话，台账没清时定向请求永远走不到，M19 表现为毫无反应。
         * 一次性，失败也清；释放放在请求前面，同拍两个都置时以释放为准。 */
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

        /* 位置/转速/模式是一次性意图：报失败但不重试。上层每拍会重发，
         * 留着 dirty 反而会在急停后、轴重新使能时把过期目标下下去。 */
        if (s_req_pos_dirty[i]) {
            s_req_pos_dirty[i] = 0;
            report_req("hal_rt_axis_write_pos", i, RT_REQ_POS,
                       hal_rt_axis_write_pos(s_ctx, (HalAxisId)hal_axis, s_req_pos[i]));
        }
        /* 刻度命令放在普通位置写**之后**：同拍两者都置时以刻度为准（HAL 那边
         * 后写覆盖先写）。与 axis_pos 一样是一次性意图，失败也清。 */
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

        /* 模式切换握手。看驱动器实际模式（快照的 mode 来自 0x6061）有没有变成
         * 最近一次请求的那个。别在这里读 desired_mode——HAL 不对外暴露它。
         *
         * 两个前提不满足就**不握手**，否则是假警报（启动时那条
         * "主轴模式切换超时"就是这么来的：主轴还没使能、压根没下发过模式，
         * 拿 s_req_mode 的初值 CSV 去比驱动器当前模式，永远对不上）：
         *   ① 真的发过一次模式请求；
         *   ② 这根轴当前被请求使能——驱动器不会在未使能时切模式。
         * 一旦被去使能就清掉"发过请求"的记号：下次要握手得重新发请求。 */
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

/* 本拍的业务等待截止时刻。PLC 与运动共用——各自等一个完整周期会把一拍拖成两拍。 */
static void make_deadline(struct timespec* ts) {
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec  += (time_t)(s_cycle_timeout_ms / 1000u);
    ts->tv_nsec += (long)(s_cycle_timeout_ms % 1000u) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) { ts->tv_sec += 1; ts->tv_nsec -= 1000000000L; }
}

/* 等一个带周期号的完成信号。
 * 只看信号量不够：它只证明"有人 post 过"，不证明 post 的是本拍——业务线程晚一拍
 * 时旧结果会被当成下一拍的结果消费。所以对不上周期号就继续等。 */
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

/* 超时/中止时的停机帧：全轴急停 + 下发停机映像 + 提交一帧。
 * 只能在 begin 与 commit 之间调用。 */
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
        /* 没配停机映像就不写 Y，已下发的电平保持原状 */
        fprintf(stderr, "[HAL] 停机帧：未配置停机输出映像，本帧不写 Y（计划书 §8）\n");
    }

    (void)hal_rt_commit_cycle(s_ctx);   /* 把上面的急停控制字发出去 */
    s_in_cycle = 0;
}

/* 只读快照转储，CNC_RT_DUMP=1 打第 100 拍、=N 打第 N 拍，也支持逗号列表
 * （如 "9500,14000"）——一次运行里抓两个窗口，省得为了看力矩状态跑两遍。
 * 不参与控制。 */
static void dump_snapshot_once(uint32_t cycle) {
    const char* on = getenv("CNC_RT_DUMP");
    if (!on || !*on) return;
    /* =1 打第 100 拍（等采样稳定）；要看晚态就传更大的 N（或逗号列表） */
    int hit = 0;
    for (const char* p = on; *p; ) {
        char* end = NULL;
        const long v = strtol(p, &end, 10);
        if (end == p) break;                        /* 不是数字就不往下解析了 */
        if (v > 0 && cycle == (uint32_t)(v == 1 ? 100 : v)) { hit = 1; break; }
        if (*end != ',') break;
        p = end + 1;                                /* 跳过逗号继续 */
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

        /* 周期号先写再放行，业务线程醒来才知道自己算第几拍 */
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
            /* IO 下发失败不让整圈失败：commit 会再返回同一个总线错误 */
            s_io_ready = 0;
        }

        rc = hal_rt_commit_cycle(s_ctx);          /* 本拍唯一发送出口 */
        s_in_cycle = 0;
        if (rc) break;

        dump_snapshot_once(cycle);
    }

    if (rc && !__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        /* 记下来给 RT 侧报警用，不能只打一行 stderr */
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

/* ============================================================================
 * 反馈快照
 * ========================================================================== */

int rt_hal_axis_pos(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_status[slot].actual_pos;
    return HAL_OK;
}

int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_status[slot];
    return HAL_OK;
}

/* speed_valid 为 0 时返回 UNSUPPORTED 而不是 0：要让上层区分"没速度 PDO"和"停着" */
int rt_hal_axis_speed(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].speed_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_speed;
    return HAL_OK;
}

/* 与 rt_hal_axis_speed() 同一个读数，换成 rpm 口径（HAL 已按 speed_unit 换算）。 */
int rt_hal_axis_speed_rpm(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    if (!s_snap_status[slot].speed_valid) return HAL_ERROR_UNSUPPORTED;
    *out = s_snap_status[slot].actual_speed_rpm;
    return HAL_OK;
}

int rt_hal_spindle_status(RtHalAxisSlot slot, HalCSpindleStatus* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    /* 非主轴槽 */
    if (!rt_hal_config_is_spindle(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
    *out = s_snap_spindle[slot];
    return HAL_OK;
}

int rt_hal_spindle_speed(RtHalAxisSlot slot, double* out) {
    if (!slot_ok(slot) || !out) return HAL_ERROR_ARGUMENT;
    /* 判断是不是主轴*/
    if (!rt_hal_config_is_spindle(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_snap_valid) return HAL_ERROR_NOT_RUNNING;
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

/* ============================================================================
 * 命令请求
 * ========================================================================== */

/* 请求主轴定向。不受台账阻挡（它不产生位移，只是让驱动器对准）。 */
int rt_hal_spindle_request_orient(RtHalAxisSlot slot) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    s_req_orient[slot] = 1;
    s_req_orient_dirty[slot] = 1;
    return HAL_OK;
}

/* 只读快照，任意线程可调 —— **不碰 HalContext**（那是 PDO 线程的东西）。 */
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

/* 主轴单圈刻度。与 axis_pos 同一条台账闸门——它同样会让轴动。 */
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

/* 把该轴的当前位置重新定义为 pos（用户单位）。攻丝起点要以"当前角度"为 0°；
 * 回零/对刀也要用。**不写驱动器、不动电机**，所以不过台账闸门。 */
int rt_hal_axis_set_pos(RtHalAxisSlot slot, double pos) {
    if (!slot_ok(slot)) return HAL_ERROR_ARGUMENT;
    if (!s_ctx) return HAL_ERROR_NOT_RUNNING;
    s_req_setpos[slot] = pos;
    s_req_setpos_dirty[slot] = 1;
    return HAL_OK;
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
    /* 期望模式没变就不重复提交，少一次跨线程写 */
    if (s_req_mode_dirty[slot] && s_req_mode[slot] == mode) return HAL_OK;
    s_req_mode[slot] = mode;
    s_req_mode_dirty[slot] = 1;
    return HAL_OK;
}

/* 期望使能状态。有意不做台账检查：台账关的是位置/转速/模式，不管使能。
 * 主轴也用这一个（HAL 的通用轴使能路径覆盖主轴）。值没变就不重复下发。 */
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

/* ============================================================================
 * IO 映像
 * ========================================================================== */

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
