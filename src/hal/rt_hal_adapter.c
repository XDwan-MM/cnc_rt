/* 适配层实现：cnc_rt 与 cnc_hal 之间的唯一通道。
 *
 * 三条不变式（改本文件前先确认没有破坏它们）：
 *   1. 只有 PDO 线程调用 HAL 的周期接口（wait / begin / commit 与请求提交）；
 *   2. 运动线程只写请求暂存区、只读快照，绝不直接调用 HAL；
 *   3. 适配层不推导 DS402 状态，只做槽号映射、按拍暂存与错误翻译。
 *
 * 一拍的数据流：
 *   begin → publish_snapshot() 填快照 → 放行 PLC/运动线程（sem_post）
 *   → 等两个完成屏障 → take_requests() 把暂存区交给 HAL
 *   → flush Y 映像 → commit（本拍唯一的发送点）。
 */
#define _POSIX_C_SOURCE 200809L
#include "rt_hal_adapter.h"
#include "rt_sem.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static HalContext* s_ctx;
static int s_running, s_snap_valid, s_in_cycle;
static uint32_t s_cycle_timeout_ms, s_x_size, s_y_size;
static int32_t s_cycle_error;
static HalAxisSnapshot s_snap[RT_HAL_AXIS_COUNT];
static uint8_t s_snap_fresh[RT_HAL_AXIS_COUNT];
/* 拒绝记录的归属：由 PDO 线程持有并写进快照。
 * 运动线程只在自己的屏障处把本地拒绝交接过来（s_local_rejection_dirty）。 */
static HalAxisIssue s_rejection[RT_HAL_AXIS_COUNT], s_local_rejection[RT_HAL_AXIS_COUNT];
static uint8_t s_local_rejection_dirty[RT_HAL_AXIS_COUNT];
static double s_req_pos[RT_HAL_AXIS_COUNT], s_req_setpos[RT_HAL_AXIS_COUNT], s_req_speed[RT_HAL_AXIS_COUNT];
static int s_req_speed_dir[RT_HAL_AXIS_COUNT];
static uint8_t s_req_pos_dirty[RT_HAL_AXIS_COUNT], s_req_setpos_dirty[RT_HAL_AXIS_COUNT], s_req_speed_dirty[RT_HAL_AXIS_COUNT];
static uint8_t s_req_estop[RT_HAL_AXIS_COUNT], s_req_reset[RT_HAL_AXIS_COUNT];
static uint8_t s_req_enable_dirty[RT_HAL_AXIS_COUNT], s_req_mode_dirty[RT_HAL_AXIS_COUNT];
static uint8_t s_req_orient_dirty[RT_HAL_AXIS_COUNT], s_req_exit_dirty[RT_HAL_AXIS_COUNT], s_req_clear[RT_HAL_AXIS_COUNT];
static int s_req_enable[RT_HAL_AXIS_COUNT];
static HalAxisMode s_req_mode[RT_HAL_AXIS_COUNT];
static HalOrientExitTarget s_req_exit[RT_HAL_AXIS_COUNT];
static uint8_t s_x_image[RT_HAL_IO_IMAGE_MAX], s_y_image[RT_HAL_IO_IMAGE_MAX];
static int s_io_ready;
static uint8_t s_stop_image[RT_HAL_IO_IMAGE_MAX];
static int s_stop_image_valid;

static int slot_ok(int slot) { return slot>=0 && slot<RT_HAL_AXIS_COUNT; }
static void log_hs(const char* what,int slot,int32_t rc) {
    char text[96];hal_error_text(rc,text,sizeof(text));
    fprintf(stderr,"[HAL] %s slot=%d rc=%d (%s)\n",what,slot,rc,text);
}
static int local_reply(RtHalAxisSlot slot,int category,int rc) {
    if(rc && slot_ok(slot) && s_ctx && rt_hal_config_hal_axis(slot)>=0) {
        s_local_rejection[slot]=(HalAxisIssue){category,rc};
        s_local_rejection_dirty[slot]=1;
    }
    return rc;
}
static int admission(RtHalAxisSlot slot,int category,int motion) {
    int rc=!slot_ok(slot)||rt_hal_config_hal_axis(slot)<0?HAL_ERROR_ARGUMENT:
        (!s_ctx||!s_running?HAL_ERROR_NOT_RUNNING:
        (motion&&!rt_hal_config_motion_allowed()?HAL_ERROR_NOT_RUNNING:HAL_OK));
    return local_reply(slot,category,rc);
}
static void submitted(int slot,int category,int rc) {
    if(rc) s_rejection[slot]=(HalAxisIssue){category,rc};
    else if(s_rejection[slot].category==category)s_rejection[slot]=(HalAxisIssue){0,0};
}
static void clear_axis_requests(int i) {
    s_req_pos_dirty[i]=s_req_setpos_dirty[i]=s_req_speed_dirty[i]=0;
    s_req_estop[i]=s_req_reset[i]=s_req_enable_dirty[i]=s_req_mode_dirty[i]=0;
    s_req_orient_dirty[i]=s_req_exit_dirty[i]=s_req_clear[i]=0;
}
static void reset_transport(void) {
    s_snap_valid=s_io_ready=s_in_cycle=0;s_cycle_error=0;
    memset(s_snap_fresh,0,sizeof(s_snap_fresh));
    memset(s_snap,0,sizeof(s_snap));
    memset(s_rejection,0,sizeof(s_rejection));
    memset(s_local_rejection_dirty,0,sizeof(s_local_rejection_dirty));
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i)clear_axis_requests(i);
}
int rt_hal_create(const RtHalAxisParams* params, char* err, uint32_t err_len) {
    if (!params) return RT_HAL_ERR_NULL_ARG;
    if (s_ctx) return HAL_ERROR_BUSY;   /* 已经有实例了：当前只支持单实例，拒绝重复 create */

    if (err && err_len) err[0] = '\0';

    /* 先按台架 profile 装配一份占位配置。
         * 缺参数的轴就停在这个占位值上，不会拿 0 或猜测值去驱动。 */
    HalCConfig cfg;
    int rc = rt_hal_config_build(&cfg, err, err_len);
    if (rc) return rc;

    /* 逐轴用参数域的真实值替换占位换算量。
         * 缺参数的轴不调用 set_scaling：保留占位值，同时由未确认台账把运动挡住。
         * 这样"参数没确认"不会退化成"轴按错的当量悄悄跑"。 */
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        if (!params[i].loaded) continue;
        rc = rt_hal_config_set_scaling(&cfg, (RtHalAxisSlot)i, &params[i].scaling);
        if (rc) {
            if (err && err_len)
                snprintf(err, err_len, "第 %d 根轴的机床参数非法（当量/enc_off/每转计数）", i);
            return rc;
        }
    }

    /* 主轴转速上限。缺参数时保留占位上限，并在启动打印里明确提示——
         * 超过占位上限的转速会被 HAL 直接拒收，不会静默钳位。 */
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

    /* 静态校验并打印装配结果。这一步不碰硬件，
         * 所以没有台架时也能先确认配置对不对。 */
    rc = rt_hal_config_finish(&cfg, err, err_len);
    if (rc) return rc;

    s_cycle_timeout_ms = cfg.cycle_timeout_ms;

    /* 配置到这里才算定稿，创建 context（仍然不碰硬件） */
    HalContext* ctx = NULL;
    rc = hal_context_create(&cfg, &ctx, err, err_len);
    if (rc) { log_hs("hal_context_create", -1, rc); return rc; }
    if (!ctx) {
        if (err && err_len) snprintf(err, err_len, "hal_context_create 返回成功但句柄为空");
        return HAL_ERROR_STATE;
    }

    s_ctx = ctx;
    reset_transport();
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

    static const char* const names[RT_HAL_AXIS_COUNT] = {"X", "Y", "Z", "SP"};
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        HalCCapability capability;
        const int axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (axis >= 0 &&
            hal_axis_capability(s_ctx, (HalAxisId)axis, HAL_FUNC_POSITION, &capability) == HAL_OK)
            printf("[HAL] 生效轴绑定 %s: 从站 %d 轴 %d\n", names[i],
                   capability.slave_pos, capability.axis_index);
    }

    /* 问一次 IO 映像段长并缓存。查不到不算致命：
         * 按 0 处理等价于本拍不下发 IO，比拿一个猜出来的长度去拷贝安全。 */
    s_x_size = s_y_size = 0;
    (void)hal_io_image_size(s_ctx, &s_x_size, &s_y_size);
    printf("映像段长（按设备 Entry 实算）：X=%u 字节, Y=%u 字节\n",
           (unsigned)s_x_size, (unsigned)s_y_size);
    /* 段长超过本层缓冲必须在这里拒绝启动。
         * 后果不是"IO 少几个点"，而是越界写：线程层是按 HAL 段长去写自己那份
         * 512 字节缓冲的。RT_HAL_IO_IMAGE_MAX 三处必须一致——
         * 本层缓冲、线程层缓冲、共享内存 REG_X / REG_Y 长度。 */
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
    /* emcmotTrajPlan() 用 op_flag 做总门控。
         * "主站已经进 OP"这件事只有这里知道，所以置位不能提前到
         * hal_context_start 返回之前。 */
    op_flag = 1;
    return HAL_OK;
}
void rt_hal_shutdown(void) {
    if (!s_ctx) return;

    op_flag = 0;   /* 先关总门控：避免还在跑的周期继续按旧状态下发指令 */

    /* 本函数不 join PDO 线程。调用方必须先让周期循环退出，
         * 否则这里会拆掉别人还在使用的 context。 */
    if (s_running) {
        (void)hal_context_request_stop(s_ctx);
        (void)hal_context_stop(s_ctx);
        s_running = 0;
    }
    hal_context_destroy(s_ctx);
    s_ctx = NULL;

    s_snap_valid = s_io_ready = 0;
    memset(s_snap_fresh, 0, sizeof(s_snap_fresh));

    s_in_cycle = 0;
    s_x_size = s_y_size = 0;
}
int rt_hal_running(void) { return s_running; }
int rt_hal_axis_sdo_read(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                         void* data, uint32_t size, uint32_t timeout_ms) {
    const int axis = rt_hal_config_hal_axis(slot);
    if (axis < 0) return HAL_ERROR_ARGUMENT;
    if (!s_ctx || !s_running) return HAL_ERROR_NOT_RUNNING;
    return hal_axis_sdo_read(s_ctx, (HalAxisId)axis, index, subindex, data, size, timeout_ms);
}
int rt_hal_axis_sdo_write(RtHalAxisSlot slot, uint16_t index, uint8_t subindex,
                          const void* data, uint32_t size, uint32_t timeout_ms) {
    const int axis = rt_hal_config_hal_axis(slot);
    if (axis < 0) return HAL_ERROR_ARGUMENT;
    if (!s_ctx || !s_running) return HAL_ERROR_NOT_RUNNING;
    return hal_axis_sdo_write(s_ctx, (HalAxisId)axis, index, subindex, data, size, timeout_ms);
}
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

static void publish_snapshot(void) {
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        s_snap_fresh[i]=0;
        const int id=rt_hal_config_hal_axis((RtHalAxisSlot)i);
        HalAxisSnapshot snap;
        if(id<0 || hal_rt_axis_read_snapshot(s_ctx,(HalAxisId)id,&snap)!=HAL_OK)continue;
        snap.request_rejection=s_rejection[i];
        s_snap[i]=snap;s_snap_fresh[i]=1;
    }
    s_snap_valid=1;
}
static void take_requests(void) {
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        int id=rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if(id<0)continue;
        if(s_local_rejection_dirty[i]) {
            s_rejection[i]=s_local_rejection[i];s_local_rejection_dirty[i]=0;
        }
        if(s_req_setpos_dirty[i]) {
            s_req_setpos_dirty[i]=0;
            submitted(i,HAL_ISSUE_SET_POSITION,hal_rt_axis_set_pos(s_ctx,id,s_req_setpos[i]));
        }
        if(s_req_estop[i]) {
            clear_axis_requests(i);
            submitted(i,HAL_ISSUE_ESTOP,hal_rt_axis_request_estop(s_ctx,id));
            continue;
        }
        if(s_req_reset[i]) {
            const int disable=s_req_enable_dirty[i]&&!s_req_enable[i];
            const int clear=s_req_clear[i];
            clear_axis_requests(i);
            if(clear)submitted(i,HAL_ISSUE_CLEAR_EXECUTION_ERROR,hal_rt_axis_clear_execution_error(s_ctx,id));
            submitted(i,HAL_ISSUE_FAULT_RESET,hal_rt_axis_request_fault_reset(s_ctx,id));
            if(disable)submitted(i,HAL_ISSUE_DISABLE,hal_rt_axis_request_disable(s_ctx,id));
            continue;
        }
        if(s_req_enable_dirty[i]&&!s_req_enable[i]) {
            s_req_enable_dirty[i]=0;
            submitted(i,HAL_ISSUE_DISABLE,hal_rt_axis_request_disable(s_ctx,id));
        }
        if(s_req_mode_dirty[i]) {
            s_req_mode_dirty[i]=0;
            submitted(i,HAL_ISSUE_MODE,hal_rt_axis_request_mode(s_ctx,id,s_req_mode[i]));
        }
        if(s_req_enable_dirty[i]) {
            s_req_enable_dirty[i]=0;
            submitted(i,HAL_ISSUE_ENABLE,hal_rt_axis_request_enable(s_ctx,id));
        }
        if(s_req_exit_dirty[i]) {
            s_req_exit_dirty[i]=s_req_orient_dirty[i]=0;
            submitted(i,HAL_ISSUE_EXIT_ORIENT,hal_rt_spindle_request_exit_orient(s_ctx,id,&s_req_exit[i]));
        } else if(s_req_orient_dirty[i]) {
            s_req_orient_dirty[i]=0;
            submitted(i,HAL_ISSUE_ORIENT,hal_rt_spindle_request_orient(s_ctx,id));
        }
        if(s_req_pos_dirty[i]) {
            s_req_pos_dirty[i]=0;
            submitted(i,HAL_ISSUE_POSITION,hal_rt_axis_write_pos(s_ctx,id,s_req_pos[i]));
        }
        if(s_req_speed_dirty[i]) {
            s_req_speed_dirty[i]=0;
            submitted(i,HAL_ISSUE_SPINDLE_SPEED,hal_rt_spindle_write_speed(s_ctx,id,s_req_speed[i],s_req_speed_dir[i]));
        }
        if(s_req_clear[i]) {
            s_req_clear[i]=0;
            submitted(i,HAL_ISSUE_CLEAR_EXECUTION_ERROR,hal_rt_axis_clear_execution_error(s_ctx,id));
        }
    }
}
static void make_deadline(struct timespec* ts) {
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec  += (time_t)(s_cycle_timeout_ms / 1000u);
    ts->tv_nsec += (long)(s_cycle_timeout_ms % 1000u) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) { ts->tv_sec += 1; ts->tv_nsec -= 1000000000L; }
}
static int32_t wait_done(const char* what, volatile uint32_t* done, sem_t* sem,
                         uint32_t cycle, const struct timespec* deadline) {
    while (__atomic_load_n(done, __ATOMIC_ACQUIRE) != cycle) {
        if (__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) return HAL_ERROR_STOPPED;
        if (sem_timedwait(sem, deadline) != 0) {
            if (errno == EINTR) continue;
            if (errno == ETIMEDOUT) {
                fprintf(stderr, "[HAL] 第 %u 拍共同等待超时（PLC/运动共享总预算 %u ms；"
                        "当前等待 %s；PLC完成拍=%u，运动完成拍=%u）\n",
                        cycle, s_cycle_timeout_ms, what,
                        __atomic_load_n(&plc_done_cycle, __ATOMIC_ACQUIRE),
                        __atomic_load_n(&motion_done_cycle, __ATOMIC_ACQUIRE));
                return HAL_ERROR_TIMEOUT;
            }
            return HAL_ERROR_STATE;
        }
    }
    return HAL_OK;
}
static void stop_frame(void) {
    if (!s_in_cycle) return;

    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        const int hal_axis = rt_hal_config_hal_axis((RtHalAxisSlot)i);
        if (hal_axis < 0) continue;
        const int rc=hal_rt_axis_request_estop(s_ctx, (HalAxisId)hal_axis);
        if(rc)log_hs("停机帧急停请求失败（原周期错误保留）",i,rc);
    }

    if (s_stop_image_valid) {
        const int rc=hal_rt_io_flush_outputs(s_ctx, s_stop_image, RT_HAL_IO_IMAGE_MAX);
        if(rc)log_hs("停机帧Y映像提交失败（原周期错误保留）",-1,rc);
    } else {
        /* 没有配置停机映像就不写 Y，保持上一次已下发的电平。
             * 这比写全 0 安全：写 0 可能松开夹具或抬起刀。 */
        fprintf(stderr, "[HAL] 停机帧：未配置停机输出映像，本帧不写 Y（计划书 §8）\n");
    }

    const int rc=hal_rt_commit_cycle(s_ctx); /* 只提交这一拍，不重试，也不用新错误码覆盖原始错误 */
    if(rc)log_hs("停机帧周期提交失败（原周期错误保留）",-1,rc);
    s_in_cycle = 0;
}
int rt_hal_pdo_loop(void) {
    if (!s_ctx || !s_running) return RT_HAL_ERR_ORDER;

    uint32_t cycle = 0;
    int32_t rc = HAL_OK;

    while (!__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        ++cycle;

        rc = hal_rt_wait_cycle(s_ctx);            /* 第一拍：等节拍并收帧 */
        if (rc) break;

        rc = hal_rt_begin_cycle(s_ctx);           /* 第二拍：采样、推进 DS402 状态机、准备输入映像 */
        if (rc) break;
        s_in_cycle = 1;

        publish_snapshot();
        s_io_ready = (hal_rt_io_snapshot_inputs(s_ctx, s_x_image, RT_HAL_IO_IMAGE_MAX) == HAL_OK);

        /* 先把周期号写出去，再 sem_post 放行业务线程。
             * 顺序反了，业务线程会读到上一拍的号；它收工时把旧号写回，
             * 屏障判定就永远对不上。 */
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
            /* IO 下发失败不单独中止本拍：同一笔总线错误会在 commit 那里再报一次，
                 * 由那里统一闭锁，避免同一故障被记两遍。 */
            s_io_ready = 0;
        }

        rc = hal_rt_commit_cycle(s_ctx);          /* 本拍唯一的发送出口 */
        s_in_cycle = 0;
        if (rc) break;


    }

    if (rc && !__atomic_load_n(&thread_exit_flag, __ATOMIC_ACQUIRE)) {
        /* 除了打日志，还要把错误码留下来：周期循环一退出，运动线程就卡在
             * sem_wait 上不再跑，只能靠 rt_hal_cycle_error() 把原因带回 RT 侧报警。 */
        s_cycle_error = rc;
        log_hs("周期循环", -1, rc);
    }
    s_snap_valid=s_io_ready=0;
    memset(s_snap_fresh,0,sizeof(s_snap_fresh));
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i)clear_axis_requests(i);
    return rc;
}

int32_t rt_hal_cycle_error(void) { return s_cycle_error; }
int rt_hal_axis_snapshot(RtHalAxisSlot slot,HalAxisSnapshot* out) {
    if(!slot_ok(slot)||!out)return HAL_ERROR_ARGUMENT;
    if(!s_snap_valid||!s_snap_fresh[slot])return HAL_ERROR_NOT_RUNNING;
    *out=s_snap[slot];return HAL_OK;
}
int rt_hal_axis_status(RtHalAxisSlot slot,RtHalAxisStatus* out) {
    if(!out)return HAL_ERROR_ARGUMENT;
    HalAxisSnapshot s;int rc=rt_hal_axis_snapshot(slot,&s);if(rc)return rc;
    if(!(s.feedback_valid&HAL_FEEDBACK_STATUS))return HAL_ERROR_NOT_RUNNING;
    RtHalAxisStatus v={0};
    v.enabled=s.drive_state==HAL_DRIVE_OPERATION_ENABLED;
    v.position_valid=!!(s.feedback_valid&HAL_FEEDBACK_POSITION);
    v.actual_pos=s.actual_pos;v.command_pos=(s.command_valid&HAL_COMMAND_POSITION)?s.command_pos:NAN;
    v.speed_valid=!!(s.feedback_valid&HAL_FEEDBACK_SPEED);
    v.actual_speed=s.actual_speed_device_units;v.actual_speed_rpm=s.actual_speed_rpm;
    v.raw_status=s.raw_status;v.error_code=s.drive_error_code;
    v.error_code_valid=!!(s.feedback_valid&HAL_FEEDBACK_ERROR_CODE);
    v.actual_speed_mm_min=s.actual_speed_mm_min;
    v.linear_speed_valid=!!(s.feedback_valid&HAL_FEEDBACK_LINEAR_SPEED);
    *out=v;return HAL_OK;
}
static int scalar(RtHalAxisSlot slot,double* out,uint32_t bit,int which) {
    if(!out)return HAL_ERROR_ARGUMENT;
    HalAxisSnapshot s;int rc=rt_hal_axis_snapshot(slot,&s);if(rc)return rc;
    if(!(s.feedback_valid&bit))return HAL_ERROR_UNSUPPORTED;
    *out=which==0?s.actual_pos:which==1?s.actual_speed_device_units:
        which==2?s.actual_speed_rpm:s.actual_speed_mm_min;
    return HAL_OK;
}
int rt_hal_axis_pos(RtHalAxisSlot s,double* out){return scalar(s,out,HAL_FEEDBACK_POSITION,0);}
int rt_hal_axis_speed(RtHalAxisSlot s,double* out){return scalar(s,out,HAL_FEEDBACK_SPEED,1);}
int rt_hal_axis_speed_rpm(RtHalAxisSlot s,double* out){return scalar(s,out,HAL_FEEDBACK_SPEED,2);}
int rt_hal_axis_speed_mm_min(RtHalAxisSlot s,double* out){return scalar(s,out,HAL_FEEDBACK_LINEAR_SPEED,3);}
int rt_hal_spindle_status(RtHalAxisSlot slot,RtHalSpindleStatus* out) {
    if(!out||!rt_hal_config_is_spindle(slot))return HAL_ERROR_ARGUMENT;
    HalAxisSnapshot s;int rc=rt_hal_axis_snapshot(slot,&s);if(rc)return rc;
    if(!(s.feedback_valid&HAL_FEEDBACK_STATUS))return HAL_ERROR_NOT_RUNNING;
    RtHalSpindleStatus v={0};
    v.enabled=s.drive_state==HAL_DRIVE_OPERATION_ENABLED;
    v.mode=(s.feedback_valid&HAL_FEEDBACK_MODE)?s.actual_mode:HAL_AXIS_MODE_UNSET;
    v.command_speed=(s.command_valid&HAL_COMMAND_SPINDLE_SPEED_RPM)?s.command_speed_rpm:NAN;
    v.actual_speed=(s.feedback_valid&HAL_FEEDBACK_SPEED)?s.actual_speed_rpm:NAN;
    v.position_deg=(s.feedback_valid&HAL_FEEDBACK_POSITION)?s.actual_pos:NAN;
    v.raw_status=s.raw_status;*out=v;return HAL_OK;
}
int rt_hal_spindle_speed(RtHalAxisSlot slot,double* out) {
    if(!rt_hal_config_is_spindle(slot))return HAL_ERROR_ARGUMENT;
    return rt_hal_axis_speed_rpm(slot,out);
}
double rt_hal_spindle_rpm(void) {
    double rpm;return rt_hal_spindle_speed(RT_HAL_AXIS_SP,&rpm)?NAN:rpm;
}
int rt_hal_fault_reset_state(RtHalAxisSlot slot,int32_t* out) {
    if(!out)return HAL_ERROR_ARGUMENT;
    HalAxisSnapshot s;int rc=rt_hal_axis_snapshot(slot,&s);if(rc)return rc;
    if(s.control_state==HAL_CONTROL_RESETTING_FAULT)*out=RT_HAL_RESET_PENDING;
    else if((s.execution_error.category==HAL_ISSUE_FAULT_RESET&&s.execution_error.code) ||
            (s.request_rejection.category==HAL_ISSUE_FAULT_RESET&&s.request_rejection.code))*out=RT_HAL_RESET_FAILED;
    else if((s.feedback_valid&HAL_FEEDBACK_STATUS)&&s.drive_state==HAL_DRIVE_READY)*out=RT_HAL_RESET_DONE;
    else *out=RT_HAL_RESET_NONE;
    return HAL_OK;
}
int rt_hal_fault_reset_settled(int* any_timeout) {
    int failed=0;
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        if(rt_hal_config_hal_axis((RtHalAxisSlot)i)<0)continue;
        int32_t state;
        if(s_req_reset[i]||rt_hal_fault_reset_state((RtHalAxisSlot)i,&state))return 0;
        if(state==RT_HAL_RESET_FAILED)failed=1;
        else if(state!=RT_HAL_RESET_DONE)return 0;
    }
    if(any_timeout)*any_timeout=failed;
    return 1;
}
int rt_hal_spindle_orient_state(RtHalAxisSlot slot,int32_t* out) {
    if(!out||!rt_hal_config_is_spindle(slot))return HAL_ERROR_ARGUMENT;
    HalAxisSnapshot s;int rc=rt_hal_axis_snapshot(slot,&s);if(rc)return rc;
    if(!(s.feedback_valid&HAL_FEEDBACK_STATUS))return HAL_ERROR_NOT_RUNNING;
    *out=s.control_state==HAL_CONTROL_ORIENTING?RT_HAL_ORIENT_RUNNING:
        s.control_state==HAL_CONTROL_ORIENT_HOLD?RT_HAL_ORIENT_DONE:
        (s.control_state==HAL_CONTROL_EXITING_ORIENT||s.control_state==HAL_CONTROL_ORIENT_HANDOVER_UNKNOWN)?
        RT_HAL_ORIENT_UNKNOWN:RT_HAL_ORIENT_NONE;
    return HAL_OK;
}

int rt_hal_request_axis_pos(RtHalAxisSlot slot,double pos) {
    int rc=admission(slot,HAL_ISSUE_POSITION,1);if(rc)return rc;
    if(!isfinite(pos))return local_reply(slot,HAL_ISSUE_POSITION,HAL_ERROR_ARGUMENT);
    s_req_pos[slot]=pos;s_req_pos_dirty[slot]=1;return HAL_OK;
}
int rt_hal_axis_set_pos(RtHalAxisSlot slot,double pos) {
    int rc=admission(slot,HAL_ISSUE_SET_POSITION,0);if(rc)return rc;
    if(!isfinite(pos))return local_reply(slot,HAL_ISSUE_SET_POSITION,HAL_ERROR_ARGUMENT);
    s_req_setpos[slot]=pos;s_req_setpos_dirty[slot]=1;return HAL_OK;
}
int rt_hal_request_spindle_speed(RtHalAxisSlot slot,double rpm,int dir) {
    int rc=admission(slot,HAL_ISSUE_SPINDLE_SPEED,1);if(rc)return rc;
    if(!rt_hal_config_is_spindle(slot)||!isfinite(rpm)||rpm<0||dir < -1||dir>1)
        return local_reply(slot,HAL_ISSUE_SPINDLE_SPEED,HAL_ERROR_ARGUMENT);
    s_req_speed[slot]=rpm;s_req_speed_dir[slot]=dir;s_req_speed_dirty[slot]=1;return HAL_OK;
}
int rt_hal_request_axis_mode(RtHalAxisSlot slot,HalAxisMode mode) {
    int rc=admission(slot,HAL_ISSUE_MODE,1);if(rc)return rc;
    if(mode!=HAL_AXIS_MODE_CSP&&mode!=HAL_AXIS_MODE_CSV)return local_reply(slot,HAL_ISSUE_MODE,HAL_ERROR_ARGUMENT);
    s_req_mode[slot]=mode;s_req_mode_dirty[slot]=1;return HAL_OK;
}
int rt_hal_request_spindle_mode(RtHalAxisSlot slot,HalAxisMode mode) {
    if(!rt_hal_config_is_spindle(slot))return local_reply(slot,HAL_ISSUE_MODE,HAL_ERROR_ARGUMENT);
    return rt_hal_request_axis_mode(slot,mode);
}
int rt_hal_request_axis_enable(RtHalAxisSlot slot,int on) {
    const int category=on?HAL_ISSUE_ENABLE:HAL_ISSUE_DISABLE;
    int rc=admission(slot,category,0);if(rc)return rc;
    if(on!=0&&on!=1)return local_reply(slot,category,HAL_ERROR_ARGUMENT);
    /* 同拍内的下使能不许被随后的上使能覆盖：
         * 下使能代表"停"，不能被后面的请求翻案。 */
    if(on&&s_req_enable_dirty[slot]&&!s_req_enable[slot])return local_reply(slot,category,HAL_ERROR_BUSY);
    s_req_enable[slot]=on;s_req_enable_dirty[slot]=1;return HAL_OK;
}
int rt_hal_request_estop(RtHalAxisSlot slot) {
    int rc=admission(slot,HAL_ISSUE_ESTOP,0);if(rc)return rc;
    s_req_estop[slot]=1;return HAL_OK;
}
int rt_hal_request_fault_reset(RtHalAxisSlot slot) {
    int rc=admission(slot,HAL_ISSUE_FAULT_RESET,0);if(rc)return rc;
    s_req_reset[slot]=1;return HAL_OK;
}
int rt_hal_clear_execution_error(RtHalAxisSlot slot) {
    int rc=admission(slot,HAL_ISSUE_CLEAR_EXECUTION_ERROR,0);if(rc)return rc;
    s_req_clear[slot]=1;return HAL_OK;
}
int rt_hal_spindle_request_orient(RtHalAxisSlot slot) {
    int rc=admission(slot,HAL_ISSUE_ORIENT,0);if(rc)return rc;
    if(!rt_hal_config_is_spindle(slot))return local_reply(slot,HAL_ISSUE_ORIENT,HAL_ERROR_ARGUMENT);
    s_req_orient_dirty[slot]=1;return HAL_OK;
}
int rt_hal_spindle_exit_orient(RtHalAxisSlot slot,const HalOrientExitTarget* target) {
    int rc=admission(slot,HAL_ISSUE_EXIT_ORIENT,0);if(rc)return rc;
    if(!target||!rt_hal_config_is_spindle(slot)||target->reserved||!isfinite(target->value))
        return local_reply(slot,HAL_ISSUE_EXIT_ORIENT,HAL_ERROR_ARGUMENT);
    s_req_exit[slot]=*target;s_req_exit_dirty[slot]=1;return HAL_OK;
}
int rt_hal_spindle_set_orient_pulses(RtHalAxisSlot slot,uint32_t pulses,uint32_t timeout_ms) {
    if(!slot_ok(slot))return HAL_ERROR_ARGUMENT;
    int id=rt_hal_config_hal_axis(slot);
    if(!s_ctx||!s_running)return HAL_ERROR_NOT_RUNNING;
    if(id<0)return HAL_ERROR_ARGUMENT;
    return hal_spindle_set_orient_pulses(s_ctx,(HalAxisId)id,pulses,timeout_ms);
}
int rt_hal_io_set_stop_image(const uint8_t* y, uint32_t len) {
    if (!y) return HAL_ERROR_ARGUMENT;
    if (len < RT_HAL_IO_IMAGE_MAX) return HAL_ERROR_ARGUMENT;
    memcpy(s_stop_image, y, RT_HAL_IO_IMAGE_MAX);
    s_stop_image_valid = 1;
    return HAL_OK;
}
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
