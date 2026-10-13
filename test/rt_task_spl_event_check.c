#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

/* 走真实事件入口 task_handle_spl_event()：验证"定向/M29 期间的停止事件仍要取消
 * 排队中的 M3/M4"。只保留这条链：LOG 关掉，其余外部依赖给桩，不接共享内存。 */
#undef LOG
#define LOG(...) ((void)0)
#include "../src/task/task.c"

spindle_context_t spl_context;
static unsigned clear_calls;
void spindle_clear_velocity_arrival(void) { ++clear_calls; }

static PLC_EVENT ev;

static void prepare_stop_event(void)
{
    memset(&spl_context, 0, sizeof(spl_context));
    memset(&ev, 0, sizeof(ev));
    ev.spindle_payload.direction = 0;   /* 0 = 停止 */
    clear_calls = 0;
}

/* 1) 定向运行中：排队 CCW500 + 真实停止事件（复查复现的时序） */
static void test_stop_during_orient_cancels_deferred(void)
{
    prepare_stop_event();
    spl_context.orient_phase = SPINDLE_ORIENT_RUNNING;
    spl_context.m19_active = true;
    spl_context.command_state = SPINDLE_CW;
    spl_context.command_speed = 1000.0;
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CCW;
    spl_context.deferred_start_speed = 500.0;

    task_handle_spl_event(&ev);

    assert(clear_calls == 1);
    assert(!spl_context.deferred_start_pending);        /* 关键：提前返回也要清 */
    assert(!spl_context.start_request_pending);
    assert(spl_context.command_speed == 0.0);
    assert(spl_context.command_state == SPINDLE_STOP);
    assert(spl_context.spl_event != SPINDLE_EVENT_STOP); /* 仍不打断定向 */
    puts("PASS: 定向运行中收到停止事件 -> 取消排队启动且不打断定向");
}

/* 2) 定向释放中（还有待处理释放请求）同样要清 */
static void test_stop_during_release_cancels_deferred(void)
{
    prepare_stop_event();
    spl_context.orient_phase = SPINDLE_ORIENT_RELEASING;
    spl_context.orient_release_pending = true;
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 500.0;

    task_handle_spl_event(&ev);

    assert(!spl_context.deferred_start_pending);
    assert(spl_context.spl_event != SPINDLE_EVENT_STOP);
    puts("PASS: 定向释放中收到停止事件 -> 取消排队启动且不打断释放");
}

/* 3) 定向请求还在事件槽里（G29.6 下降沿跟着 G70.6 上升沿） */
static void test_stop_does_not_override_pending_orient(void)
{
    prepare_stop_event();
    spl_context.spl_event = SPINDLE_EVENT_ORIENT;
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CCW;
    spl_context.deferred_start_speed = 500.0;

    task_handle_spl_event(&ev);

    assert(spl_context.spl_event == SPINDLE_EVENT_ORIENT);
    assert(!spl_context.deferred_start_pending);
    puts("PASS: 待处理定向请求不被停止边沿覆盖，排队启动仍被取消");
}

/* 4) M29 进行中：不打断，但排队启动同样取消 */
static void test_stop_during_m29_cancels_deferred(void)
{
    prepare_stop_event();
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_ORIENTING;
    spl_context.deferred_start_pending = true;
    spl_context.deferred_start_state = SPINDLE_CW;
    spl_context.deferred_start_speed = 500.0;

    task_handle_spl_event(&ev);

    assert(!spl_context.deferred_start_pending);
    assert(spl_context.spl_event != SPINDLE_EVENT_STOP);
    puts("PASS: M29 进行中收到停止事件 -> 取消排队启动且不打断 M29");
}

/* 5) 普通停止（无定向/M29）：照旧产生停止事件并归位命令字段 */
static void test_plain_stop_still_creates_event(void)
{
    prepare_stop_event();
    spl_context.command_state = SPINDLE_CW;
    spl_context.command_speed = 1000.0;
    spl_context.start_request_pending = true;
    spl_context.deferred_start_pending = true;

    task_handle_spl_event(&ev);

    assert(spl_context.spl_event == SPINDLE_EVENT_STOP);
    assert(spl_context.command_state == SPINDLE_STOP);
    assert(spl_context.command_speed == 0.0);
    assert(!spl_context.start_request_pending);
    assert(!spl_context.deferred_start_pending);
    puts("PASS: 普通停止事件仍照旧产生 SPINDLE_EVENT_STOP");
}

int main(void)
{
    test_stop_during_orient_cancels_deferred();
    test_stop_during_release_cancels_deferred();
    test_stop_does_not_override_pending_orient();
    test_stop_during_m29_cancels_deferred();
    test_plain_stop_still_creates_event();
    puts("PASS: task 主轴事件入口（停止取消排队启动）");
    return 0;
}