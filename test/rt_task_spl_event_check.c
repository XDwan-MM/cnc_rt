#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

/* 停止事件回归：编入 task.c 真实入口 task_handle_spl_event()，不接共享内存；验定向/M29 期间的停止事件仍取消排队 M3/M4。 */
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

/* 1) 定向运行中（排队 CCW500）*/
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
    assert(!spl_context.deferred_start_pending);        /* 提前返回路径也要清 */
    assert(!spl_context.start_request_pending);
    assert(spl_context.command_speed == 0.0);
    assert(spl_context.command_state == SPINDLE_STOP);
    assert(spl_context.spl_event != SPINDLE_EVENT_STOP); /* 仍不打断定向 */
    puts("PASS: 定向运行中收到停止事件 -> 取消排队启动且不打断定向");
}

/* 2) 定向释放中（有未处理释放请求）*/
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

/* 4) M29 进行中：不打断，取消排队启动 */
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

/* 5) 普通停止（无定向/M29）：照旧产生停止事件 */
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