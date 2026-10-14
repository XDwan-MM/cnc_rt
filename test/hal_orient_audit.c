// 定向完成位审计：复用隔离 HAL 总线夹具（HAL_CONTEXT_TEST），不接机器 SDK。
#define main hal_regression_unused_main
#include HAL_CONTEXT_TEST
#undef main

int main(void)
{
    HalCConfig cfg = config();
    cfg.spindles[0].axis.vendor_orient_delay_ms = 10;
    HalContext *c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == HAL_OK);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == HAL_OK);
    for (int i = 0; i < 5; ++i) cycle(c);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437;
    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 3) == HAL_OK);
    CHECK(hal_rt_commit_cycle(c) == HAL_OK);
    for (int i = 0; i < 30; ++i) {
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437;
        cycle(c);
    }
    int32_t state = HAL_ORIENT_NONE;
    CHECK(hal_rt_spindle_orient_state(c, 3, &state) == HAL_OK);
    CHECK(state == HAL_ORIENT_RUNNING);
    puts("CURRENT HAL: bit10 high for 30 cycles -> still RUNNING");
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037;
    cycle(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &state) == HAL_OK);
    CHECK(state == HAL_ORIENT_RUNNING);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437;
    cycle(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &state) == HAL_OK);
    CHECK(state == HAL_ORIENT_DONE);
    puts("CURRENT HAL: subsequent low -> high -> DONE");
    hal_context_destroy(c);
    puts("PASS: reference and HAL reject stale high and require a fresh low -> high");
    return 0;
}
