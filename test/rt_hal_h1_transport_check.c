/* 真实的 HAL core + RT 适配层 + 内存 PDO 后端；绝不链接硬件主站。 */
#include "../src/hal/rt_hal_adapter.c"
#include "../src/hal/rt_hal_sim_backend.c"
#include <stdlib.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);abort();} }while(0)
static void begin(void) {
    CHECK(hal_rt_wait_cycle(s_ctx)==HAL_OK);
    CHECK(hal_rt_begin_cycle(s_ctx)==HAL_OK);publish_snapshot();
}
static void commit(void) { take_requests();CHECK(hal_rt_commit_cycle(s_ctx)==HAL_OK); }
static void frames(int n) { while(n--){commit();begin();} }
static HalAxisSnapshot state(void) {
    HalAxisSnapshot a;CHECK(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&a)==HAL_OK);return a;
}
static void prepared(HalAxisMode mode) {
    CHECK(rt_hal_request_axis_mode(RT_HAL_AXIS_SP,mode)==HAL_OK);frames(6);
    CHECK(state().mode_setting_state==HAL_MODE_SETTING_READY);
    CHECK(state().target_mode==mode);
    CHECK(state().drive_state!=HAL_DRIVE_OPERATION_ENABLED);
}
static void enable(void) {
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,1)==HAL_OK);frames(5);
    CHECK(state().drive_state==HAL_DRIVE_OPERATION_ENABLED);
}
int main(void) {
    RtHalAxisParams params[RT_HAL_AXIS_COUNT]={0};
    for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
        params[i].loaded=1;params[i].max_speed=10000;
        params[i].scaling.units_per_count=.01;
        params[i].scaling.feedback_pulses_per_rev=10000;
        params[i].scaling.command_pulses_per_rev=10000;
    }
    CHECK(rt_hal_create(params,NULL,0)==HAL_OK);
    CHECK(rt_hal_start(NULL,0)==HAL_OK);begin();
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,1)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_ENABLE);
    CHECK(state().request_rejection.code==HAL_ERROR_MODE_MISMATCH);
    CHECK(!s_req_enable_dirty[RT_HAL_AXIS_SP]);frames(3);
    CHECK(state().drive_state!=HAL_DRIVE_OPERATION_ENABLED);
    prepared(HAL_AXIS_MODE_CSV);
    CHECK(state().request_rejection.category==HAL_ISSUE_ENABLE);enable();
    CHECK(!state().request_rejection.code);
    CHECK(rt_hal_request_spindle_speed(RT_HAL_AXIS_SP,NAN,1)==HAL_ERROR_ARGUMENT);
    CHECK(rt_hal_request_axis_mode(RT_HAL_AXIS_SP,HAL_AXIS_MODE_CSV)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_SPINDLE_SPEED);
    CHECK(state().request_rejection.code==HAL_ERROR_ARGUMENT);frames(1);
    CHECK(state().request_rejection.code==HAL_ERROR_ARGUMENT); /* reads / empty HAL record do not clear */
    CHECK(rt_hal_request_spindle_speed(RT_HAL_AXIS_SP,100,1)==HAL_OK);frames(1);
    CHECK(!state().request_rejection.code && state().command_speed_rpm==100);
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,0)==HAL_OK);
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,1)==HAL_ERROR_BUSY);frames(5);
    CHECK(state().drive_state==HAL_DRIVE_READY);
    CHECK(state().mode_setting_state==HAL_MODE_SETTING_READY);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].target_speed==0);enable();
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,1)==HAL_OK);
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,0)==HAL_OK);frames(5);
    CHECK(state().drive_state==HAL_DRIVE_READY);enable();
    CHECK(rt_hal_request_axis_mode(RT_HAL_AXIS_SP,HAL_AXIS_MODE_CSP)==HAL_OK);frames(1);
    CHECK(rt_hal_request_axis_mode(RT_HAL_AXIS_SP,HAL_AXIS_MODE_CSV)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_MODE && state().request_rejection.code==HAL_ERROR_BUSY);
    frames(8);CHECK(state().target_mode==HAL_AXIS_MODE_CSP);
    CHECK(state().actual_mode==HAL_AXIS_MODE_CSP && state().drive_state==HAL_DRIVE_READY);
    CHECK(!s_req_mode_dirty[RT_HAL_AXIS_SP]);
    prepared(HAL_AXIS_MODE_CSV);enable();
    CHECK(rt_hal_spindle_request_orient(RT_HAL_AXIS_SP)==HAL_OK);frames(30);
    CHECK(state().control_state==HAL_CONTROL_ORIENT_HOLD);
    HalOrientExitTarget t={HAL_ORIENT_EXIT_SPINDLE_SPEED,0,0};
    CHECK(rt_hal_spindle_exit_orient(RT_HAL_AXIS_SP,&t)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_EXIT_ORIENT && state().request_rejection.code==HAL_ERROR_UNSUPPORTED);
    CHECK(state().control_state==HAL_CONTROL_ORIENT_HOLD);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].control_word==RT_PLACEHOLDER_ORIENT_CW);
    frames(3);CHECK(state().control_state==HAL_CONTROL_ORIENT_HOLD);
    CHECK(rt_hal_spindle_request_orient(RT_HAL_AXIS_SP)==HAL_OK);
    CHECK(rt_hal_request_estop(RT_HAL_AXIS_SP)==HAL_OK);frames(4);
    CHECK(!s_req_orient_dirty[RT_HAL_AXIS_SP]);
    CHECK(state().mode_setting_state==HAL_MODE_SETTING_UNSET);
    CHECK(state().control_state!=HAL_CONTROL_ORIENTING && state().control_state!=HAL_CONTROL_ORIENT_HOLD);
    frames(5);CHECK(state().drive_state!=HAL_DRIVE_OPERATION_ENABLED);
    HalAxisSnapshot unchanged=state(),out=unchanged;
    CHECK(hal_context_request_stop(s_ctx)==HAL_OK);
    /* 真实循环退出后适配层反馈即作废；诊断值不能当作完成证据。 */
    s_snap_valid=0;
    CHECK(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&out)==HAL_ERROR_NOT_RUNNING);
    CHECK(!memcmp(&out,&unchanged,sizeof(out)));
    CHECK(isnan(rt_hal_spindle_rpm()));
    rt_hal_shutdown();
    printf("HAL H1 transport: %u checks passed\n",checks);
    return 0;
}
