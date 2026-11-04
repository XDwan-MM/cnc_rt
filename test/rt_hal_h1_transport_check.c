/* 真实的 HAL core + RT 适配层 + 内存 PDO 后端；绝不链接硬件主站。 */
#include "../src/hal/rt_hal_adapter.c"
#include "../src/hal/rt_hal_sim_backend.c"
#include <stdlib.h>
#include <float.h>
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
    int reset_failed=-1;
    CHECK(rt_hal_fault_reset_settled_mask(0,&reset_failed) && !reset_failed);
    CHECK(!rt_hal_fault_reset_settled_mask(1u<<RT_HAL_AXIS_SP,&reset_failed));
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
    int32_t set_result=-1;
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_ERROR_STATE);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,0)==HAL_OK);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_ERROR_BUSY);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,1)==HAL_ERROR_BUSY);
    frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_OK);
    CHECK(state().coordinate_offset==0);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,DBL_MAX)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,-DBL_MAX)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_ERROR_ARGUMENT);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,0)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_OK);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,3)==HAL_OK);
    CHECK(rt_hal_request_axis_enable(RT_HAL_AXIS_SP,0)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_ERROR_STOPPED);
    CHECK(state().coordinate_offset==0);frames(5);enable();
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,3)==HAL_OK);
    CHECK(rt_hal_request_fault_reset(RT_HAL_AXIS_SP)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_ERROR_STOPPED);
    frames(6);
    CHECK(rt_hal_fault_reset_settled_mask(1u<<RT_HAL_AXIS_SP,&reset_failed) && !reset_failed);
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
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,3)==HAL_OK);
    CHECK(rt_hal_request_estop(RT_HAL_AXIS_SP)==HAL_OK);frames(1);
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_OK && set_result==HAL_ERROR_STOPPED);
    HalAxisSnapshot unchanged=state(),out=unchanged;
    CHECK(hal_context_request_stop(s_ctx)==HAL_OK);
    /* 真实循环退出后适配层反馈即作废；诊断值不能当作完成证据。 */
    s_snap_valid=0;
    CHECK(rt_hal_axis_set_pos_result(RT_HAL_AXIS_SP,&set_result)==HAL_ERROR_NOT_RUNNING);
    CHECK(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&out)==HAL_ERROR_NOT_RUNNING);
    CHECK(!memcmp(&out,&unchanged,sizeof(out)));
    CHECK(isnan(rt_hal_spindle_rpm()));
    rt_hal_shutdown();
    /* 适配层 CSP 退出：只保持现位置，退出期间不能插入普通运动。 */
    setenv("CNC_RT_SIM_GREE4","1",1);
    CHECK(rt_hal_create(params,NULL,0)==HAL_OK);
    CHECK(rt_hal_start(NULL,0)==HAL_OK);begin();
    prepared(HAL_AXIS_MODE_CSP);enable();
    CHECK(rt_hal_request_axis_pos(RT_HAL_AXIS_SP,7)==HAL_OK);frames(1);
    CHECK(rt_hal_spindle_request_orient(RT_HAL_AXIS_SP)==HAL_OK);frames(30);
    CHECK(state().control_state==HAL_CONTROL_ORIENT_HOLD);
    const uint32_t held_raw=s_servo[SIM_SLAVE_SPINDLE].actual_pos;
    t=(HalOrientExitTarget){HAL_ORIENT_EXIT_POSITION,0,state().actual_pos+1};
    CHECK(rt_hal_spindle_exit_orient(RT_HAL_AXIS_SP,&t)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_EXIT_ORIENT &&
          state().request_rejection.code==HAL_ERROR_ARGUMENT);
    CHECK(state().control_state==HAL_CONTROL_ORIENT_HOLD);
    t.value=state().actual_pos;
    CHECK(rt_hal_spindle_exit_orient(RT_HAL_AXIS_SP,&t)==HAL_OK);frames(1);
    CHECK(state().control_state==HAL_CONTROL_EXITING_ORIENT);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].target_pos==held_raw);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].control_word==0x0f);
    CHECK(rt_hal_request_axis_pos(RT_HAL_AXIS_SP,8)==HAL_OK);frames(1);
    CHECK(state().request_rejection.category==HAL_ISSUE_POSITION &&
          state().request_rejection.code==HAL_ERROR_BUSY);
    CHECK(rt_hal_axis_set_pos(RT_HAL_AXIS_SP,0)==HAL_OK);frames(1);
    frames(15);
    CHECK(state().control_state==HAL_CONTROL_IDLE);
    CHECK(state().drive_state==HAL_DRIVE_OPERATION_ENABLED);
    CHECK(state().actual_mode==HAL_AXIS_MODE_CSP);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].target_pos==held_raw);
    CHECK(fabs(state().actual_pos)<1e-6 && fabs(state().command_pos)<1e-6);
    rt_hal_shutdown();unsetenv("CNC_RT_SIM_GREE4");

    /* 模拟收帧故障：真实 PDO 循环应闭锁、作废快照并清除待下发请求。 */
    CHECK(rt_hal_create(params,NULL,0)==HAL_OK);
    CHECK(rt_hal_start(NULL,0)==HAL_OK);
    s_snap_valid=s_io_ready=1;
    s_snap_fresh[RT_HAL_AXIS_SP]=1;
    s_req_enable_dirty[RT_HAL_AXIS_SP]=1;
    s_opened=0; /* 模拟驱动层收帧失败 */
    CHECK(rt_hal_pdo_loop()==HAL_ERROR_BUS);
    CHECK(__atomic_load_n(&op_flag,__ATOMIC_ACQUIRE)==0);
    CHECK(rt_hal_cycle_error()==HAL_ERROR_BUS);
    CHECK(!s_snap_valid && !s_io_ready && !s_snap_fresh[RT_HAL_AXIS_SP]);
    CHECK(!s_req_enable_dirty[RT_HAL_AXIS_SP]);
    CHECK(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&out)==HAL_ERROR_NOT_RUNNING);
    CHECK(rt_hal_lost_slave()==-1);
    CHECK(hal_rt_wait_cycle(s_ctx)==HAL_ERROR_BUS); /* 持续闭锁 */
    rt_hal_shutdown();
    printf("HAL H1 transport: %u checks passed\n",checks);
    return 0;
}
