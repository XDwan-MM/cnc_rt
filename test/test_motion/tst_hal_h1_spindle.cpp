#include "rt_drive_control.h"
#include "motion_csp_stop.h"
#ifdef RT_HAL_H1_TEST
#include <gtest/gtest.h>
#include "spindle_internal.h"
#include "rt_hal_adapter.h"
#include "alarm.h"
#include "emcmot.h"
#include "auto_mst.h"
#include <alarmapi.h>
#include <regapi.h>
#include "plc_f_signal_handler.h"
#include "test_env.h"
#include <cmath>
#include <cstring>
#include <type_traits>
extern "C" int h1_test_begin(void);
extern "C" void h1_test_request_admission_running(int);
extern "C" void h1_test_invalidate_snapshot(void);
extern "C" void h1_test_axis_position_valid(int,int);
extern "C" void h1_test_spindle_follow_error(double);
extern "C" void h1_test_spindle_feedback_delta(double,double);
extern "C" void h1_test_spindle_mode_feedback_valid(int);
extern "C" void h1_test_spindle_mode_failure(void);
extern "C" void h1_test_spindle_speed_failure(void);
extern "C" void h1_test_spindle_rejection(int,int);
extern "C" void rt_test_force_spindle_enable_wait(void);
extern "C" void h1_test_override_spindle_drive_state(int);
extern "C" int h1_test_commit(void);
extern "C" int h1_test_force_disable(void);
extern "C" int h1_test_spindle_position(uint32_t*,int);
namespace {
class HalH1SpindleTest : public ::testing::Test {
protected:
    PLAN_BLOCK* saved_block=nullptr;
    bool gree4_fixture=false;
    void SetUp() override {
        ASSERT_TRUE(testEnvReady());
        saved_block=cycle_tp->plan_block;
        rt_hal_shutdown();
        std::memset(rt_data->sys_state,0,sizeof(rt_data->sys_state));
        rt_data->nc_event=SYS_EVENT_NONE;
        reset_data_reset();
        cleanAllAlarm(ALARM_AX, ALARM_LEVEL_ALL);
        cleanAllAlarm(ALARM_ETHCAT_SV, ALARM_LEVEL_ALL);
        cycle_tp->exec_state=EXEC_IDLE;
        cycle_tp->exec_cmd={};
        update_pos_flag=1;
        for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
            rt_data->axis_data[i].status.is_disable=i!=RT_HAL_AXIS_SP;
            rt_data->axis_data[i].status.is_error=false;
            rt_data->axis_data[i].status.spindle_wants_torque=false;
        }
        rt_data->axis_data[AXIS_SP].max_feed_acc=10000;
        rt_data->axis_data[AXIS_SP].spdl_default_speed=0;
        alarm_reset_latch_clear();spindle_init();
        RtHalAxisParams params[RT_HAL_AXIS_COUNT]={};
        for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
            params[i].loaded=1;params[i].max_speed=10000;
            params[i].scaling.units_per_count=.01;
            params[i].scaling.feedback_pulses_per_rev=10000;
            params[i].scaling.command_pulses_per_rev=10000;
        }
        ASSERT_EQ(rt_hal_create(params,nullptr,0),HAL_OK);
        ASSERT_EQ(rt_hal_start(nullptr,0),HAL_OK);
        ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    void TearDown() override {
        cycle_tp->plan_block=saved_block;rt_hal_shutdown();
        if(gree4_fixture)unsetenv("CNC_RT_SIM_GREE4");
    }
    void useGree4() {
        rt_hal_shutdown();setenv("CNC_RT_SIM_GREE4","1",1);gree4_fixture=true;
        rt_data->axis_data[AXIS_SP].spdl_max_speed=10000;
        setVarChanDouble(0,VAR_CHAN_SPDL_OVERRIDE,100,0);
        RtHalAxisParams params[RT_HAL_AXIS_COUNT]={};
        for(int i=0;i<RT_HAL_AXIS_COUNT;++i) {
            params[i].loaded=1;params[i].max_speed=10000;
            params[i].scaling.units_per_count=.01;
            params[i].scaling.feedback_pulses_per_rev=10000;
            params[i].scaling.command_pulses_per_rev=10000;
        }
        ASSERT_EQ(rt_hal_create(params,nullptr,0),HAL_OK);
        ASSERT_EQ(rt_hal_start(nullptr,0),HAL_OK);
        ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    void fullFrame() {
        emcmotTrajPlan();alarm_poll_hal();nc_set_resetted();hal_axis_intent_poll();
        if(nc_reset->is_resetted) {
            const auto a=axis();
            EXPECT_TRUE(a.drive_state==HAL_DRIVE_READY || a.drive_state==HAL_DRIVE_SWITCH_ON_DISABLED);
            EXPECT_EQ(a.control_state,HAL_CONTROL_IDLE);
            EXPECT_FALSE(reset_stop_pending());
        }
        plc_f_signal_handler_cycle();
        ASSERT_EQ(h1_test_commit(),HAL_OK);ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    void fullResetSetup() {
        rt_data->current_mode=rt_data->target_mode=MODE_EDIT;
        setVarSysInt(VAR_SYS_CHAN_NUM,1,0);
    }
    void frames(int n) {
        while(n--) {
            if(rt_data->nc_event==SYS_EVENT_HMI_RESET) nc_set_resetting(HMI_RESET);
            reset_prepare_motion();
            spindle_control_update();motion_csp_stop_update();
            emcmot_test_spindle_position_period();
            alarm_poll_hal();nc_set_resetted();hal_axis_intent_poll();
            rt_data->nc_event=SYS_EVENT_NONE;
            ASSERT_EQ(h1_test_commit(),HAL_OK);
            ASSERT_EQ(h1_test_begin(),HAL_OK);
        }
    }
    HalAxisSnapshot axis() {
        HalAxisSnapshot a={};EXPECT_EQ(rt_hal_axis_snapshot(RT_HAL_AXIS_SP,&a),HAL_OK);return a;
    }
    void runZero() {
        ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(20);
        EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
        EXPECT_EQ(axis().mode_setting_state,HAL_MODE_SETTING_READY);
        EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    }
    void cancelPreparation(bool no_orient, spindle_m29_phase_t phase, bool estop) {
        if(no_orient) { runZero();ASSERT_TRUE(spindle_request_m29_no_orient()); }
        else ASSERT_TRUE(spindle_request_m29());
        for(int i=0;i<130 && g_spindle.m29_phase!=phase;++i)frames(1);
        ASSERT_EQ(g_spindle.m29_phase,phase);
        fullResetSetup();
        if(estop) {
            rt_data->sys_state[SYS_ESTOP_INPUT]=1;
            for(int i=0;i<5;++i)fullFrame();
            EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
            rt_data->sys_state[SYS_ESTOP_INPUT]=0;
        }
        rt_data->nc_event=SYS_EVENT_HMI_RESET;
        for(int i=0;i<40;++i)fullFrame();
        EXPECT_FALSE(nc_reset->is_resetting);
        EXPECT_FALSE(g_spindle.m29_active);EXPECT_FALSE(g_spindle.m19_active);
        EXPECT_FALSE(g_spindle.orient_request_pending);
        EXPECT_FALSE(g_spindle.start_request_pending);EXPECT_FALSE(g_spindle.run_authorized);
        EXPECT_FALSE(motion_spindle_position_active());EXPECT_FALSE(reset_stop_pending());
        for(int i=0;i<5;++i) {
            fullFrame();EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
        }
    }
    void orient() {
        spindle_begin_m19();spindle_request_orientation(true);frames(80);
        EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
    }
};
}
TEST_F(HalH1SpindleTest, M3S0PreparesModeAndExplicitlyEnables) { runZero(); }
TEST_F(HalH1SpindleTest, M19PreparesThenOrientsOnce) { orient(); }
TEST_F(HalH1SpindleTest, UnsupportedExitCannotStartDeferredRotation) {
    orient();
    g_spindle.deferred_start_pending=true;
    g_spindle.deferred_start_state=SPINDLE_CW;g_spindle.deferred_start_speed=200;
    spindle_request_orientation(false);frames(8);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
    EXPECT_EQ(axis().request_rejection.category,HAL_ISSUE_EXIT_ORIENT);
    EXPECT_EQ(axis().request_rejection.code,HAL_ERROR_UNSUPPORTED);
    EXPECT_FALSE(g_spindle.deferred_start_pending);
    EXPECT_FALSE(g_spindle.run_authorized);
}
/* 现场故障回归：复位后主轴尚未使能，M19 的定向请求还没提交给 HAL，
 * 下一条 M3 的 SF 已经让梯子撤销 G70.6 并释放定向。此时没有硬件动作可退，
 * 必须本地收口并执行排队的 M3，不能停在永远推进不了的 RELEASING。 */
TEST_F(HalH1SpindleTest, ReleaseBeforeSubmitFinalizesAndStartsM3) {
    useGree4();
    spindle_begin_m19();spindle_request_orientation(true);
    spindle_control_update();
    ASSERT_FALSE(g_spindle.orient_submitted);
    ASSERT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_WAIT_ZERO);
    ASSERT_NE(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
    ASSERT_NE(axis().control_state,HAL_CONTROL_ORIENTING);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,200));
    spindle_request_orientation(false);
    frames(80);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
    EXPECT_FALSE(g_spindle.orient_request_pending);
    EXPECT_TRUE(g_spindle.run_authorized);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    EXPECT_GT(axis().command_speed_rpm,0);
}
/* 现场故障回归（17:55 日志）：第三条 M19 紧跟 100 rpm 的 M3，定向还在
 * HAL_CONTROL_ORIENTING 时下一条 M4 的 SF 就要求退出。HAL 此时返回 BUSY
 * （未停稳），只能重试，不能当成失败报警 2100。 */
TEST_F(HalH1SpindleTest, ReleaseWhileOrientingRetriesInsteadOfFailing) {
    useGree4();
    spindle_begin_m19();spindle_request_orientation(true);
    for(int i=0;i<80 && axis().control_state!=HAL_CONTROL_ORIENTING;++i)frames(1);
    ASSERT_EQ(axis().control_state,HAL_CONTROL_ORIENTING);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,200));
    spindle_request_orientation(false);
    frames(2);
    /* 定向尚未到位：必须停在 RELEASING 重试，而不是报警并锁复位启动。 */
    EXPECT_FALSE(g_spindle.reset_requires_start);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_RELEASING);
    frames(120);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
    EXPECT_FALSE(g_spindle.reset_requires_start);
    EXPECT_TRUE(g_spindle.run_authorized);
    EXPECT_GT(axis().command_speed_rpm,0);
}
/* 现场 18:08 日志回归：退出请求已经暂存，HAL 在下一拍快照里带回
 * EXIT_ORIENT/BUSY（定向保持但未停稳）。必须退回重试，不能报 2100。 */
TEST_F(HalH1SpindleTest, AsyncBusyExitRejectionRetriesInsteadOfFailing) {
    useGree4();orient();
    g_spindle.orient_exit_requested=true;
    g_spindle.orient_exit_submitted=true;
    g_spindle.orient_phase=SPINDLE_ORIENT_RELEASING;
    h1_test_spindle_rejection(HAL_ISSUE_EXIT_ORIENT,HAL_ERROR_BUSY);
    spindle_control_update();
    EXPECT_FALSE(g_spindle.reset_requires_start);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_RELEASING);
    frames(60);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_IDLE);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
/* 现场 19:31 回归：使能请求被拒后，拒绝记录被同一拍后续的定向请求类别覆盖，
 * ENABLE_WAIT 看不到任何 ENABLE 拒绝，旧实现会永远等下去。这里强制构造该等待态，
 * 要求控制周期在若干拍后重新提交使能。 */
TEST_F(HalH1SpindleTest, EnableWaitWithoutVisibleRejectionRetries) {
    useGree4();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,200));
    frames(60);
    spindle_request_stop();
    frames(90);
    ASSERT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    /* 直接构造"业务要力矩、定向等待中、使能请求却没有任何可见结果"的状态：
     * 定向阶段会让驱动控制保留力矩需求，不会被 stop 收尾清掉。 */
    g_spindle.orient_phase=SPINDLE_ORIENT_WAIT_ZERO;
    g_spindle.orient_request_pending=true;
    rt_test_force_spindle_enable_wait();
    frames(60);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
/* 没有定向在途时收到释放：应直接收口，不能进入 RELEASING。 */
TEST_F(HalH1SpindleTest, ReleaseWithoutOrientationDoesNotStick) {
    runZero();
    ASSERT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    spindle_request_orientation(false);
    frames(3);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_IDLE);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
/* 未消费的 ORIENT 不得被手动方向事件覆盖，应转为定向释放后的排队启动。 */
TEST_F(HalH1SpindleTest, ManualDirectionDoesNotOverwriteQueuedOrientation) {
    spindle_begin_m19();spindle_request_orientation(true);
    ASSERT_EQ(g_spindle.pending_event,SPINDLE_EVENT_ORIENT);
    ASSERT_TRUE(spindle_request_manual_run(SPINDLE_CW));
    EXPECT_EQ(g_spindle.pending_event,SPINDLE_EVENT_ORIENT);
    EXPECT_TRUE(g_spindle.deferred_start_pending);
    EXPECT_EQ(g_spindle.deferred_start_state,SPINDLE_CW);
}
/* 状态不一致时的看门狗：非 IDLE 且没有任何在途动作，必须在有限拍内自愈。 */
TEST_F(HalH1SpindleTest, OrientationStallWatchdogRecovers) {
    runZero();
    g_spindle.orient_phase=SPINDLE_ORIENT_RELEASING;
    g_spindle.orient_request_pending=false;
    g_spindle.orient_submitted=false;
    g_spindle.orient_exit_submitted=false;
    g_spindle.orient_stall_cycles=0;
    frames(static_cast<int>(SPINDLE_ORIENT_STALL_CYCLES)+5);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_EQ(g_spindle.orient_stall_cycles,0U);
    EXPECT_TRUE(g_spindle.orient_hold_stop);
}
TEST_F(HalH1SpindleTest, CsvExitKeepsEnableAndWaitsForFeedback) {
    useGree4();orient();
    spindle_request_orientation(false);frames(5);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_EXITING_ORIENT);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(g_spindle.run_authorized);
    frames(25);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_IDLE);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_NEAR(axis().command_speed_rpm,0,1e-6);
}
TEST_F(HalH1SpindleTest, CsvExitThenM3UsesNewAuthorization) {
    useGree4();orient();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,200));
    spindle_request_orientation(false);frames(40);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_TRUE(g_spindle.run_authorized);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    EXPECT_GT(axis().command_speed_rpm,0);
}
/* 现场 18:27 回归：停车收口会同一拍暂存"写 0 速"和"撤力矩"。速度必须先于
 * 下使能提交，否则 0 速会被 stop_goal 拒成 NOT_ENABLED，再被当成速度业务故障
 * 报 139。这里跑完整的 M3 400→M5→停稳下使能，确认不产生启动锁和报警。 */
TEST_F(HalH1SpindleTest, M5StopDoesNotReportVelocityFailure) {
    useGree4();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,400));
    frames(80);
    ASSERT_GT(axis().command_speed_rpm,0);
    spindle_request_stop();
    frames(120);
    EXPECT_FALSE(g_spindle.reset_requires_start);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
/* 停车收口同一拍会同时暂存 0 速和下使能。速度先提交时 0 速被接受，
 * 再撤力矩；顺序相反会留下 SPINDLE_SPEED/NOT_ENABLED 拒绝记录。 */
TEST_F(HalH1SpindleTest, StopStagesZeroSpeedBeforeDisable) {
    useGree4();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,400));
    frames(80);
    spindle_request_stop();
    for(int i=0;i<160 && axis().drive_state==HAL_DRIVE_OPERATION_ENABLED;++i)frames(1);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(axis().request_rejection.category==HAL_ISSUE_SPINDLE_SPEED &&
                 axis().request_rejection.code==HAL_ERROR_NOT_ENABLED);
}
/* 停车还没完全收口时立刻来 M19：使能管理的 BUSY、定向请求的 NOT_ENABLED
 * 都必须按瞬态重试，不能粘连成失败。 */
TEST_F(HalH1SpindleTest, ImmediateM19AfterStopRecovers) {
    useGree4();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,400));
    frames(80);
    spindle_request_stop();
    frames(45);
    spindle_begin_m19();spindle_request_orientation(true);
    frames(160);
    EXPECT_FALSE(g_spindle.reset_requires_start);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
}
TEST_F(HalH1SpindleTest, RepeatedM19ExitsOldHoldBeforeNewOrientation) {
    useGree4();orient();
    spindle_begin_m19();spindle_request_orientation(true);frames(100);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_DONE);
    EXPECT_FALSE(g_spindle.reset_requires_start);
}
TEST_F(HalH1SpindleTest, M5AfterCsvExitStillDisables) {
    useGree4();orient();spindle_request_orientation(false);frames(30);
    ASSERT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    spindle_request_stop();frames(15);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, ManualStartAfterCsvExitConsumesNewRequest) {
    useGree4();orient();spindle_request_orientation(false);frames(30);
    g_spindle.remembered_program_speed=200;
    ASSERT_TRUE(spindle_request_manual_run(SPINDLE_CCW));frames(30);
    EXPECT_TRUE(g_spindle.run_authorized);
    EXPECT_LT(axis().command_speed_rpm,0);
}
TEST_F(HalH1SpindleTest, M29AfterM19CanExitOldHoldAndReorient) {
    useGree4();orient();
    ASSERT_TRUE(spindle_request_m29());frames(220);
    EXPECT_TRUE(spindle_m29_is_ready());
    EXPECT_TRUE(motion_spindle_position_active());
}
TEST_F(HalH1SpindleTest, ResetDuringCsvExitCannotReplayPendingM3) {
    useGree4();orient();fullResetSetup();
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,200));
    spindle_request_orientation(false);frames(3);
    ASSERT_EQ(axis().control_state,HAL_CONTROL_EXITING_ORIENT);
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_FALSE(g_spindle.start_request_pending);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, M29TrueModeChangeThenOffsetAlignment) {
    spindle_request_m29();frames(110);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSP);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    EXPECT_TRUE(g_spindle.m29_feedback_stable);
    EXPECT_NEAR(axis().coordinate_offset,g_spindle.m29_expected_offset,1e-6);
    EXPECT_TRUE(axis().command_valid&HAL_COMMAND_POSITION);
    rt_data->sys_state[SYS_ESTOP_INPUT]=1;
    EXPECT_FALSE(spindle_m29_is_ready()); /* raw stop input wins before the next control update */
}
TEST_F(HalH1SpindleTest, M29WithoutOrientationAlignsBeforeReady) {
    runZero();ASSERT_TRUE(spindle_request_m29_no_orient());frames(60);
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    EXPECT_TRUE(g_spindle.m29_feedback_stable);
    EXPECT_NEAR(axis().coordinate_offset,g_spindle.m29_expected_offset,1e-6);
}
TEST_F(HalH1SpindleTest, M29KeepsOrientationDuringSettling) {
    ASSERT_TRUE(spindle_request_m29());
    for(int i=0;i<100 && g_spindle.m29_phase!=SPINDLE_M29_ORIENT_SETTLING;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    frames(20);
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    frames(90);EXPECT_TRUE(spindle_m29_is_ready());
}
TEST_F(HalH1SpindleTest, M29MovingFeedbackCannotPassSettlingWindow) {
    ASSERT_TRUE(spindle_request_m29());
    for(int i=0;i<100 && g_spindle.m29_phase!=SPINDLE_M29_ORIENT_SETTLING;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    for(int i=0;i<70;++i) {
        h1_test_spindle_feedback_delta(i%2?.7:0,0);frames(1);
    }
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    for(int i=0;i<60;++i) { h1_test_spindle_feedback_delta(0,2);frames(1); }
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    frames(100);EXPECT_TRUE(spindle_m29_is_ready());
}
TEST_F(HalH1SpindleTest, M29RejectsPositionShiftAfterRezero) {
    ASSERT_TRUE(spindle_request_m29());
    for(int i=0;i<150 && g_spindle.m29_phase!=SPINDLE_M29_ALIGNING;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_ALIGNING);
    h1_test_spindle_feedback_delta(2,0);frames(1);
    EXPECT_TRUE(spindle_m29_has_error());
    EXPECT_FALSE(motion_spindle_position_active());
}
TEST_F(HalH1SpindleTest, M29RejectsPositionShiftBeforeRezero) {
    ASSERT_TRUE(spindle_request_m29());
    for(int i=0;i<150 && g_spindle.m29_phase!=SPINDLE_M29_SWITCHING_CSP;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_SWITCHING_CSP);
    const double offset=axis().coordinate_offset;
    for(int i=0;i<40 && !spindle_m29_has_error();++i) {
        h1_test_spindle_feedback_delta(2,0);frames(1);
    }
    EXPECT_TRUE(spindle_m29_has_error());
    EXPECT_DOUBLE_EQ(axis().coordinate_offset,offset);
    EXPECT_FALSE(motion_spindle_position_active());
}
TEST_F(HalH1SpindleTest, M29SettlingHasFiniteTimeout) {
    ASSERT_TRUE(spindle_request_m29());
    for(int i=0;i<100 && g_spindle.m29_phase!=SPINDLE_M29_ORIENT_SETTLING;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_ORIENT_SETTLING);
    g_spindle.orient_elapsed_cycles=9999;frames(1);
    EXPECT_TRUE(spindle_m29_has_error());
    EXPECT_FALSE(motion_spindle_position_active());
}
TEST_F(HalH1SpindleTest, RepeatedM29RequiresFreshSettling) {
    for(int repeat=0;repeat<3;++repeat) {
        ASSERT_TRUE(spindle_request_m29());frames(170);
        ASSERT_TRUE(spindle_m29_is_ready());
        EXPECT_TRUE(g_spindle.m29_orient_reference_valid);
        EXPECT_NEAR(axis().actual_pos,0,.011);
        spindle_complete_m29();
    }
}
TEST_F(HalH1SpindleTest, UnexpectedEnableLossDoesNotReplayOldEnable) {
    runZero();ASSERT_EQ(h1_test_force_disable(),0);frames(15);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_EQ(axis().execution_error.category,HAL_ISSUE_AXIS_CONDITION);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(20);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED); /* fresh business may explicitly recover */
}
TEST_F(HalH1SpindleTest, EstopCancelsSpindleBusinessAndDoesNotRestoreIt) {
    orient();rt_data->sys_state[SYS_ESTOP_INPUT]=1;frames(5);
    EXPECT_FALSE(g_spindle.m19_active);EXPECT_FALSE(g_spindle.orient_request_pending);
    EXPECT_FALSE(g_spindle.deferred_start_pending);
    rt_data->sys_state[SYS_ESTOP_INPUT]=0;frames(10);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(g_spindle.run_authorized);
}
TEST_F(HalH1SpindleTest, ResetAfterM29ReleasesTorque) {
    spindle_request_m29();frames(110);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    rt_data->nc_event=SYS_EVENT_HMI_RESET;nc_reset->is_resetting=1;frames(12);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(g_spindle.m29_active);
}

TEST_F(HalH1SpindleTest, TappingResetKeepsTorqueUntilInterpolationIsIdle) {
    spindle_request_m29();frames(110);ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    auto saved=cycle_tp->plan_block;
    std::remove_reference<decltype(*cycle_tp->plan_block)>::type block={};
    block.type=RIGID_TAP;block.plane=PLANE_XY;
    cycle_tp->plan_block=&block;cycle_tp->exec_cmd=DO_TAP_CMD;cycle_tp->exec_state=EXEC_RUNNING;
    nc_reset->reset_trigger=HMI_RESET;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;nc_reset->is_resetting=1;
    frames(5);
    EXPECT_TRUE(reset_stop_pending());
    EXPECT_TRUE(motion_csp_stop_pending());
    EXPECT_TRUE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    cycle_tp->exec_state=EXEC_IDLE;
    frames(15);
    EXPECT_FALSE(reset_stop_pending());
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    cycle_tp->plan_block=saved;
}

TEST_F(HalH1SpindleTest, M3AfterM29ExplicitlyPreparesCsvAndEnables) {
    spindle_request_m29();frames(110);ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(30);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_TRUE(g_spindle.run_authorized);
}
TEST_F(HalH1SpindleTest, M19AfterM29ExitsCspAndOrients) {
    useGree4();
    spindle_request_m29();frames(110);ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    spindle_complete_m29();
    spindle_begin_m19();
    spindle_request_orientation(true);
    frames(120);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    EXPECT_EQ(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
}
TEST_F(HalH1SpindleTest, M5AfterM29OwnershipEndsDoesNotReenable) {
    spindle_request_m29();frames(110);ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    g_spindle.m29_active=false; /* ordinary M5 is allowed after the tapping owner relinquishes control */
    spindle_request_stop();bool left_enabled=false;
    for(int n=0;n<25;++n) {
        frames(1);
        if(axis().drive_state!=HAL_DRIVE_OPERATION_ENABLED)left_enabled=true;
        if(left_enabled)EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    }
    EXPECT_TRUE(left_enabled);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
}

TEST_F(HalH1SpindleTest, NoOrientAlignmentKeepsPhysicalPositionAndIgnoresOldPositionRejection) {
    runZero();
    uint32_t raw=12500;ASSERT_EQ(h1_test_spindle_position(&raw,1),0);frames(1);
    ASSERT_EQ(rt_hal_request_axis_pos(RT_HAL_AXIS_SP,NAN),HAL_ERROR_ARGUMENT);frames(1);
    ASSERT_EQ(axis().request_rejection.category,HAL_ISSUE_POSITION);
    ASSERT_TRUE(spindle_request_m29_no_orient());frames(65);
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_READY);
    EXPECT_TRUE(g_spindle.m29_feedback_stable);
    EXPECT_NEAR(axis().coordinate_offset,-125,1e-6);
    EXPECT_NEAR(axis().actual_pos,0,1e-6);
    raw=0;ASSERT_EQ(h1_test_spindle_position(&raw,0),0);
    EXPECT_EQ(raw,12500u);
}

// 按生产的 运动/报警/意图/PLC 顺序驱动，而不只是跑主轴循环。
TEST_F(HalH1SpindleTest, FullResetCannotFinishBeforeDisableFeedback) {
    runZero();
    rt_data->current_mode=rt_data->target_mode=MODE_EDIT;
    setVarSysInt(VAR_SYS_CHAN_NUM,1,0);
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    emcmotTrajPlan();
    alarm_poll_hal();
    nc_set_resetted();
    hal_axis_intent_poll();
    // 适配层还没提交下使能，反馈仍是 OperationEnabled。
    ASSERT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(nc_reset->is_resetted);
    plc_f_signal_handler_cycle();
    EXPECT_TRUE(nc_reset->is_resetting);
}


TEST_F(HalH1SpindleTest, FullResetCompletesAfterFeedbackAndDoesNotRestart) {
    runZero();fullResetSetup();rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_FALSE(reset_spindle_torque_allowed(true) && g_spindle.run_authorized);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, RepeatedResetKeepsCsvDecelerationAndStopsOnce) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(reset_stop_pending());
    auto elapsed=g_spindle.csv_stop_cycles;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    EXPECT_TRUE(g_spindle.csv_stop_cycles>elapsed);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, FullResetAfterM29DoesNotDependOnReadyLatch) {
    spindle_request_m29();frames(110);fullResetSetup();
    ASSERT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSP);
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(g_spindle.m29_active);
}
TEST_F(HalH1SpindleTest, EstopPreemptsCsvResetAndAutomaticRecoveryIsRetained) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(reset_stop_pending());
    rt_data->sys_state[SYS_ESTOP_INPUT]=true;
    for(int i=0;i<5;++i)fullFrame();
    EXPECT_FALSE(reset_stop_pending());EXPECT_TRUE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    rt_data->sys_state[SYS_ESTOP_INPUT]=false;
    // 与现有 G8.4 释放链路（D66）发出的是同一个事件。
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, FullTappingResetRunsRealInterpolationToIdle) {
    spindle_request_m29();frames(110);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    fullResetSetup();rt_data->current_mode=rt_data->target_mode=MODE_MEM;
    queue_reset(plan_queue);
    PLAN_BLOCK block={};block.type=RIGID_TAP;block.mode=PMC_MODE_TAP;
    block.plane=PLANE_XY;block.start_vec.z=-1;block.end_pos.z=-10;
    block.length=10;block.max_acc=100;block.max_jerk=1000;
    block.max_vel=block.feed_vel=block.param_vel=1;
    block.rigid_data.screw_offset=1;
    queue_push(plan_queue,&block);cycle_tp->plan_block=queue_peak(plan_queue);
    cycle_tp->exec_state=EXEC_RUNNING;cycle_tp->running_stage=PROGRAM_RUNNING;
    cycle_tp->exec_cmd=DO_TAP_CMD;cycle_tp->ncu_cycle=.001;cycle_tp->velplan_mode=static_cast<decltype(cycle_tp->velplan_mode)>(0);
    for(int i=0;i<3;++i) {
        rt_data->axis_data[i].plmt=1000;rt_data->axis_data[i].nlmt=-1000;
        rt_data->axis_data[i].lmt2_enable=false;rt_data->axis_data[i].lmt3_enable=false;
        rt_data->axis_data[i].max_dynamic_lag=.1;rt_data->axis_data[i].max_static_lag=.1;
    }
    rt_data->chan_data[0].cmd_vel=1;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    bool moved=false,kept_enabled=false;
    for(int i=0;i<150;++i) {
        fullFrame();
        if(std::fabs(rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur)>.001)moved=true;
        if(cycle_tp->exec_state!=EXEC_IDLE) {
            kept_enabled=true;EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
            EXPECT_TRUE(nc_reset->is_resetting);
        }
    }
    EXPECT_TRUE(moved);EXPECT_TRUE(kept_enabled);
    EXPECT_EQ(cycle_tp->exec_state,EXEC_IDLE);
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, NonOeFaultQuickStopAndUnknownAreNotDisableSuccess) {
    runZero();fullResetSetup();rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    for(int state : {HAL_DRIVE_FAULT,HAL_DRIVE_QUICK_STOP_ACTIVE,HAL_DRIVE_UNKNOWN}) {
        h1_test_override_spindle_drive_state(state);nc_set_resetted();
        EXPECT_FALSE(nc_reset->is_resetted);EXPECT_TRUE(nc_reset->is_resetting);
    }
    ASSERT_EQ(h1_test_commit(),HAL_OK);ASSERT_EQ(h1_test_begin(),HAL_OK);
    for(int i=0;i<20;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
}
TEST_F(HalH1SpindleTest, MissingFeedbackAbortsStopAndCannotCompleteReset) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(reset_stop_pending());
    h1_test_invalidate_snapshot();
    reset_prepare_motion();nc_set_resetted();hal_axis_intent_poll();
    EXPECT_FALSE(reset_stop_pending());EXPECT_FALSE(nc_reset->is_resetted);
    EXPECT_FALSE(reset_spindle_torque_allowed(true));
    EXPECT_TRUE(nc_reset->is_resetting);
}

TEST_F(HalH1SpindleTest, PublicResetOverridesStaleTorqueAndDoesNotRetryRejection) {
    runZero();fullResetSetup();nc_set_resetting(HMI_RESET);reset_prepare_motion();
    spindle_control_update();alarm_poll_hal();nc_set_resetted();
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque=true;
    EXPECT_FALSE(reset_spindle_torque_allowed(true));
    EXPECT_TRUE(reset_spindle_disable_needed());
    // 注入一次暂存阶段拒绝：必须可见，且不能被重试。
    reset_spindle_disable_submitted(HAL_ERROR_BUSY);
    for(int i=0;i<5;++i) {
        nc_set_resetted();
        EXPECT_FALSE(reset_spindle_disable_needed());
        EXPECT_FALSE(reset_spindle_torque_allowed(true));
        EXPECT_FALSE(nc_reset->is_resetted);
    }
}
TEST_F(HalH1SpindleTest, FullResetCancelsNoOrientationM29Preparation) {
    runZero();ASSERT_TRUE(spindle_request_m29_no_orient());fullResetSetup();
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_FALSE(g_spindle.m29_active);EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, FullResetCancelsOrientationHoldWithoutNormalExit) {
    orient();fullResetSetup();rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_FALSE(g_spindle.m19_active);EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_NE(axis().control_state,HAL_CONTROL_ORIENT_HOLD);
}

TEST_F(HalH1SpindleTest, StopTimeoutReportsFailureAndCannotFinishReset) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(reset_stop_pending());
    g_spindle.csv_stop_cycles=29999; // reach the existing executor budget
    fullFrame();
    EXPECT_FALSE(reset_stop_pending());
    EXPECT_FALSE(reset_spindle_torque_allowed(true));
    for(int i=0;i<10;++i)fullFrame();
    EXPECT_TRUE(nc_reset->is_resetting);EXPECT_FALSE(nc_reset->is_resetted);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, DiagnosticTorqueMirrorCannotCancelActiveBusiness) {
    runZero();
    rt_data->axis_data[AXIS_SP].status.spindle_wants_torque=false;
    for(int i=0;i<8;++i) {
        hal_axis_intent_poll();
        ASSERT_EQ(h1_test_commit(),HAL_OK);ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_TRUE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
}
TEST_F(HalH1SpindleTest, DiagnosticTorqueMirrorCannotCreateEnableDemand) {
    for(int i=0;i<15;++i) {
        rt_data->axis_data[AXIS_SP].status.spindle_wants_torque=true;
        hal_axis_intent_poll();
        ASSERT_EQ(h1_test_commit(),HAL_OK);ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
}

TEST_F(HalH1SpindleTest, NormalEnableRejectedDuringResetIsNotDeferred) {
    runZero();fullResetSetup();nc_set_resetting(HMI_RESET);
    EXPECT_FALSE(alarm_spindle_request_torque(true));
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    for(int i=0;i<10;++i)fullFrame();
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, NormalEnableRejectedDuringEstopIsNotDeferred) {
    fullResetSetup();rt_data->sys_state[SYS_ESTOP_INPUT]=true;
    EXPECT_FALSE(alarm_spindle_request_torque(true));
    for(int i=0;i<5;++i)fullFrame();
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    rt_data->sys_state[SYS_ESTOP_INPUT]=false;rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, PublicNormalDemandCanBeRevokedWithoutLegacyFieldWrite) {
    runZero();ASSERT_TRUE(alarm_spindle_request_torque(false));
    for(int i=0;i<8;++i) {
        hal_axis_intent_poll();
        ASSERT_EQ(h1_test_commit(),HAL_OK);ASSERT_EQ(h1_test_begin(),HAL_OK);
    }
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, CspStopFreezesFinalTargetWhileWaitingForSettle) {
    spindle_request_m29();frames(110);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    fullResetSetup();
    PLAN_BLOCK block={};block.type=RIGID_TAP;block.plane=PLANE_XY;
    cycle_tp->plan_block=&block;cycle_tp->exec_cmd=DO_TAP_CMD;
    cycle_tp->exec_state=EXEC_RUNNING;
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=1.23;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    cycle_tp->exec_state=EXEC_IDLE;fullFrame();
    ASSERT_NEAR(axis().command_pos,1.23,.011);
    // 之后再写可变插补游标，不算产生了新的运动。
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=99;
    fullFrame();
    EXPECT_NEAR(axis().command_pos,1.23,.011);
}

TEST_F(HalH1SpindleTest, CspStopRejectsInvalidAssociatedFeedPosition) {
    spindle_request_m29();frames(110);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    fullResetSetup();PLAN_BLOCK block={};block.type=RIGID_TAP;block.plane=PLANE_XY;
    cycle_tp->plan_block=&block;cycle_tp->exec_cmd=DO_TAP_CMD;cycle_tp->exec_state=EXEC_RUNNING;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(motion_csp_stop_pending());
    h1_test_axis_position_valid(RT_HAL_AXIS_Z,0);fullFrame();
    EXPECT_FALSE(motion_csp_stop_pending());EXPECT_FALSE(reset_spindle_torque_allowed(true));
    EXPECT_TRUE(nc_reset->is_resetting);
}
TEST_F(HalH1SpindleTest, FeedRebuildDoesNotRequireInvalidSpindlePosition) {
    spindle_request_m29();frames(110);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    fullResetSetup();PLAN_BLOCK block={};block.type=RIGID_TAP;block.plane=PLANE_XY;
    cycle_tp->plan_block=&block;cycle_tp->exec_cmd=DO_TAP_CMD;cycle_tp->exec_state=EXEC_RUNNING;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    cycle_tp->exec_state=EXEC_IDLE;
    for(int i=0;i<12 && motion_csp_stop_pending();++i)fullFrame();
    ASSERT_FALSE(motion_csp_stop_pending());
    rt_data->axis_data[AXIS_SP].motor_cmd_pos=123;
    for(int i=0;i<20;++i) {
        h1_test_axis_position_valid(RT_HAL_AXIS_SP,0);fullFrame();
    }
    EXPECT_EQ(update_pos_flag,1); // feed baseline has recovered independently
    EXPECT_TRUE(reset_spindle_position_required());
    EXPECT_TRUE(nc_reset->is_resetting);
    EXPECT_DOUBLE_EQ(rt_data->axis_data[AXIS_SP].motor_cmd_pos,123);
    fullFrame(); // valid spindle feedback permits its own rebuild and completion
    EXPECT_FALSE(reset_spindle_position_required());EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NEAR(rt_data->axis_data[AXIS_SP].motor_cmd_pos,axis().actual_pos,.011);
}

TEST_F(HalH1SpindleTest, PositionOutputSurvivesM29BusinessCompletion) {
    spindle_request_m29();frames(110);ASSERT_TRUE(spindle_m29_is_ready());
    spindle_complete_m29();
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_IDLE);
    fullResetSetup();rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=3;
    fullFrame();
    EXPECT_NEAR(axis().command_pos,3,.011);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, NewM3CannotSwitchCsvBeforeCspFollowingSettles) {
    spindle_request_m29();frames(110);
    h1_test_spindle_follow_error(20);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(1);
    EXPECT_EQ(axis().target_mode,HAL_AXIS_MODE_CSP);
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSP);
    frames(30);EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSV);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, M29ModePreparationWaitsForActualFeedbackSeparately) {
    spindle_request_m29();
    for(int i=0;i<100 && g_spindle.m29_phase!=SPINDLE_M29_SWITCHING_CSP;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_SWITCHING_CSP);
    for(int i=0;i<3;++i) {
        h1_test_spindle_mode_feedback_valid(0);frames(1);
        EXPECT_FALSE(spindle_m29_has_error());EXPECT_FALSE(spindle_m29_is_ready());
    }
    frames(40);EXPECT_TRUE(spindle_m29_is_ready());
}
TEST_F(HalH1SpindleTest, NoOrientModePreparationWaitsForActualFeedbackSeparately) {
    runZero();ASSERT_TRUE(spindle_request_m29_no_orient());
    for(int i=0;i<40 && g_spindle.m29_phase!=SPINDLE_M29_NO_ORIENT_SWITCHING;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_NO_ORIENT_SWITCHING);
    for(int i=0;i<3;++i) {
        h1_test_spindle_mode_feedback_valid(0);frames(1);
        EXPECT_FALSE(spindle_m29_has_error());EXPECT_FALSE(spindle_m29_is_ready());
    }
    frames(40);EXPECT_TRUE(spindle_m29_is_ready());
}
TEST_F(HalH1SpindleTest, M29ModeFailureCannotHandoverOrAutomaticallyRetry) {
    spindle_request_m29();
    for(int i=0;i<100 && g_spindle.m29_phase!=SPINDLE_M29_SWITCHING_CSP;++i)frames(1);
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_SWITCHING_CSP);
    h1_test_spindle_mode_failure();frames(1);
    EXPECT_TRUE(spindle_m29_has_error());EXPECT_FALSE(motion_spindle_position_active());
    frames(20);
    EXPECT_FALSE(motion_spindle_position_active());EXPECT_FALSE(spindle_m29_is_ready());
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, PlcFinCompletesM29WithoutOwningPositionOutput) {
    spindle_request_m29();frames(110);ASSERT_TRUE(spindle_m29_is_ready());
    PLAN_BLOCK block={};block.mst_data.m_code=M29;cycle_tp->plan_block=&block;
    cycle_tp_mstb_cmd_data_reset();cycle_tp->mstb_cmd_data.signal_mf=true;
    const auto saved_delay=par_chan[0][PAR_CH_TFIN_DELEY].value.val_int;
    par_chan[0][PAR_CH_TFIN_DELEY].value.val_int=2;
    setRegBit(REG_F,7,0);clearRegBit(REG_G,4,3);auto_execute_mstb_cmd();
    EXPECT_FALSE(cycle_tp->mstb_cmd_data.finished);
    setRegBit(REG_G,4,3);
    for(int i=0;i<4 && !cycle_tp->mstb_cmd_data.finished;++i)auto_execute_mstb_cmd();
    EXPECT_TRUE(cycle_tp->mstb_cmd_data.finished);
    EXPECT_EQ(g_spindle.m29_phase,SPINDLE_M29_IDLE);
    EXPECT_FALSE(g_spindle.m29_active);EXPECT_TRUE(motion_spindle_position_active());
    fullResetSetup();rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=2;fullFrame();
    EXPECT_NEAR(axis().command_pos,2,.011);
    par_chan[0][PAR_CH_TFIN_DELEY].value.val_int=saved_delay;
    clearRegBit(REG_G,4,3);clearRegBit(REG_F,7,0);
}

TEST_F(HalH1SpindleTest, ResetCancelsNewM3WaitingForPositionHandover) {
    spindle_request_m29();frames(110);fullResetSetup();
    h1_test_spindle_follow_error(20);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,100));frames(1);
    ASSERT_TRUE(g_spindle.csv_switch_pending);
    EXPECT_TRUE(motion_spindle_position_active());
    rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<35;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);EXPECT_FALSE(g_spindle.csv_switch_pending);
    EXPECT_FALSE(motion_spindle_position_active());EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, PositionOutputCannotBeCreatedByReadyEnumAlone) {
    fullResetSetup();g_spindle.m29_phase=SPINDLE_M29_READY;
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur=5;
    for(int i=0;i<5;++i)fullFrame();
    EXPECT_FALSE(motion_spindle_position_active());
    EXPECT_FALSE(axis().command_valid&HAL_COMMAND_POSITION);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, RejectedM29CannotAbortExistingControlledStop) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(reset_stop_pending());
    EXPECT_FALSE(spindle_request_m29());
    EXPECT_TRUE(reset_stop_pending());
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

TEST_F(HalH1SpindleTest, LostCspEnableDoesNotRestorePositionOwnership) {
    spindle_request_m29();frames(110);ASSERT_TRUE(motion_spindle_position_active());
    ASSERT_EQ(h1_test_force_disable(),0);
    frames(15);
    EXPECT_FALSE(motion_spindle_position_active());
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_FALSE(spindle_m29_is_ready());
    frames(15);EXPECT_FALSE(motion_spindle_position_active());
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, RawEstopImmediatelyInvalidatesOrientationCompletion) {
    orient();ASSERT_TRUE(spindle_orientation_is_done());
    rt_data->sys_state[SYS_ESTOP_INPUT]=1;
    EXPECT_FALSE(spindle_orientation_is_done());
}
TEST_F(HalH1SpindleTest, ResetDiscardsUnconsumedNormalOrientationRelease) {
    runZero();spindle_request_orientation(false);
    nc_set_resetting(HMI_RESET);reset_prepare_motion();spindle_control_update();
    EXPECT_EQ(g_spindle.orient_phase,SPINDLE_ORIENT_IDLE);
    EXPECT_FALSE(g_spindle.orient_exit_submitted);
}
TEST_F(HalH1SpindleTest, InvalidFeedbackDiscardsUnconsumedOrientationEvent) {
    spindle_begin_m19();spindle_request_orientation(true);
    h1_test_invalidate_snapshot();spindle_control_update();
    EXPECT_EQ(g_spindle.pending_event,SPINDLE_EVENT_NONE);
    EXPECT_FALSE(g_spindle.m19_active);
    frames(20);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, InvalidSpeedCannotRelinquishPositionOwnership) {
    spindle_request_m29();frames(110);spindle_complete_m29();
    spindle_set_speed_source(SPINDLE_SPEED_SOURCE_PLC);spindle_set_plc_speed(NAN);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,100));frames(1);
    EXPECT_FALSE(g_spindle.csv_switch_pending);
    EXPECT_TRUE(motion_spindle_position_active());
    EXPECT_EQ(axis().actual_mode,HAL_AXIS_MODE_CSP);
}
TEST_F(HalH1SpindleTest, OrdinaryCsvModeFailureRevokesStartAuthorization) {
    runZero();ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(1);
    h1_test_spindle_mode_failure();frames(1);
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_FALSE(g_spindle.program_run_authorized);
    EXPECT_TRUE(g_spindle.reset_requires_start);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    frames(20);EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, CsvTargetSubmissionFailureRevokesBusiness) {
    runZero();h1_test_request_admission_running(0); // valid cached feedback, requests unavailable
    spindle_control_update();
    h1_test_request_admission_running(1);
    hal_axis_intent_poll();
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_TRUE(g_spindle.reset_requires_start);
    EXPECT_FALSE(rt_data->axis_data[AXIS_SP].status.spindle_wants_torque);
    frames(20);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, CsvSnapshotRejectionCannotBeOverwrittenByNextTarget) {
    runZero();h1_test_spindle_speed_failure();frames(1);
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_TRUE(g_spindle.reset_requires_start);
    frames(20);EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    ASSERT_TRUE(spindle_request_program_run(SPINDLE_CW,0));frames(20);
    EXPECT_TRUE(g_spindle.run_authorized);
    EXPECT_EQ(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
TEST_F(HalH1SpindleTest, InvalidLivePlcSpeedRevokesCsvAuthorization) {
    runZero();spindle_set_speed_source(SPINDLE_SPEED_SOURCE_PLC);
    spindle_set_plc_speed(NAN);frames(1);
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_TRUE(g_spindle.reset_requires_start);
    frames(20);EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}

#define PREPARATION_CANCEL_TEST(Name,NoOrient,Phase,Estop) \
TEST_F(HalH1SpindleTest, Name) { cancelPreparation(NoOrient,Phase,Estop); }
PREPARATION_CANCEL_TEST(ResetDuringM29Orientation,false,SPINDLE_M29_ORIENTING,false)
PREPARATION_CANCEL_TEST(ResetDuringM29Settling,false,SPINDLE_M29_ORIENT_SETTLING,false)
PREPARATION_CANCEL_TEST(EstopDuringM29Settling,false,SPINDLE_M29_ORIENT_SETTLING,true)
PREPARATION_CANCEL_TEST(ResetDuringM29ModePreparation,false,SPINDLE_M29_SWITCHING_CSP,false)
PREPARATION_CANCEL_TEST(ResetDuringM29Alignment,false,SPINDLE_M29_ALIGNING,false)
PREPARATION_CANCEL_TEST(EstopDuringM29Orientation,false,SPINDLE_M29_ORIENTING,true)
PREPARATION_CANCEL_TEST(EstopDuringM29ModePreparation,false,SPINDLE_M29_SWITCHING_CSP,true)
PREPARATION_CANCEL_TEST(EstopDuringM29Alignment,false,SPINDLE_M29_ALIGNING,true)
PREPARATION_CANCEL_TEST(ResetDuringNoOrientDeceleration,true,SPINDLE_M29_NO_ORIENT_STOPPING,false)
PREPARATION_CANCEL_TEST(ResetDuringNoOrientModePreparation,true,SPINDLE_M29_NO_ORIENT_SWITCHING,false)
PREPARATION_CANCEL_TEST(ResetDuringNoOrientAlignment,true,SPINDLE_M29_NO_ORIENT_ALIGNING,false)
PREPARATION_CANCEL_TEST(EstopDuringNoOrientDeceleration,true,SPINDLE_M29_NO_ORIENT_STOPPING,true)
PREPARATION_CANCEL_TEST(EstopDuringNoOrientModePreparation,true,SPINDLE_M29_NO_ORIENT_SWITCHING,true)
PREPARATION_CANCEL_TEST(EstopDuringNoOrientAlignment,true,SPINDLE_M29_NO_ORIENT_ALIGNING,true)
#undef PREPARATION_CANCEL_TEST
TEST_F(HalH1SpindleTest, ResetDuringActualM19OrientationUsesCommonStop) {
    spindle_begin_m19();spindle_request_orientation(true);
    for(int i=0;i<40 && axis().control_state!=HAL_CONTROL_ORIENTING;++i)frames(1);
    ASSERT_EQ(axis().control_state,HAL_CONTROL_ORIENTING);
    fullResetSetup();rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);EXPECT_FALSE(g_spindle.m19_active);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    EXPECT_NE(axis().request_rejection.category,HAL_ISSUE_EXIT_ORIENT);
}
TEST_F(HalH1SpindleTest, CsvStopTargetRejectionFailsCommonResetWithoutRetry) {
    runZero();fullResetSetup();g_spindle.velocity_planner.current_vel=100;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();ASSERT_TRUE(reset_stop_pending());
    h1_test_spindle_speed_failure();fullFrame();
    EXPECT_FALSE(reset_stop_pending());EXPECT_FALSE(nc_reset->is_resetted);
    for(int i=0;i<30;++i)fullFrame();
    EXPECT_TRUE(nc_reset->is_resetting);EXPECT_FALSE(nc_reset->is_resetted);
    EXPECT_FALSE(g_spindle.run_authorized);
}
TEST_F(HalH1SpindleTest, EstopPreemptsCspStopWithoutRestoringPositionOwnership) {
    spindle_request_m29();frames(110);
    rt_data->axis_data[AXIS_Z].status.is_disable=false;frames(20);
    fullResetSetup();PLAN_BLOCK block={};block.type=RIGID_TAP;block.plane=PLANE_XY;
    cycle_tp->plan_block=&block;cycle_tp->exec_cmd=DO_TAP_CMD;cycle_tp->exec_state=EXEC_RUNNING;
    rt_data->nc_event=SYS_EVENT_HMI_RESET;fullFrame();
    ASSERT_TRUE(motion_csp_stop_pending());
    rt_data->sys_state[SYS_ESTOP_INPUT]=1;
    for(int i=0;i<5;++i)fullFrame();
    EXPECT_FALSE(reset_stop_pending());EXPECT_FALSE(motion_spindle_position_active());
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
    rt_data->sys_state[SYS_ESTOP_INPUT]=0;rt_data->nc_event=SYS_EVENT_HMI_RESET;
    for(int i=0;i<40;++i)fullFrame();
    EXPECT_FALSE(nc_reset->is_resetting);EXPECT_FALSE(motion_spindle_position_active());
    EXPECT_FALSE(g_spindle.run_authorized);
    EXPECT_NE(axis().drive_state,HAL_DRIVE_OPERATION_ENABLED);
}
#ifndef DEBUG_MODE_ENABLED
TEST_F(HalH1SpindleTest, ProductionM29CommandEntersPreparationThroughAutoMst) {
    PLAN_BLOCK block={};block.mst_data.m_code=M29;block.mst_data.size=1;
    cycle_tp->plan_block=&block;auto_get_mstb_cmd();
    ASSERT_TRUE(g_spindle.m29_active);frames(110);
    EXPECT_TRUE(spindle_m29_is_ready());EXPECT_TRUE(motion_spindle_position_active());
}
TEST_F(HalH1SpindleTest, ProductionM29ReturnCommandUsesNoOrientationPreparation) {
    runZero();rt_data->sys_state[SYS_IS_RIGID_RETURN]=1;
    PLAN_BLOCK block={};block.mst_data.m_code=M29;block.mst_data.size=1;
    cycle_tp->plan_block=&block;auto_get_mstb_cmd();
    ASSERT_EQ(g_spindle.m29_phase,SPINDLE_M29_NO_ORIENT_STOPPING);
    frames(65);EXPECT_TRUE(spindle_m29_is_ready());
    EXPECT_EQ(axis().control_state,HAL_CONTROL_IDLE);
}
#endif
#endif
