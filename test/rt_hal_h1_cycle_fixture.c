#include "../src/hal/rt_hal_adapter.c"
#include "backend/greemaster/pdo/entry_access.h"
#include "backend/greemaster/bus/master.h"
int h1_test_begin(void) {
    int rc=hal_rt_wait_cycle(s_ctx);if(rc)return rc;
    rc=hal_rt_begin_cycle(s_ctx);if(rc)return rc;
    publish_snapshot();return HAL_OK;
}
int h1_test_commit(void) { take_requests();return hal_rt_commit_cycle(s_ctx); }
int h1_test_force_disable(void) {
    /* 模拟外部掉 OE；这个夹具只与内存后端链接，接不到硬件主站。 */
    return Master_ServoWrite(2,DEV_DICT_ROLE_CONTROL_WORD,0);
}

int h1_test_spindle_position(uint32_t* value,int write) {
    return Master_SdoTransfer(2,write?0x607A:0x6064,0,value,4,write,1000);
}

void h1_test_invalidate_snapshot(void) { s_snap_valid=0; }
void h1_test_override_spindle_drive_state(int state) {
    s_snap[RT_HAL_AXIS_SP].drive_state=state;
}

void h1_test_axis_position_valid(int slot,int valid) {
    if(valid)s_snap[slot].feedback_valid|=HAL_FEEDBACK_POSITION;
    else s_snap[slot].feedback_valid&=~HAL_FEEDBACK_POSITION;
}

void h1_test_spindle_follow_error(double error) {
    HalAxisSnapshot* axis=&s_snap[RT_HAL_AXIS_SP];
    axis->command_valid|=HAL_COMMAND_POSITION;
    axis->command_pos=axis->actual_pos+error;
}

void h1_test_spindle_mode_feedback_valid(int valid) {
    if(valid)s_snap[RT_HAL_AXIS_SP].feedback_valid|=HAL_FEEDBACK_MODE;
    else s_snap[RT_HAL_AXIS_SP].feedback_valid&=~HAL_FEEDBACK_MODE;
}
void h1_test_spindle_mode_failure(void) {
    s_snap[RT_HAL_AXIS_SP].mode_setting_state=HAL_MODE_SETTING_FAILED;
    s_snap[RT_HAL_AXIS_SP].execution_error.category=HAL_ISSUE_MODE;
    s_snap[RT_HAL_AXIS_SP].execution_error.code=HAL_ERROR_TIMEOUT;
}

void h1_test_request_admission_running(int running) { s_running=running; }

void h1_test_spindle_speed_failure(void) {
    s_snap[RT_HAL_AXIS_SP].request_rejection.category=HAL_ISSUE_SPINDLE_SPEED;
    s_snap[RT_HAL_AXIS_SP].request_rejection.code=HAL_ERROR_BUSY;
}
