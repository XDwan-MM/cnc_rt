#include "rt_drive_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

#undef LOG
static void discard_log(const char *format, ...) { (void)format; }
#define LOG(level, ...) discard_log(__VA_ARGS__)
#include "../src/drive/rt_drive_control.c"
#include "../src/protect/alarm.c"
#include "../src/plc/plc_scanner/plc_f_signal_handler.c"

static RT_DATA data;
RT_DATA *rt_data = &data;
FIR_MA *filter_ma;
void fir_ma_reset(FIR_MA *filter, int window_size) { (void)filter; (void)window_size; }
static RtHalAxisStatus feedback[RT_HAL_AXIS_COUNT];
static unsigned char f[512];
static int unreadable = -1;
static int alarms[256], reset_requests, settled, reset_timeout, request_error;
static uint32_t reset_request_mask, observed_wait_mask;
static int enabled[4], estops;
static int mock_oe_loss[4], alarm_drop, alarm_attempts, rebuild_count;
void emcmot_rebuild_axis_position_from_feedback(int axis) {
    ++rebuild_count;
    rt_data->axis_data[axis].motor_cmd_pos=rt_data->axis_data[axis].motor_act_pos;
}
EVENT_QUEUE *event_queue;
int update_pos_flag;

int setAlarm(int type, int level, int id)
{ (void)type; (void)level; if(id==132) { ++alarm_attempts; if(alarm_drop) return -1; } alarms[id] = 1; return 0; }
int delAlarm(int type, int level, int id)
{ (void)type; (void)level; alarms[id] = 0; return 0; }
int cleanAllAlarm(int type, int level)
{ (void)type; (void)level; memset(alarms, 0, sizeof(alarms)); return 0; }
int setVarChanInt(int no, int index, int value, int client)
{ (void)no; (void)index; (void)value; (void)client; return 0; }
GNC_RES tp_reset(void) { return 0; }
GNC_RES rt_data_reset(void) { return 0; }
GNC_RES event_queue_reset(EVENT_QUEUE *queue) { (void)queue; return 0; }
void Axis_Limit_Manager_Init(void) {}
void plc_lock_state_resync(void) {}
void reset_ferror_control(void) {}
void rt_set_parser_state(int error, int line) { (void)error; (void)line; }
int32_t rt_hal_cycle_error(void) { return HAL_OK; }
void hal_error_text(int32_t code, char *out, uint32_t size)
{ (void)code; if (size) out[0] = '\0'; }
int rt_hal_fault_reset_settled(int *timeout)
{ *timeout = reset_timeout; return settled; }
int rt_hal_fault_reset_settled_mask(uint32_t mask, int *timeout)
{ observed_wait_mask = mask; *timeout = reset_timeout; return mask == 0 || settled; }
int rt_hal_fault_reset_state(RtHalAxisSlot slot, int32_t *out)
{ (void)slot; *out = settled ? RT_HAL_RESET_DONE : RT_HAL_RESET_PENDING; return HAL_OK; }
int rt_hal_config_hal_axis(RtHalAxisSlot slot) { return (int)slot; }
int rt_hal_config_is_spindle(RtHalAxisSlot slot) { return slot == RT_HAL_AXIS_SP; }
int rt_hal_request_fault_reset(RtHalAxisSlot slot)
{ reset_request_mask |= 1u << slot; ++reset_requests; return request_error; }
/* 这个孤立的报警用例没有启用公共复位协调器。完整的复位行为由
 * tst_hal_h1_spindle 通过真实的 reset.c 覆盖。 */
bool reset_spindle_torque_allowed(bool ordinary) { return ordinary; }
bool reset_spindle_business_allowed(void) { return true; }
bool reset_spindle_disable_needed(void) { return false; }
void reset_spindle_disable_submitted(int rc) { (void)rc; }
static unsigned enable_calls, axis_enable_calls[4];
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on)
{ ++enable_calls; ++axis_enable_calls[slot]; enabled[slot] = on;feedback[slot].enabled=on;return HAL_OK; }
int rt_hal_clear_execution_error(RtHalAxisSlot slot) { (void)slot; return HAL_OK; }
int rt_hal_request_axis_mode(RtHalAxisSlot slot, HalAxisMode mode) { (void)slot;(void)mode;return HAL_OK; }
int rt_hal_axis_snapshot(RtHalAxisSlot slot, HalAxisSnapshot* out) {
    if ((int)slot==unreadable)return HAL_ERROR_NOT_RUNNING;
    memset(out,0,sizeof(*out));
    out->feedback_valid=HAL_FEEDBACK_STATUS|HAL_FEEDBACK_MODE;
    if(feedback[slot].position_valid) out->feedback_valid |= HAL_FEEDBACK_POSITION;
    out->actual_pos=feedback[slot].actual_pos;
    if(mock_oe_loss[slot]) {
        out->execution_error.category=HAL_ISSUE_AXIS_CONDITION;
        out->execution_error.code=HAL_ERROR_NOT_ENABLED;
    }
    out->drive_state=feedback[slot].enabled?HAL_DRIVE_OPERATION_ENABLED:HAL_DRIVE_READY;
    out->actual_mode=out->target_mode=HAL_AXIS_MODE_CSP;
    out->mode_setting_state=HAL_MODE_SETTING_READY;
    return HAL_OK;
}
int rt_hal_request_estop(RtHalAxisSlot slot)
{ (void)slot; ++estops; return HAL_OK; }

int rt_hal_axis_status(RtHalAxisSlot slot, RtHalAxisStatus *out)
{
    assert(slot >= 0 && slot < RT_HAL_AXIS_COUNT);
    if ((int)slot == unreadable) return HAL_ERROR_STATE;
    *out = feedback[slot];
    return HAL_OK;
}

int setRegBit(RegType type, int idx, int bit)
{ assert(type == REG_F); f[idx] |= 1u << bit; return 0; }
int clearRegBit(RegType type, int idx, int bit)
{ assert(type == REG_F); f[idx] &= ~(1u << bit); return 0; }

static void publish(void)
{ alarm_poll_drive_errors(); plc_output_axis_status(); }
static int healthy(void) { return (f[0] >> 6) & 1; }
static int spindle_fault(void) { return f[45] & 1; }

static void expire_reset_wait(void)
{
    s_reset_started.tv_sec -= 7;
    alarm_poll_hal();
#ifdef DEBUG_MODE_ENABLED
    assert(s_reset_waiting && !s_reset_failed);
    s_reset_started.tv_sec -= DEBUG_HAL_TIMEOUT_MS / 1000u;
    alarm_poll_hal();
#endif
}

int main(void)
{
    memset(&data, 0, sizeof(data));
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        feedback[i].error_code_valid = 1;
        feedback[i].position_valid = 1;
        feedback[i].raw_status = 0x27;
    }
    publish();
    assert(healthy() && !spindle_fault());
    rt_drive_request_fault_reset();
    assert(reset_requests == 0 && !s_reset_waiting);
    for (int i = 0; i < 5100; ++i) alarm_poll_hal();
    assert(!alarms[132]); // Prolonged absence of enable is diagnostic only.
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        feedback[i].error_code = 0x1234;
        publish();
        assert(!healthy());
        assert(alarms[30 + i]);
        assert(spindle_fault() == (i == RT_HAL_AXIS_SP));
        for (int j = 0; j < RT_HAL_AXIS_COUNT; ++j)
            assert(data.axis_data[j].status.is_error == (i == j));
        feedback[i].error_code = 0;
        publish();
        assert(healthy() && !spindle_fault());
        assert(!alarms[30 + i]);
    }
    puts("PASS: real alarm publisher -> axis flags -> F0.6/F45.0 for all four slots");

    // Masking suppresses alarms, not actual fault feedback.
    feedback[0].error_code = 0x1234;
    data.axis_data[0].status.is_disable = true;
    publish();
    assert(!healthy() && !alarms[30]);
    data.axis_data[0].status.is_disable = false;
    feedback[0].error_code_valid = 0;
    publish();
    assert(healthy());
    feedback[0].error_code_valid = 1;
    publish();
    unreadable = 0;
    publish();
    assert(!healthy() && alarms[30]);
    unreadable = -1;
    publish();
    assert(!healthy());
    // Drive-only alarm goes through the HMI_RESET alarm handler.
    assert(alarms[30]);
    data.nc_event = SYS_EVENT_HMI_RESET;
    handle_alarm_cmd();
    assert(reset_requests == 1 && reset_request_mask == 1u && s_reset_waiting && alarms[30]);
    memset(enabled, -1, sizeof(enabled));
    hal_axis_intent_poll();
    assert(enabled[0] == -1); // Never cancel an in-flight reset with disable.
    settled = 1; // HAL reset completion and drive fault feedback are separate.
    alarm_poll_hal();
    assert(!s_reset_waiting && alarms[30] && observed_wait_mask == 1u);
    feedback[0].error_code = 0;
    feedback[0].raw_status = 0x08;
    alarm_poll_hal();
    assert(!s_reset_waiting && alarms[30]);
    feedback[0].raw_status = 0x27;
    alarm_poll_hal();
    assert(!s_reset_waiting && !alarms[30]);
    publish();
    assert(healthy());
    hal_axis_intent_poll();
    hal_axis_intent_poll();
    hal_axis_intent_poll();
    assert(enabled[0] == 1);
    data.sys_state[SYS_IS_ALARMING] = true;
    hal_axis_intent_poll();
    assert(enabled[0] == 1); // Unrelated alarms do not change the enable policy.
    data.sys_state[SYS_IS_ALARMING] = false;
    puts("PASS: drive-only alarm -> HMI reset -> only faulty axis -> confirmed recovery");
    /* 使能完成后若观察到状态丢失，不会再回放那个旧请求。 */
    feedback[0].enabled=0;
    unsigned before=enable_calls;
    for(int n=0;n<10;++n)hal_axis_intent_poll();
    assert(enable_calls==before);
    puts("PASS: completed enable loss does not automatically re-enable");
    feedback[0].error_code=0xDEAD;publish();hal_axis_intent_poll();
    assert(enabled[0]==0);
    before=enable_calls; feedback[0].error_code=0; feedback[0].actual_pos=5.0;
    data.axis_data[0].motor_cmd_pos=1.0;
    const int old_rebuilds=rebuild_count;
    publish(); for(int n=0;n<10;++n)hal_axis_intent_poll();
    assert(enable_calls>before && enabled[0]==1);
    assert(rebuild_count>old_rebuilds && data.axis_data[0].motor_cmd_pos==5.0);
    puts("PASS: residual code disable realigns to new feedback before re-enable");

    feedback[0].error_code = 0xFF43;
    publish();
    hal_axis_intent_poll();
    assert(enabled[0] == 0 && enabled[1] == 1);
    settled = 0;
    handle_alarm_cmd();
    expire_reset_wait();
    assert(!s_reset_waiting && s_reset_failed && alarms[131] && alarms[30]);
    hal_axis_intent_poll();
    for (int i = 0; i < 4; ++i) assert(enabled[i] == 0);
    handle_alarm_cmd(); // Explicit retry releases the timeout latch.
    assert(!s_reset_failed && s_reset_waiting);
    feedback[0].error_code = 0;
    unreadable = 0;
    alarm_poll_hal();
    assert(s_reset_waiting);
    expire_reset_wait();
    assert(s_reset_failed && alarms[131]);
    unreadable = -1;
    feedback[0].error_code = 0x1234;
    request_error = HAL_ERROR_NOT_RUNNING;
    settled = 0;
    handle_alarm_cmd();
    assert(alarms[123]);
    alarm_poll_hal(); /* 请求直接被拒时不必等到总超时。 */
    assert(s_reset_failed && !s_reset_waiting);
    request_error = 0;
    handle_alarm_cmd();
    data.sys_state[SYS_ESTOP_INPUT] = true;
    hal_axis_intent_poll();
    assert(!s_reset_waiting && estops == 4);
    int requests_before = reset_requests;
    handle_alarm_cmd();
    assert(reset_requests == requests_before);
    data.sys_state[SYS_ESTOP_INPUT] = false;
    handle_alarm_cmd();
    settled = 1;
    reset_timeout = 1;
    alarm_poll_hal();
    assert(s_reset_failed && alarms[131]);
    reset_timeout = 0;
    handle_alarm_cmd();
    alarm_poll_hal();
    assert(!s_reset_failed && !s_reset_waiting && !alarms[131]);
    puts("PASS: residual code / unreadable / rejected reset / HAL timeout / retry / estop");
    feedback[0].error_code=0x1111; feedback[2].raw_status=0x08;
    reset_request_mask=0; settled=0;
    rt_drive_request_fault_reset();
    assert(reset_request_mask==((1u<<0)|(1u<<2)) && s_reset_waiting);
    alarm_poll_hal();
    assert(observed_wait_mask==((1u<<0)|(1u<<2)) && s_reset_waiting);
    settled=1; alarm_poll_hal(); assert(!s_reset_waiting);
    feedback[0].error_code=0; feedback[2].raw_status=0x27;
    unreadable=1; reset_request_mask=0;
    rt_drive_request_fault_reset();
    assert(s_reset_waiting && reset_request_mask==0);
    alarm_poll_hal(); assert(s_reset_waiting);
    unreadable=-1; feedback[1].raw_status=0x08; settled=0;
    alarm_poll_hal();
    assert(s_reset_waiting && reset_request_mask==(1u<<1));
    settled=1; alarm_poll_hal(); assert(!s_reset_waiting);
    assert(s_pos_realign[1]); /* 先不可读、后来才发现故障也要重建位置 */
    puts("PASS: multiple faults and unreadable feedback becoming a fault");

    for(int i=0;i<4;++i) { feedback[i].raw_status=0x27; feedback[i].error_code=0; }
    publish(); s_reset_waiting=s_reset_failed=0;
    data.sys_state[SYS_ESTOP_INPUT]=data.sys_state[SYS_IS_ESTOP_ACTIVE]=0;
    feedback[0].enabled=0; s_enable_stage[0]=ENABLE_COMPLETE; s_enable_intent[0]=1;
    mock_oe_loss[0]=1; alarm_drop=1; alarm_attempts=0;
    alarm_poll_hal(); assert((s_oe_latched&1u) && alarm_attempts==1 && !alarms[132]);
    alarm_drop=0; alarm_poll_hal(); assert(alarm_attempts==2 && alarms[132]);
    data.axis_data[0].status.is_disable=1; hal_axis_intent_poll();
    data.axis_data[0].status.is_disable=0; alarm_axis_mode_requested(0,HAL_AXIS_MODE_CSP);
    before=axis_enable_calls[0]; for(int n=0;n<5;++n)hal_axis_intent_poll();
    assert(axis_enable_calls[0]==before && (s_oe_latched&1u));
    alarm_reset_latch_clear(); assert(s_oe_latched&1u); /* 通用重置入口不能绕过 HMI 复位 */
    mock_oe_loss[0]=0; data.nc_event=SYS_EVENT_HMI_RESET; handle_alarm_cmd();
    assert(!(s_oe_latched&1u)); alarm_poll_hal();
    for(int n=0;n<5;++n)hal_axis_intent_poll();
    assert(axis_enable_calls[0]>before);
    puts("PASS: unexpected enable loss stays locked until HMI reset; alarm retries");

    s_pos_realign[0]=1; s_enable_stage[0]=ENABLE_MODE_WAIT; s_enable_mode[0]=HAL_AXIS_MODE_CSP;
    s_enable_intent[0]=1; feedback[0].enabled=0; feedback[0].position_valid=0;
    before=axis_enable_calls[0]; hal_axis_intent_poll();
    assert(axis_enable_calls[0]==before && s_enable_stage[0]==ENABLE_FAILED && s_pos_realign[0]);
    puts("PASS: invalid position feedback prevents re-enable");

    data.sys_state[SYS_IS_READY]=1; data.axis_data[AXIS_SP].status.spindle_wants_torque=1;
    f[1]|=1u<<7; f[0]|=1u<<6; rt_drive_publish_bus_fault();
    assert(!data.sys_state[SYS_IS_READY] && data.sys_state[SYS_IS_ALARMING]);
    assert(!data.axis_data[AXIS_SP].status.spindle_wants_torque);
    assert(!(f[1]&(1u<<7)) && !(f[0]&(1u<<6)) && (f[1]&1));
    puts("PASS: bus fault revokes ready and feedback-health indications");
    return 0;
}
