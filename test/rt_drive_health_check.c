#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

#undef LOG
static void discard_log(const char *format, ...) { (void)format; }
#define LOG(level, ...) discard_log(__VA_ARGS__)
#include "../src/protect/alarm.c"
#include "../src/plc/plc_scanner/plc_f_signal_handler.c"

static RT_DATA data;
RT_DATA *rt_data = &data;
static HalCAxisStatus feedback[RT_HAL_AXIS_COUNT];
static unsigned char f[512];
static int unreadable = -1;
static int alarms[256], reset_requests, settled, reset_timeout, request_error;
static int enabled[4], estops;
EVENT_QUEUE *event_queue;
int update_pos_flag;

int setAlarm(int type, int level, int id)
{ (void)type; (void)level; alarms[id] = 1; return 0; }
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
void reset_ferror_control(void) {}
void rt_set_parser_state(int error, int line) { (void)error; (void)line; }
int32_t rt_hal_cycle_error(void) { return HAL_OK; }
void hal_error_text(int32_t code, char *out, uint32_t size)
{ (void)code; if (size) out[0] = '\0'; }
int rt_hal_fault_reset_settled(int *timeout)
{ *timeout = reset_timeout; return settled; }
int rt_hal_request_fault_reset(RtHalAxisSlot slot)
{ (void)slot; ++reset_requests; return request_error; }
int rt_hal_request_axis_enable(RtHalAxisSlot slot, int on)
{ enabled[slot] = on; return HAL_OK; }
int rt_hal_request_estop(RtHalAxisSlot slot)
{ (void)slot; ++estops; return HAL_OK; }

int rt_hal_axis_status(RtHalAxisSlot slot, HalCAxisStatus *out)
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

int main(void)
{
    memset(&data, 0, sizeof(data));
    for (int i = 0; i < RT_HAL_AXIS_COUNT; ++i) {
        feedback[i].error_code_valid = 1;
        feedback[i].raw_status = 0x27;
    }
    publish();
    assert(healthy() && !spindle_fault());
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
    assert(reset_requests == 4 && s_reset_waiting && alarms[30]);
    memset(enabled, -1, sizeof(enabled));
    hal_axis_intent_poll();
    assert(enabled[0] == -1); // Never cancel an in-flight reset with disable.
    settled = 1; // HAL DONE alone is insufficient; 603F must read 0.
    alarm_poll_hal();
    assert(s_reset_waiting && alarms[30]);
    feedback[0].error_code = 0;
    feedback[0].raw_status = 0x08;
    alarm_poll_hal();
    assert(s_reset_waiting && alarms[30]);
    feedback[0].raw_status = 0x27;
    alarm_poll_hal();
    assert(!s_reset_waiting && !alarms[30]);
    publish();
    assert(healthy());
    hal_axis_intent_poll();
    assert(enabled[0] == 1);
    data.sys_state[SYS_IS_ALARMING] = true;
    hal_axis_intent_poll();
    assert(enabled[0] == 1); // Unrelated alarms do not change the enable policy.
    data.sys_state[SYS_IS_ALARMING] = false;
    puts("PASS: drive-only alarm -> HMI reset -> four requests -> confirmed recovery");

    feedback[0].error_code = 0xFF43;
    publish();
    hal_axis_intent_poll();
    assert(enabled[0] == 0 && enabled[1] == 1);
    handle_alarm_cmd();
    s_reset_started.tv_sec -= 7;
    alarm_poll_hal();
    assert(!s_reset_waiting && s_reset_failed && alarms[131] && alarms[30]);
    hal_axis_intent_poll();
    for (int i = 0; i < 4; ++i) assert(enabled[i] == 0);
    handle_alarm_cmd(); // Explicit retry releases the timeout latch.
    assert(!s_reset_failed && s_reset_waiting);
    feedback[0].error_code = 0;
    unreadable = 0;
    alarm_poll_hal();
    assert(s_reset_waiting);
    s_reset_started.tv_sec -= 7;
    alarm_poll_hal();
    assert(s_reset_failed && alarms[131]);
    unreadable = -1;
    request_error = HAL_ERROR_NOT_RUNNING;
    settled = 0;
    handle_alarm_cmd();
    assert(alarms[123]);
    s_reset_started.tv_sec -= 7;
    alarm_poll_hal();
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
    return 0;
}
