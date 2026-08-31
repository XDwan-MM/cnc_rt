#include "hmi_scanner.h"
#include "event_queue.h"
#include <hmisysapi.h>
#include "global.h"

static OperateKeyEvent hmi_events[DEMO_MAX_EVENTS_PER_POLL];
static PLC_EVENT event;

void hmi_events_scan(void)
{
    int n = operateSignalPopEvents(hmi_events, DEMO_MAX_EVENTS_PER_POLL);

    for (int i = 0; i < n; i++) {
        uint32_t code  = hmi_events[i].keyCode & KEY_CODE_MASK;
        bool ctrl  = hmi_events[i].keyCode & KEY_MOD_CTRL;
        bool alt   = hmi_events[i].keyCode & KEY_MOD_ALT;
        bool shift = hmi_events[i].keyCode & KEY_MOD_SHIFT;
        bool pressed = (hmi_events[i].state == KEY_STATE_PRESSED);

        switch (hmi_events[i].source) {
        case KEY_SOURCE_KEYBOARD:   // 物理键盘事件
            switch (code)
            {
            case KEY_KP_0:
                if (pressed){
                    event.id = PLC_EV_MSG_HMI;
                    event.hmi_payload.press_key = code;
                    event_queue_push(event_queue, &event);
                    LOG(LOG_LEVEL_INFO, "扫描到HMI按下reset");
                } 
                break;
            default:
                break;
            }
            break;
        case KEY_SOURCE_SOFTKEY:    // 软键事件
            break;
        case KEY_SOURCE_COMMAND:
            switch (code)
            {
            case CMD_RESET_PIECE_COUNT:
                event.id = PLC_EV_MSG_HMI;
                event.hmi_payload.press_key = code;
                event_queue_push(event_queue, &event);
                LOG(LOG_LEVEL_INFO, "扫描到HMI按下重置加工件数");
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }

        // LOG(LOG_LEVEL_DEBUG, "[OperateSignal] keyCode=0x%03X src=%s state=%s mod=%s%s%s",
        //        code,
        //        sourceName(hmi_events[i].source),
        //        stateName(hmi_events[i].state),
        //        ctrl  ? "Ctrl+"  : "",
        //        alt   ? "Alt+"   : "",
        //        shift ? "Shift+" : "");
    }
}