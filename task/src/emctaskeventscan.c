#include "emctaskeventscan.h"
//#include "regapi.h"
#include "emc_type.h"
#include "kevent.h"
#include "shm_write.h"
#include <string.h>

void emctaskEventScan()
{
    detect_operation_mode();
    check_nc_file_path();
    cycle_start();


}
//typedef enum{
//    SYS_EVENT_NONE = 0,
//    SYS_EVENT_RESET,
//    SYS_EVENT_RERUN,
//    SYS_EVENT_PAUSE,
//    SYS_EVENT_X,           // +X 正向
//    SYS_EVENT_X_NEG,       // -X 负向
//    SYS_EVENT_Y,           // +Y 正向
//    SYS_EVENT_Y_NEG,       // -Y 负向
//    SYS_EVENT_Z,           // +Z 正向
//    SYS_EVENT_Z_NEG,       // -Z 负向
//}SYS_EVENT;

//// 系统状态
//typedef struct{
//    // button模式：按下激活
//    SYS_EVENT nc_event;
void cycle_start()
{
    if(cnc_state->cycle_start)
    {
        sys->nc_event = SYS_EVENT_RERUN;
        cmd._type = EMC_TASK_PLAN_RUN_TYPE;
        kevent_put(&cmd);
        cnc_state->cycle_start = 0;
    }
}

void reset_event_scan()
{
    if(cnc_state->reset)
    {
        sys->nc_event = SYS_EVENT_RESET;
        cmd._type = EMC_TASK_PLAN_RUN_TYPE;
        kevent_put(&cmd);
        cnc_state->reset = 0;
    }
}
// 判断FANUC操作模式
void detect_operation_mode() {
    if(cnc_state->current_mode != cnc_state->pre_current_mode)
    {
        cnc_state->pre_current_mode = cnc_state->current_mode;
        cmd._type = EMC_TASK_SET_MODE_TYPE;
        cmd.mode = cnc_state->current_mode;
        kevent_put(&cmd);
    }

//    switch(shared->current_mode)
//    case MODE_EDIT:
//        cmd.mode = shared->current_mode;
//        kevent_put()
//        break;
//    case MODE_MEM:
//        break;
//    case MODE_MDI:
//        break;
//    case MODE_MPG:
//        break;
//    case MODE_JOG:
//        break;
//    case MODE_RAPID:
//        break;
//    case MODE_HOMING:
//        break;
//    default:
//        break;
//     int gn043;
//     getRegValue8(REG_G,43,&gn043);
//     // 提取关键位（位0=MD1, 位1=MD2, 位2=MD4, 位7=ZRN）
//     unsigned char md4 = (gn043 >> 2) & 0x01;  // 位2
//     unsigned char md2 = (gn043 >> 1) & 0x01;  // 位1
//     unsigned char md1 = gn043 & 0x01;         // 位0
//     unsigned char dnci = (gn043 >> 5) & 0x01; // 位5
//     unsigned char zrn = (gn043 >> 7) & 0x01;  // 位7

//     // 模式判断逻辑（基于表2.6(a)）
//     if (md4 == 0 && md2 == 0 && md1 == 0) {
//         return MODE_MDI;
//     }
//     else if (md4 == 0 && md2 == 0 && md1 == 1 ) {
//         return (dnci == 0) ? MODE_MEMORY : MODE_EDIT;
//     }
//     else if (md4 == 1 && md2 == 0 && md1 == 0) {
//         return MODE_HANDLE;
//     }
//     else if (md4 == 1 && md2 == 0 && md1 == 1) {
//         return (zrn == 1) ? MODE_REF_RETURN : MODE_JOG;
//     }
//     else {
//         return MODE_UNKNOWN;
//     }
}

#define MDI_NC_FILE_PATH "/home/gree/Desktop/build-RT_CNC-Desktop_Qt_5_15_16_GCC_64bit-Debug/a.nc"
#define MEM_NC_FILE_PATH "/home/gree/Desktop/build-RT_CNC-Desktop_Qt_5_15_16_GCC_64bit-Debug/a.nc"
static char pre_mdi_nc_file[100];
static char pre_mem_nc_file[100];

#include <unistd.h>
 //判断人机交互的MDI、MEN的NC文件路径是否存在。若存在则发送EMC_TASK_PLAN_OPEN_TYPE消息，否则发送EMC_TASK_PLAN_CLOSE_TYPE消息。
 void check_nc_file_path() {
    if(emcStatus->task.mode == MODE_MDI)
    {
        if(strcmp(pre_mdi_nc_file,MDI_NC_FILE_PATH))//文件不同
        {
            strcpy(pre_mdi_nc_file, MDI_NC_FILE_PATH);
            if(access(MDI_NC_FILE_PATH,F_OK) == 0) {
                strcpy(cmd.file, MDI_NC_FILE_PATH);
                cmd._type = EMC_TASK_PLAN_OPEN_TYPE;
                kevent_put(&cmd);
            } else {
                kevent_put(EMC_TASK_PLAN_CLOSE_TYPE);
            }

        }

    }
    else if(emcStatus->task.mode == MODE_MEM)
    {
        if(strcmp(pre_mem_nc_file, MEM_NC_FILE_PATH))
        {
            strcpy(pre_mem_nc_file, MEM_NC_FILE_PATH);
            if(access(MEM_NC_FILE_PATH,F_OK) == 0) {
                cmd.file = MEM_NC_FILE_PATH;
                cmd._type = EMC_TASK_PLAN_OPEN_TYPE;
                kevent_put(&cmd);
            } else {
                kevent_put(EMC_TASK_PLAN_CLOSE_TYPE);
            }
        }
    }

 }


// //判断主轴倍率是否变化，若变化，发送EMC_TRAJ_SET_SPINDLE_SCALE_TYPE命令
// void check_spindle_scale_change() {
//     if(check_spindle_scale_change(SPINDLE_SCALE)) {
//         kevent_put(EMC_TRAJ_SET_SPINDLE_SCALE_TYPE);
//     }
// }

// bool check_spindle_scale_change(double scale) {
//     return scale != get_spindle_scale();
// }

// double get_spindle_scale() {
//     return 0.0;
// }

// //快速进给倍率，发送EMC_TRAJ_SET_RAPID_SCALE_TYPE
// void check_rapid_scale_change() {
//     if(check_rapid_scale_change(RAPID_SCALE)) {
//         kevent_put(EMC_TRAJ_SET_RAPID_SCALE_TYPE);
//     }
// }

// bool check_rapid_scale_change(double scale) {
//     return scale != get_rapid_scale();
// }

// double get_rapid_scale() {
//     return 0.0;
// }

// //进给倍率，发送EMC_TRAJ_SET_SCALE_TYPE
// void check_feed_scale_change() {
//     if(check_feed_scale_change(FEED_SCALE)) {
//         kevent_put(EMC_TRAJ_SET_SCALE_TYPE);
//     }
// }

// bool check_feed_scale_change(double scale) {
//     return scale != get_feed_scale();
// }

// double get_feed_scale() {
//     return 0.0;
// }
