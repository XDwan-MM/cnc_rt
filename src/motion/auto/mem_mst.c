#include "mem_mst.h"
#include "mcode_queue.h"
#include "global.h"
#include <varapi.h>

static MCODE_BLOCK* mcode_block;

// todo：统一M指令处理
void execute_mcode(){
    // if(mcode_queue_is_empty(mcode_queue)) return;
    // // 获取M指令
    // mcode_block = mcode_queue_peak(mcode_queue);
    // setVarChanInt(0, VAR_CHAN_RUN_ROW, mcode_block->line_number, 0);
    // if (mcode_block == NULL) {
    //     printf("Error: mcode_queue_pop returned NULL!\n");
    //     return; // 或其他错误处理
    // }
    // // 根据M指令类型处理
    // switch (mcode_block->m_code)
    // {
    // case M00: // 停止主轴，关冷却液，指向下一程序段，并停止做进一步处理，按”循环启动“程序继续执行
    //     spindle_data->spl_event = SPINDLE_EVENT_STOP;
    //     spindle_data->mcode_feedback = false;
    //     break;
    // case M02: // 停主轴，关冷却液，消除G93坐标偏置和刀具偏置，返回到起始程序段。系统将切换到基准工件坐标系。
    //     spindle_data->spl_event = SPINDLE_EVENT_STOP;
    //     spindle_data->mcode_feedback = false;
    //     break;    
    // case M03: // 主轴正转
    //     spindle_data->spl_event = SPINDLE_EVENT_CW;
    //     spindle_data->mcode_feedback = false;
    //     break;
    // case M04: // 主轴反转
    //     spindle_data->spl_event = SPINDLE_EVENT_CCW;
    //     spindle_data->mcode_feedback = false;
    //     break;
    // case M05: // 主轴停止
    //     spindle_data->spl_event = SPINDLE_EVENT_STOP;
    //     spindle_data->mcode_feedback = false;
    //     break;
    // case M08: // 切削液开
    //     break;
    // case M09: // 切削液关
    //     break;
    // case M12: // 暂停，按”循环启动“程序继续执行
    //     break;
    // case M32: // 润滑开
    //     break;
    // case M33: // 润滑关
    //     break;
    // default:
    //     break;
    // }
    // // 更新F变量状态
    // // 完成后删除M指令
    // mcode_queue_pop(mcode_queue);
}

