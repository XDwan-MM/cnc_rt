#include "mem_mst.h"
#include "mcode_queue.h"
#include "global.h"
#include <varapi.h>
#include "spl_control.h"

static MCODE_BLOCK* mcode_block;

// todo：统一M指令处理
void execute_mcode(){
     if(mcode_queue_is_empty(mcode_queue)) return;
     // 获取M指令
     mcode_block = mcode_queue_peak(mcode_queue);
     setVarChanInt(0, VAR_CHAN_RUN_ROW, mcode_block->line_number, 0);
     if (mcode_block == NULL) {
         printf("Error: mcode_queue_pop returned NULL!\n");
         return; // 或其他错误处理
     }
     // 根据M指令类型处理
     switch (mcode_block->m_code)
     {
     case M00: // 暂停
         spl_data.spl_event = SPINDLE_EVENT_STOP;
         spl_data.mcode_feedback = false;
         break;
     case M02: // 复位
         spl_data.spl_event = SPINDLE_EVENT_STOP;
         spl_data.mcode_feedback = false;
         break;
     case M03: // 主轴正转
         spl_data.spl_event = SPINDLE_EVENT_CW;
         spl_data.emot_speed = mcode_block->spindle.speed;
         spl_data.mcp_target_speed = mcode_block->spindle.speed;
         spl_data.mcode_feedback = false;
         break;
     case M04: // 主轴反转
         spl_data.spl_event = SPINDLE_EVENT_CCW;
         spl_data.emot_speed = -1.0 * mcode_block->spindle.speed;
         spl_data.mcp_target_speed = mcode_block->spindle.speed;
         spl_data.mcode_feedback = false;
         break;
     case M05: // 主轴停止
         spl_data.spl_event = SPINDLE_EVENT_STOP;
         spl_data.emot_speed = 0;
         spl_data.mcode_feedback = false;
         break;
     case M08: // 切削液开
         break;
     case M09: // 切削液关
         break;
     case M12: // 暂停，按”循环启动“程序继续执行
         break;
     case M19: // 主轴定位
         spl_data.spl_event = SPINDLE_EVENT_ORIENT;
         break;
     case M32: // 润滑开
         break;
     case M33: // 润滑关
         break;
     default:
         break;
     }
     // 更新F变量状态
     // 完成后删除M指令
     mcode_queue_pop(mcode_queue);
}

