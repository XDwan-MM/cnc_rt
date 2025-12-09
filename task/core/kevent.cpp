#include "kevent.h"
#include "threadsafeshareddata_c.h"

RCS_CMD_MSG cmd;

bool kevent_put(RCS_CMD_MSG *cmd)//放入命令消息队列
{
    cmd->serial_number++;
    if (!writeEmcCommandBufferData_C(cmd)) {
        cmd->serial_number--;
        printf("!writeEmcCommandBufferData_C(&cmd)\n");
        return false;
    }
    return true;
}

int kevent_get(RCS_CMD_MSG* cmd)
{
  return readEmcCommandBufferData_C(cmd);
}

int kevent_get_qSize()
{
   return  getEmcCommandBufferDataCount_C();
}

void kevent_init()
{
    clearEmcCommandBufferData_C();
}
