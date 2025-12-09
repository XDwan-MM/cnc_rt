#ifndef TASK_H
#define TASK_H

#include "ncinterpreter.h"
#include "ci_pmc.h"
#include "cmd_msg.h"
#include "status_msg.h"


extern int stepping;
extern int steppingWait;
extern int emcTaskQueueCommand(RCS_CMD_MSG &cmd);
//extern int emcTaskOnce(const char *inifile, EMC_IO_STAT &emcioStatus);
extern QList<MotionDataSend> interp_list;
extern NCInterpreter *m_interpreter;
extern ItpParam itp_parameter;

// Returns 0 if all joints are homed, 1 if any joints are un-homed.
int all_homed(void);
bool jogging_is_active(void);

int emcTaskInit();
int emcTaskHalt();
int emcTaskStateRestore();
int emcTaskAbort();
int emcTaskSetMode(NC_MODE mode);
int emcTaskSetState(EMC_TASK_STATE state);
int emcTaskUpdate(EMC_TASK_STAT * stat);

int emcTaskPlanInit();
int emcTaskPlanSetWait();
int emcTaskPlanIsWait();
int emcTaskPlanClearWait();
int emcTaskPlanSynch();
int emcTaskPlanSetOptionalStop(bool state);
int emcTaskPlanSetBlockDelete(bool state);
void emcTaskPlanExit();
int emcTaskPlanOpen(const char *file);
int emcTaskPlanPause();
int emcTaskPlanResume();
int emcTaskPlanClose();
int emcTaskPlanReset();

int emcTaskPlanLine();
int emcTaskPlanLevel();
int emcTaskPlanCommand(char *cmd);

int emcIoAbort();

#endif // TASK_H
