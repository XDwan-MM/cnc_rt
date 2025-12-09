// #include <stdlib.h>
// #include <sys/stat.h>		// struct stat
// #include <unistd.h>		// stat()
// #include <limits.h>		// PATH_MAX

#include "textbuffer.h"
#include "emc.h"

// #include "emc_nml.h"
#include "task.h"		// emcTaskCommand etc
#include "QString"
#include "textbuffer.h"
#include "emcglb.h"
// #define USER_DEFINED_FUNCTION_MAX_DIRS 5
// #define MAX_M_DIRS (USER_DEFINED_FUNCTION_MAX_DIRS+1)
// //note:the +1 is for the PROGRAM_PREFIX or default directory==nc_files

// /* flag for how we want to interpret traj coord mode, as mdi or auto */
static EMC_OPERATION_MODE mdiOrAuto = EMC_OPERATION_MODE::MEM;

NCInterpreter *m_interpreter;
ItpParam itp_parameter;
QList<MotionDataSend> interp_list;
MotionDataSend motiondatasend;

EMC_STAT *emcStatus;

#define EMC_DEBUG_INTERP 1

// // Print error messages thrown by interpreter
// static char interp_error_text_buf[LINELEN];
// static char interp_stack_buf[LINELEN];

// static void print_interp_error(int retval)
// {
//     int index = 0;
//     if (retval == 0) {
//         return;
//     }

//     if (0 != emcStatus) {
//         emcStatus->task.interpreter_errcode = retval;
//     }

//     interp_error_text_buf[0] = 0;
//     m_interpreter.error_text(retval, interp_error_text_buf, LINELEN);
//     if (0 != interp_error_text_buf[0]) {
//         rcs_print_error("interp_error: %s\n", interp_error_text_buf);
//     }
//     emcOperatorError("%s", interp_error_text_buf);
//     index = 0;
//     if (emc_debug & EMC_DEBUG_INTERP) {
//         printf("Interpreter stack: \t");
//         while (index < 5) {
//             interp_stack_buf[0] = 0;
//             m_interpreter.stack_name(index, interp_stack_buf, LINELEN);
//             if (0 == interp_stack_buf[0]) {
//                 break;
//             }
//             printf(" - %s ", interp_stack_buf);
//             index++;
//         }
//         printf("\n");
//     }
// }

// /*
//   format string for user-defined programs, e.g., "programs/M1%02d" means
//   user-defined programs are in the programs/ directory and are named
//   M1XX, where XX is a two-digit string.
// */
// static char user_defined_fmt[MAX_M_DIRS][EMC_SYSTEM_CMD_LEN]; // ex: "dirname/M1%02d"


// int emcTaskInit()
// {
//     //原linuxCNC该函数功能：1. 读取配置目录2. 解析用户 M 代码路径3. 扫描并注册 M 代码程序4. 错误处理



//     return 0;
// }

// int emcTaskHalt()
// {
//     return 0;
// }

int emcTaskStateRestore()
{
    int res = 0;
//    // Do NOT restore on MDI command
//    if (emcStatus->task.mode == EMC_OPERATION_MODE::MEM) {
//        // Validity of state tag checked within restore function
//        res = pinterp->restore_from_tag(emcStatus->motion.traj.tag);
//    }
    return res;
}

// void emcTaskQueueTaskPlanSynchCmd()
// {
//     auto taskPlanSynchCmd = std::make_unique<EMC_TASK_PLAN_SYNCH>();
//     emcTaskQueueCommand(std::move(taskPlanSynchCmd));
// }

// int emcTaskAbort()
// {
//     emcMotionAbort();

//     // clear out the pending command
//     emcTaskCommand.flag = 0;
//     interp_list.clear();

//     // clear out the interpreter state
//     emcStatus->task.interpState = EMC_TASK_INTERP::IDLE;
//     emcStatus->task.execState = EMC_TASK_EXEC::DONE;
//     emcStatus->task.task_paused = 0;
//     emcStatus->task.motionLine = 0;
//     emcStatus->task.readLine = 0;
//     emcStatus->task.command[0] = 0;
//     emcStatus->task.callLevel = 0;

//     stepping = 0;
//     steppingWait = 0;

//     // now queue up command to resynch interpreter
//     emcTaskQueueTaskPlanSynchCmd();

//     // without emcTaskPlanClose(), a new run command resumes at
//     // aborted line-- feature that may be considered later
//     {
//         int was_open = taskplanopen;
//         emcTaskPlanClose();
//         emcTaskPlanReset();  // Flush any unflushed segments
//         if (emc_debug & EMC_DEBUG_INTERP && was_open) {
//             printf("emcTaskPlanClose() called at %s:%d\n", __FILE__,
//                       __LINE__);
//         }
//     }

//     return 0;
// }

int emcTaskSetMode(EMC_OPERATION_MODE mode)
{
    int retval = 0;

    // if (jogging_is_active()) {
    //     printf("Ignoring task mode change while jogging");
    //     return 0;
    // }
    // switch (mode) {
    //     case EMC_OPERATION_MODE::JOG:
    //     case EMC_OPERATION_MODE::RAPID:
    //     case EMC_OPERATION_MODE::EDIT:
    //     case EMC_OPERATION_MODE::ZRN:
    //     case EMC_OPERATION_MODE::MPG:
    //     // go to manual mode
    //     if (all_homed()) {
    //         emcTrajSetMode(EMC_TRAJ_MODE::TELEOP);
    //     } else {
    //         emcTrajSetMode(EMC_TRAJ_MODE::FREE);
    //     }
    //     mdiOrAuto = EMC_OPERATION_MODE::MEM;	// we'll default back to here
    //     break;

    // case EMC_OPERATION_MODE::MDI:
    //     // go to mdi mode
    //     emcTrajSetMode(EMC_TRAJ_MODE::COORD);
    //     emcTaskAbort();
    //     emcTaskPlanSynch();
    //     mdiOrAuto = EMC_OPERATION_MODE::MDI;
    //     break;

    // case EMC_OPERATION_MODE::MEM:
    //     // go to auto mode
    //     emcTrajSetMode(EMC_TRAJ_MODE::COORD);
    //     emcTaskAbort();
    //     emcTaskPlanSynch();
    //     mdiOrAuto = EMC_OPERATION_MODE::MEM;
    //     break;

    // default:
    //     retval = -1;
    //     break;
    // }

    return retval;
}

int emcTaskSetState(EMC_TASK_STATE state)
{
    int t;
    int retval = 0;

   switch (state) {
   case EMC_TASK_STATE::OFF:
//        emcMotionAbort();
//        // turn the machine servos off-- go into READY state
//        for (t = 0; t < emcStatus->motion.traj.spindles; t++)  emcSpindleAbort(t);
//        emcTrajDisable();
//        // emcIoAbort(EMC_ABORT::TASK_STATE_OFF);
//        //    emcCoolantFloodOff();//TODO: race here
//        //     emcTaskAbort();
//        emcJointUnhome(-2); // only those joints which are volatile_home
//        //  emcAbortCleanup(EMC_ABORT::TASK_STATE_OFF);
//        //  emcTaskPlanSynch();
//        break;

   case EMC_TASK_STATE::ON:
       // turn the machine servos on
      // emcTrajEnable();
       //     emcCoolantFloodOff();//TODO: race here
       break;

    case EMC_TASK_STATE::ESTOP_RESET:
//        // reset the estop
//        //     emcAuxEstopOff();
//        //      emcCoolantFloodOff();//TODO: race here
//        //     emcTaskAbort();
//        // emcIoAbort(EMC_ABORT::TASK_STATE_ESTOP_RESET);
//        for (t = 0; t < emcStatus->motion.traj.spindles; t++) emcSpindleAbort(t);
//        //  emcAbortCleanup(EMC_ABORT::TASK_STATE_ESTOP_RESET);
//        //  emcTaskPlanSynch();
        break;

   case EMC_TASK_STATE::ESTOP:
    //    emcMotionAbort();
//        for (t = 0; t < emcStatus->motion.traj.spindles; t++) emcSpindleAbort(t);
//        // go into estop-- do both IO estop and machine servos off
//        //     emcAuxEstopOn();
//        emcTrajDisable();
//        //   emcCoolantFloodOff();//TODO: race here
//        //     emcTaskAbort();
//        //  emcIoAbort(EMC_ABORT::TASK_STATE_ESTOP);
//        for (t = 0; t < emcStatus->motion.traj.spindles; t++) emcSpindleAbort(t);
//        emcJointUnhome(-2); // only those joints which are volatile_home
//        //   emcAbortCleanup(EMC_ABORT::TASK_STATE_ESTOP);
//        //   emcTaskPlanSynch();
//        break;

   default:
       retval = -1;
       break;
   }

    return retval;
}

// // WM access functions

// /*
//   determineMode()

//   Looks at mode of subsystems, and returns associated mode

//   Depends on traj mode, and mdiOrAuto flag

//   traj mode   mdiOrAuto     task mode
//   ---------   ---------     ---------
//   FREE        XXX           MANUAL
//   TELEOP      XXX           MANUAL
//   COORD       MDI           MDI
//   COORD       AUTO          AUTO
//   */
// EMC_OPERATION_MODE determineMode()
// {
//     if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::FREE) {
//         return EMC_OPERATION_MODE::MANUAL;
//     }
//     if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP) {
//         return EMC_OPERATION_MODE::MANUAL;
//     }
//     // for EMC_TRAJ_MODE::COORD
//     return mdiOrAuto;
// }

// /*
//   determineState()

//   Looks at state of subsystems, and returns associated state

//   Depends on traj enabled, io estop, and desired task state

//   traj enabled   io estop      state
//   ------------   --------      -----
//   DISABLED       ESTOP         ESTOP
//   ENABLED        ESTOP         ESTOP
//   DISABLED       OUT OF ESTOP  ESTOP_RESET
//   ENABLED        OUT OF ESTOP  ON
//   */
// static EMC_TASK_STATE determineState()
// {
//     if (emcStatus->io.aux.estop) {
//         return EMC_TASK_STATE::ESTOP;
//     }

//     if (!emcStatus->motion.traj.enabled) {
//         return EMC_TASK_STATE::ESTOP_RESET;
//     }

//     return EMC_TASK_STATE::ON;
// }

static int waitFlag = 0;

const double rf = 2000;
double var[50000] = {};
MacroVar cmv[30000] = {};
TextBuffer textBuffer;

void init()
{
    for (int i = 0; i < 30000; ++i)
    {
        cmv[i].varDouble = 0.0;
    }

    cmv[18004].varDouble= 10.0;   //设置半径值超差
    cmv[17031].varDouble= 0.0; //设置坐标旋转1轴运动计算方式
    cmv[5410].varDouble= -90.0;  //设置坐标旋转默认角度
    cmv[17028].varDouble= 1.0;   //默认米制单位
    cmv[8132].varDouble= 1.0;     //激活比例缩放功能G51
    cmv[8077].varDouble= 10.0;    //设置渐开线插补误差最大值
    cmv[5114].varDouble= 0.5;     //G73钻深孔,D默认值
    cmv[5115].varDouble= 1.0;     //G83深孔加工循环,D默认值
}


int emcTaskPlanInit()
{
    int retval =1;
    init();

    m_interpreter = new NCInterpreter();
    m_interpreter->setMacroVariable(cmv);

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanInit() returned %d\n", retval);
    }

    return retval;
}

int emcTaskPlanSetWait()
{
    waitFlag = 1;

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanSetWait() called\n");
    }

    return 0;
}

int emcTaskPlanIsWait()
{
    return waitFlag;
}

int emcTaskPlanClearWait()
{
    waitFlag = 0;

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanClearWait() called\n");
    }

    return 0;
}

// int emcTaskPlanSetOptionalStop(bool state)
// {
//     itp_parameter.
//     SET_OPTIONAL_PROGRAM_STOP(state);
//     return 0;
// }

int emcTaskPlanSetBlockDelete(bool state)
{
    itp_parameter.blkSkip = state;
    m_interpreter->setItpParam(itp_parameter);
    //SET_BLOCK_DELETE(state);
    return 0;
}


int emcTaskPlanSynch()
{
    int retval = true;

    double mech_x = 0 /*m_modelMgr->posModel()->machinePos().x*/;
    double mech_y = 0 /*m_modelMgr->posModel()->machinePos().y*/;
    double mech_z = 0 /*m_modelMgr->posModel()->machinePos().z*/;
    double mech_a = 0 /*m_modelMgr->posModel()->machinePos().a*/;
    double mech_b = 0 /*m_modelMgr->posModel()->machinePos().b*/;
    double mech_c = 0 /*m_modelMgr->posModel()->machinePos().c*/;

    MacroVar mv[30000] = {};
    for (int i = 0; i < 30000; ++i)
    {
        //        qDebug() << "i: " << i;
        //  mv[i].varDouble = m_macroVarMgr->getMacroVar(i);
        mv[i].valid = false;
        mv[i].read = 6;
        mv[i].write = 6;
    }

    m_interpreter->setMacroVariable(mv);
    m_interpreter->setOriginMacPos(mech_x, mech_y, mech_z, mech_a, mech_b, mech_c);
    // ItpParam itpParam;
    // itpParam.blkSkip = m_blockSkipFlag;
    // itpParam.nullRunFeed = m_emtryRunFlag;
    // itpParam.toolLenList = m_modelMgr->toolModel()->lengthCompensation();
    // itpParam.toolRadList = m_modelMgr->toolModel()->diameterCompensation();
    // m_itp.setItpParam(itpParam);



    NCITPErrorType itpError = m_interpreter->errorList().isEmpty() ?
                                  NCITPErrorType::Exception : m_interpreter->errorList().first();

    if (itpError != NCITPErrorType::Exception)
    {
        retval = false;
        //        emcTaskAbort();
    }

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanSynch() returned %d\n", retval);
    }

    return retval;
}

// void emcTaskPlanExit()
// {
//     if (pinterp != NULL) {
//         m_interpreter.exit();
//     }
// }

int emcTaskPlanOpen(const QString *file)
{
    QString filePath = *file;
    if (emcStatus != 0) {
        emcStatus->task.motionLine = 0;
        emcStatus->task.currentLine = 0;
        emcStatus->task.readLine = 0;
    }
    textBuffer.setFile(filePath);
    m_interpreter->setTextBuffer(&textBuffer);
    int retval = true;
    // int retval = m_interpreter.open(file);
    // if (retval > INTERP_MIN_ERROR) {
    //     print_interp_error(retval);
    //     return retval;
    // }
    taskplanopen = 1;

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanOpen(%s) returned %d\n", file, retval);
    }

    return retval;
}

int emcTaskPlanClose()
{
    int retval = true;
    delete m_interpreter;

    taskplanopen = 0;
    return retval;
}

int emcTaskPlanReset()
{
    int retval = true;
    m_interpreter->reset();
    // if (retval > INTERP_MIN_ERROR) {
    //     print_interp_error(retval);
    // }

    return retval;
}

int emcTaskPlanLine()
{
    int retval = m_interpreter->lineNumber();

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanLine() returned %d\n", retval);
    }

    return retval;
}

int emcTaskPlanLevel()
{
    int retval = true;
    // m_interpreter.call_level();

    if (emc_debug & EMC_DEBUG_INTERP) {
        printf("emcTaskPlanLevel() returned %d\n", retval);
    }

    return retval;
}

// int emcTaskPlanCommand(char *cmd)
// {
//     char buf[LINELEN];

//     strcpy(cmd, m_interpreter.command(buf, LINELEN));

//     if (emc_debug & EMC_DEBUG_INTERP) {
//         printf("emcTaskPlanCommand(%s) called. (line_number=%d)\n",
//                   cmd, emcStatus->task.readLine);
//     }

//     return 0;
// }

// int emcTaskUpdate(EMC_TASK_STAT * stat)
// {
//     stat->mode = determineMode();
//     EMC_TASK_STATE oldstate = stat->state;
//     stat->state = determineState();

//     if(oldstate == EMC_TASK_STATE::ON && oldstate != stat->state) {
//         emcTaskAbort();
//         for (int s = 0; s < emcStatus->motion.traj.spindles; s++) emcSpindleAbort(s);
//         emcIoAbort(EMC_ABORT::TASK_STATE_NOT_ON);
//         emcAbortCleanup(EMC_ABORT::TASK_STATE_NOT_ON);
//     }

//     // execState set in main
//     // interpState set in main
//     if (emcStatus->motion.traj.id > 0) {
//         stat->motionLine = emcStatus->motion.traj.id;
//     }
//     // currentLine set in main
//     // readLine set in main

//     char buf[LINELEN];
//     rtapi_strxcpy(stat->file, m_interpreter.file(buf, LINELEN));
//     // command set in main

//     // update active G and M codes
//     // Start by assuming that we can't unpack a state tag from motion
//     int res_state = INTERP_ERROR;
//     if (emcStatus->task.interpState != EMC_TASK_INTERP::IDLE) {
//         res_state = m_interpreter.active_modes(&stat->activeGCodes[0],
//                                         &stat->activeMCodes[0],
//                                         &stat->activeSettings[0],
//                                         emcStatus->motion.traj.tag);
//         if (res_state != INTERP_ERROR) {
//             rtapi_strxcpy(stat->file, emcStatus->motion.traj.tag.filename);
//             //    stat->currentLine = emcStatus->motion.traj.tag.fields[0];
//         }
//     }
//     // If we get an error from trying to unpack from the motion state, always
//     // use interp's internal state, so the active state is never out of date
//     if (emcStatus->task.mode != EMC_OPERATION_MODE::MEM ||
//         res_state == INTERP_ERROR) {
//         m_interpreter.active_g_codes(&stat->activeGCodes[0]);
//         m_interpreter.active_m_codes(&stat->activeMCodes[0]);
//         m_interpreter.active_settings(&stat->activeSettings[0]);
//     }

//     //update state of optional stop
//     stat->optional_stop_state = GET_OPTIONAL_PROGRAM_STOP();

//     //update state of block delete
//     stat->block_delete_state = GET_BLOCK_DELETE();

//     return 0;
// }

// int emcAbortCleanup(EMC_ABORT reason, const char *message)
// {
//     int status = m_interpreter.on_abort((int)reason,message);
//     if (status > INTERP_MIN_ERROR)
//         print_interp_error(status);
//     return status;
// }

