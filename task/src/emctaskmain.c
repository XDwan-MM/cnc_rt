#include "cmd_msg.h"
#include "taskapi.h"
#include "status_msg.h"
#include "kevent.h"
#include "emc_type.h"
#include "emctaskeventscan.h"
#include "alarm_manager.h"
#include "task.h"
#include "emccfg.h"
#include "emc_type.h"

int stepping = 0;
int steppingWait = 0;
static int steppedLine = 0;
static int emcTaskEager = 0;

static RCS_CMD_MSG *emcCommand = 0;
static EMC_STAT *emcStatus = 0;
static MotionDataSend *emcTaskCommand = 0;
volatile int done;

static int emcTaskIssueCommand(RCS_CMD_MSG * cmd);
static int emcTaskIssueExecCommand(MotionDataSend * cmd);

static int emcTaskExecute(void);
int all_homed(void) {
//    for (int i = 0; i < emcStatus->motion.traj.joints; i++) {
//        if(!emcStatus->motion.joint[i].homed) // XXX
//            return 0;
//    }
    return 1;
}



 int program_tier = 1;
 bool tier_change_flag = false;
 
 void readahead_reading(void)
 {
     if (interp_list.size() <= DEFAULT_EMC_TASK_INTERP_MAX_LEN) {
         int count = 0;
     interpret_again:
         if (emcTaskPlanIsWait())
         {
             // delay reading of next line until all is done
             if (interp_list.size() == 0 &&
                 // emcTaskCommand == 0 &&
                 emcStatus->task.execState ==
                     EMC_TASK_EXEC::TASK_STATE_DONE) {
                 emcTaskPlanClearWait();
             }
         }
         else
         {
             m_interpreter->nextLine();

             QList<pmc_data> motionDataList = m_interpreter->motionData();
             if (motionDataList.isEmpty())
             {   // 空包
                 if (m_interpreter->state().unModalState == Unmodality::MacroUse)
                 {
                     ++program_tier;
                     tier_change_flag = true;
                 }
                 // // cmv set
                 // if (!m_interpreter->state().varInputList.isEmpty())
                 // {
                 //     for (int key : m_interpreter->state().varInputList.keys())
                 //     {
                 //         double value = m_interpreter->state().varInputList.value(key);
                 //         qDebug() << "cmv set:" << key << value;
                 //         c_cmv.set(key, value);
                 //     }
                 // }

                 MotionDataSend motionDataSend;
                 motionDataSend.lineNumber = m_interpreter->lineNumber();
                 // motionDataSend.lineNumCount = m_writeLineNum;
                 // motionDataSend.dataindex = -1;
                 motionDataSend.tier = program_tier;
             }
             else
             {   // 实包
                 int dataIndex = 1;
                 for (pmc_data motionData : motionDataList)
                 {
                     if (motionData.mode == pmc_mode::PMC_MODE_M_CMD)
                     {
                         if (motionData.m_code == 99)
                         {
                             ++program_tier;
                             tier_change_flag = true;
                         }
                     }
                     MotionDataSend motionDataSend;
                     motionDataSend.motiondata = motionData;
                     motionDataSend.lineNumber = m_interpreter->lineNumber();
                     // motionDataSend.lineNumCount = m_writeLineNum;
                     motionDataSend.dataindex = dataIndex++;
                     motionDataSend.tier = program_tier;
                     interp_list.append(motionDataSend);
                 }
             }
             // got a good line
             // record the line number and command
             emcStatus->task.readLine = emcTaskPlanLine();

             NCITPErrorType itpErrorType = m_interpreter->errorList().isEmpty() ?
                                               NCITPErrorType::Exception : m_interpreter->errorList().first();
             if (itpErrorType != NCITPErrorType::Exception) {
                 emcStatus->task.interpState =
                     EMC_TASK_INTERP::WAITING;
                 interp_list.clear();
                 printf("on_abort: [%d] %s\n", (int)EMC_ABORT::INTERPRETER_ERROR,"interpreter error");
             } else if (m_interpreter->isM2M30()) {
                 emcStatus->task.interpState =
                     EMC_TASK_INTERP::WAITING;
             }
             else if (m_interpreter->state().decodeStatus == DecodeStatus::WaitDecode)
             {
                 emcTaskPlanSetWait();
                 // emcTaskQueueTaskPlanSynchCmd();

                 //以下为原来的设计

                 // // 等待译码末端处理
                 // if (parserStateStor.state.decodeStatus == DecodeStatus::WaitDecode)
                 // {
                 //     qDebug() << "wait decode.";
                 //     m_mutex.lock();
                 //     m_waitCondition.wait(&m_mutex);
                 //     {
                 //         qDebug() << "hode decode.";

                 //         MacroVar *mv = nullptr;
                 //         if (m_interpreter.getMacroVariable())
                 //         {
                 //             mv = m_interpreter.getMacroVariable();
                 //         }
                 //         else
                 //         {
                 //             for (int i = 0; i < 30000; ++i)
                 //             {
                 //                 double value = 0.0;
                 //                 c_cmv.take(i, value);
                 //                 mv[i].varDouble = value;
                 //                 mv[i].valid = false;
                 //                 mv[i].read = 6/*c_cmv.getReadGrade(i)*/;
                 //                 mv[i].write = 6/*c_cmv.getWriteGrade(i)*/;
                 //             }
                 //         }
                 //         mv[5061].varDouble = g_stopSkipG31_pos.x;
                 //         mv[5062].varDouble = g_stopSkipG31_pos.y;
                 //         m_interpreter.setGrade(1);
                 //         m_interpreter.setMacroVariable(mv);
                 //         m_interpreter.setOriginMacPos(*j_mech_curr_encoder.x,*j_mech_curr_encoder.y,*j_mech_curr_encoder.z,
                 //                                       *j_mech_curr_encoder.a,*j_mech_curr_encoder.b,*j_mech_curr_encoder.c);

                 //         if (parserStateStor.state.unModalState == Unmodality::ToolMeasure)
                 //         {
                 //             qDebug() << "tool measure";
                 //             MacroVarUpdate result = m_interpreter.autosetToolLenthComp(*j_mech_curr_encoder.z);
                 //             if (result.success)
                 //             {
                 //                 c_cmv.set(result.macroNum, result.updateData);
                 //             }
                 //             else
                 //             {
                 //                 alarmQueue.setMacroAl("对刀错误");
                 //                 m_interpreter.reset();
                 //                 cycleStop();
                 //             }
                 //         }
                 //     }


             }
             else if (m_interpreter->isOver()) {
                 // end of file
                 emcStatus->task.interpState =
                     EMC_TASK_INTERP::WAITING;
                 emcStatus->task.motionLine = 0;
                 emcStatus->task.readLine = 0;
             } else {

                 // executed a good line
             }

             //linuxCNC跳过行在这里处理。

             if (count++ < DEFAULT_EMC_TASK_INTERP_MAX_LEN
                 && emcStatus->task.interpState == EMC_TASK_INTERP::READING
                 && interp_list.size() <= DEFAULT_EMC_TASK_INTERP_MAX_LEN * 2/3) {
                 goto interpret_again;
             }
         }		// else not emcTaskPlanIsWait
     }		// if interp len is less than max
 }

 void readahead_waiting(void)
 {
//     // now handle call logic
//     // check for subsystems done
//     if (interp_list.size() == 0 &&
//         emcStatus->motion.traj.queue == 0 /*&&
//            emcStatus->io.status == RCS_STATUS::DONE*/)
//     // finished
//     {
//         int was_open = taskplanopen;
//         if (was_open) {
//             emcTaskPlanClose();
//             if ((emc_debug & EMC_DEBUG_INTERP) && was_open) {
//                 printf("emcTaskPlanClose() called at %s:%d\n",__FILE__, __LINE__);
//             }
//             // then resynch interpreter
//             //       emcTaskQueueTaskPlanSynchCmd();
//         } else {
//             emcStatus->task.interpState = EMC_TASK_INTERP::IDLE;
//         }
//         emcStatus->task.readLine = 0;
//     } else {
//         // still executing
//     }
 }

static int emcTaskPlan(void)
{
    NMLTYPE type;
    int retval = 0;

    // 检查是否有新命令，通过下发命令序号与已执行的命令序号是否一致来就进行判断
    if (emcCommand->serial_number != emcStatus->echo_serial_number) {
        // flag it here locally as a new command
        type = emcCommand->_type;
    } else {
        // no new command-- reset local flag
        type = 0;
    }

    // 处理任意新命令
    switch (emcStatus->task.state) {
    case EMC_TASK_STATE::OFF:
    case EMC_TASK_STATE::ESTOP:
    case EMC_TASK_STATE::ESTOP_RESET:

        // now switch on the mode
        switch (emcStatus->task.mode) {
        case EMC_OPERATION_MODE::JOG:
        case EMC_OPERATION_MODE::RAPID:
        case EMC_OPERATION_MODE::MPG:
        case EMC_OPERATION_MODE::ZRN:
        case EMC_OPERATION_MODE::EDIT:
        case EMC_OPERATION_MODE::DNC:
        case EMC_OPERATION_MODE::MEM:
        case EMC_OPERATION_MODE::MDI:

            // now switch on the command
            switch (type) {
            case 0:
            case EMC_NULL_TYPE:
                // no command
                break;
            // immediate commands
            case EMC_JOINT_UNHOME_TYPE:
            case EMC_TASK_SET_MODE_TYPE:
            case EMC_TASK_SET_STATE_TYPE:
            case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
            case EMC_TRAJ_SET_SCALE_TYPE:
            case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
            case EMC_TASK_PLAN_INIT_TYPE:
            case EMC_TASK_PLAN_OPEN_TYPE:
            case EMC_TASK_ABORT_TYPE:
            case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
            case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
            case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	        case EMC_TASK_PLAN_SYNCH_TYPE:
	        case EMC_TRAJ_PROBE_TYPE:
            case EMC_SET_DEBUG_TYPE:
            retval = emcTaskIssueCommand(emcCommand);
            break;
            // otherwise we can't handle it
            default:
            //emcOperatorError(_("command (%s) cannot be executed until the machine is out of E-stop and turned on"), emc_symbol_lookup(type));
            retval = -1;
            break;

            }			// switch (type)

        default:
            // invalid mode
            break;

        }			// switch (mode)

        break;			// case EMC_TASK_STATE_OFF,ESTOP,ESTOP_RESET

    case EMC_TASK_STATE::ON:
        /* we can do everything (almost) when the machine is on, so let's
       switch on the execution mode */
        switch (emcStatus->task.mode) {
            case EMC_OPERATION_MODE::JOG:
            case EMC_OPERATION_MODE::RAPID:
            case EMC_OPERATION_MODE::MPG:
            case EMC_OPERATION_MODE::ZRN:
                switch (type) {
                case 0:
                case EMC_NULL_TYPE:
                    // no command
                    break;
                    // immediate commands
                    case EMC_JOINT_HOME_TYPE:
                    case EMC_JOINT_UNHOME_TYPE:
                    case EMC_TASK_SET_MODE_TYPE:
                    case EMC_TASK_SET_STATE_TYPE:
                    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
                    case EMC_TRAJ_SET_SCALE_TYPE:
                    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                    case EMC_TASK_PLAN_INIT_TYPE:
                    case EMC_TASK_PLAN_OPEN_TYPE:
                    case EMC_TASK_ABORT_TYPE:
                    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
                    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
                    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
                    case EMC_SET_DEBUG_TYPE:

                    case EMC_JOG_CONT_TYPE:
                    case EMC_JOG_INCR_TYPE:
                    case EMC_JOG_ABS_TYPE:
                    case EMC_JOG_STOP_TYPE:

                    case EMC_SPINDLE_ON_TYPE:
	                case EMC_SPINDLE_OFF_TYPE:
                    retval = emcTaskIssueCommand(emcCommand);
                    break;
                    // otherwise we can't handle it
                    default:
                 //   emcOperatorError(_("command (%s) cannot be executed until the machine is out of E-stop and turned on"), emc_symbol_lookup(type));
                    retval = -1;
                    break;
                }			// switch (type) in ON, MANUAL

                break;		// case EMC_OPERATION_MODE_MANUAL

        case EMC_OPERATION_MODE::MEM:	// ON, MEM
        case EMC_OPERATION_MODE::MDI:	// ON, MDI

            switch (emcStatus->task.interpState) {//确认初始化与切换模式后为IDLE-----------
            case EMC_TASK_INTERP::IDLE:	// ON, MEM, IDLE
                switch (type) {
                case 0:
                case EMC_NULL_TYPE:
                    // no command
                    break;

                    // immediate commands
                    case EMC_JOINT_UNHOME_TYPE:
                    case EMC_TASK_SET_MODE_TYPE:
                    case EMC_TASK_SET_STATE_TYPE:
                    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
                    case EMC_TRAJ_SET_SCALE_TYPE:
                    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                    case EMC_TASK_PLAN_INIT_TYPE:
                    case EMC_TASK_PLAN_OPEN_TYPE:
                    case EMC_SET_DEBUG_TYPE:

                    case EMC_TASK_PLAN_RUN_TYPE:

                    case EMC_TRAJ_PAUSE_TYPE:
                    case EMC_TRAJ_RESUME_TYPE:
                    case EMC_TRAJ_ABORT_TYPE:
                    case EMC_TASK_ABORT_TYPE:

                    retval = emcTaskIssueCommand(emcCommand);
                    break;


//                case EMC_TASK_PLAN_STEP_TYPE:
//                    // handles case where first action is to step the program
//                    taskPlanRunCmd.line = 0;	// run from start
//                    /*! \todo FIXME-- can have GUI set this; send a run instead of a
//               step */
//                    retval = emcTaskIssueCommand(&taskPlanRunCmd);
//                    if(retval != 0) break;
//                    emcTrajPause();
//                    if (emcStatus->task.interpState != EMC_TASK_INTERP::PAUSED) {
//                        interpResumeState = emcStatus->task.interpState;
//                    }
//                    emcStatus->task.interpState = EMC_TASK_INTERP::PAUSED;
//                    emcStatus->task.task_paused = 1;
//                    retval = 0;
//                    break;

                default:
                    // //EMC_OPERATION_MODE_AUTO(2) && EMC_TASK_INTERP_IDLE(1)
                    // if ( allow_while_idle_type() ) {
                    //     retval = emcTaskIssueCommand(emcCommand);
                    //     break;
                    // }
                    // emcOperatorError(0, _
                    //                  ("can't do that (%s) in auto mode with the interpreter idle"),
                    //                  emc_symbol_lookup(type));
                    // retval = -1;
                    break;

                }		// switch (type) in ON, MEM, IDLE

                break;		// EMC_TASK_INTERP_IDLE

            case EMC_TASK_INTERP::READING:	// ON, MEM, READING
                switch (type) {
                case 0:
                case EMC_NULL_TYPE:
                    // no command
                    break;
                // immediate commands
                case EMC_JOINT_UNHOME_TYPE:
                case EMC_TASK_SET_MODE_TYPE:
                case EMC_TASK_SET_STATE_TYPE:
                case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
                case EMC_TRAJ_SET_SCALE_TYPE:
                case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                case EMC_TASK_PLAN_INIT_TYPE:
                case EMC_TASK_PLAN_OPEN_TYPE:
                case EMC_SET_DEBUG_TYPE:

                case EMC_TRAJ_PAUSE_TYPE:
                case EMC_TRAJ_RESUME_TYPE:
                case EMC_TRAJ_ABORT_TYPE:

                case EMC_TASK_PLAN_PAUSE_TYPE:
                case EMC_TASK_PLAN_REVERSE_TYPE:
                case EMC_TASK_PLAN_FORWARD_TYPE:
                case EMC_TASK_PLAN_RESUME_TYPE:
                case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:


                retval = emcTaskIssueCommand(emcCommand);
                break;
//                case EMC_TASK_PLAN_STEP_TYPE:
//                    stepping = 1;	// set stepping mode in case it's not
//                    steppingWait = 0;	// clear the wait
//                    break;

                    // otherwise we can't handle it
                default:
                    // emcOperatorError(0, _
                    //                  ("can't do that (%s) in auto mode with the interpreter reading"),
                    //                  emc_symbol_lookup(type));
                    retval = -1;
                    break;

                }		// switch (type) in ON, MEM, READING

                // handle interp readahead logic
                readahead_reading();

                break;		// EMC_TASK_INTERP_READING

            case EMC_TASK_INTERP::PAUSED:	// ON, MEM, PAUSED
                switch (type) {
                case 0:
                case EMC_NULL_TYPE:
                    // no command
                    break;

                    // immediate commands
                    case EMC_JOINT_UNHOME_TYPE:
                    case EMC_TASK_SET_MODE_TYPE:
                    case EMC_TASK_SET_STATE_TYPE:
                    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
                    case EMC_TRAJ_SET_SCALE_TYPE:
                    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                    case EMC_TASK_PLAN_INIT_TYPE:
                    case EMC_TASK_PLAN_OPEN_TYPE:
                    case EMC_SET_DEBUG_TYPE:


                    retval = emcTaskIssueCommand(emcCommand);
                    break;
                case EMC_TASK_PLAN_STEP_TYPE:
//                    stepping = 1;
//                    steppingWait = 0;
//                    if (emcStatus->motion.traj.paused &&
//                        emcStatus->motion.traj.queue > 0) {
//                        // there are pending motions paused; step them
//                        emcTrajStep();
//                    } else {
//                        emcStatus->task.interpState = (enum EMC_TASK_INTERP) interpResumeState;
//                    }
//                    emcStatus->task.task_paused = 1;
                    break;

                    // otherwise we can't handle it
                default:
                    // emcOperatorError(0, _
                    //         ，         ("can't do that (%s) in auto mode with the interpreter paused"),
                    //         EMC_TASK_PLAN_STEP_TYPE         emc_symbol_lookup(type));
                   // retval = -1;
                    break;

                }		// switch (type) in ON, MEM, PAUSED

                break;		// EMC_TASK_INTERP_PAUSED

            case EMC_TASK_INTERP::WAITING://（readahead_waiting）
                // interpreter ran to end
                // handle input commands
                switch (type) {
                case 0:
                case EMC_NULL_TYPE:
                    // no command
                    break;
                    // immediate commands
                case EMC_JOINT_UNHOME_TYPE:
                case EMC_TASK_SET_MODE_TYPE:
                case EMC_TASK_SET_STATE_TYPE:
                case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
                case EMC_TRAJ_SET_SCALE_TYPE:
                case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                case EMC_TASK_PLAN_INIT_TYPE:
                case EMC_TASK_PLAN_OPEN_TYPE:
                case EMC_SET_DEBUG_TYPE:

                case EMC_TRAJ_PAUSE_TYPE:
                case EMC_TRAJ_RESUME_TYPE:
                case EMC_TRAJ_ABORT_TYPE:

                case EMC_TASK_PLAN_PAUSE_TYPE:
                case EMC_TASK_PLAN_REVERSE_TYPE:
                case EMC_TASK_PLAN_FORWARD_TYPE:
                case EMC_TASK_PLAN_RESUME_TYPE:
                case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:


                retval = emcTaskIssueCommand(emcCommand);

//                case EMC_TASK_PLAN_STEP_TYPE:
//                    stepping = 1;	// set stepping mode in case it's not
//                    steppingWait = 0;	// clear the wait
//                    break;

                    // otherwise we can't handle it
                default:
                    // emcOperatorError(0, _
                    //                  ("can't do that (%s) in auto mode with the interpreter waiting"),
                    //                  emc_symbol_lookup(type));
                    retval = -1;
                    break;

                }		// switch (type) in ON, MEM, WAITING

                // handle interp readahead logic
                readahead_waiting();

                break;		// end of case EMC_TASK_INTERP_WAITING

            default:
                // coding error
                // printf_error("invalid mode(%d)", emcStatus->task.mode);
                retval = -1;
                break;

            }			// switch (mode) in ON, MEM

            break;		// case EMC_OPERATION_MODE_AUTO


        default:
            break;

        }			// switch (mode)

        break;			// case EMC_TASK_STATE_ON

    default:
        break;

    }				// switch (task.state)

    return retval;
}


void taskmain()
{
    int taskPlanError = 0;
    int taskExecuteError = 0;
    done = 0;

    emcStatus = new EMC_STAT;
    emcCommand = new RCS_CMD_MSG;


    while(!done)
    {
        //0、遍历接口生成事件
        emctaskEventScan();
        // 1、读取命令,人机交互的命令
        if (0 != kevent_get(emcCommand)) {
            // got a new command, so clear out errors
            taskPlanError = 0;
            taskExecuteError = 0;
        }

        // 2、任务规划
        if (0 != emcTaskPlan()) {
            taskPlanError = 1;
        }

        // 3、任务执行
       if (0 != emcTaskExecute()) {
           taskExecuteError = 1;
       }

        // 4、更新状态与报警处理
        // update subordinate status

        //emcIoUpdate(&emcStatus->io);
      //  emcMotionUpdate(&emcStatus->motion);

        // do task
        emcStatus->command_type = emcCommand->_type;
        emcStatus->echo_serial_number = emcCommand->serial_number;

        // if (taskPlanError || taskExecuteError ||
        //     emcStatus->task.execState == EMC_TASK_EXEC::TASK_STATE_ERROR ||
        //     emcStatus->motion.status == RCS_STATUS::ERROR ||
        //     emcStatus->io.status == RCS_STATUS::ERROR) {
        //     emcStatus->status = RCS_STATUS::ERROR;
        //     emcStatus->task.status = RCS_STATUS::ERROR;
        // } else if (!taskPlanError && !taskExecuteError &&
        //            emcStatus->task.execState == EMC_TASK_EXEC::TASK_STATE_DONE &&
        //            emcStatus->motion.status == RCS_STATUS::DONE &&
        //            emcStatus->io.status == RCS_STATUS::DONE &&
        //            mdi_execute_queue.len() == 0 &&
        //            getEmcCommandDataCount_C() == 0 &&
        //            emcTaskCommand == 0 &&
        //            emcStatus->task.interpState == EMC_TASK_INTERP::IDLE) {
        //     emcStatus->status = RCS_STATUS::DONE;
        //     emcStatus->task.status = RCS_STATUS::DONE;
        // } else {
        emcStatus->status = RCS_STATUS::RCS_EXEC;
        // }

        
    }

}

 int emcTaskAbort()
 {
//     emcMotionAbort();

//     // clear out the pending command
//     emcTaskCommand.flag = 0;
//     interp_list.clear();

//     // clear out the interpreter state
//     emcStatus->task.interpState = EMC_TASK_INTERP::IDLE;
//     emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
//     emcStatus->task.task_paused = 0;
//     emcStatus->task.motionLine = 0;
//     emcStatus->task.readLine = 0;
//     emcStatus->task.command[0] = 0;
//     emcStatus->task.callLevel = 0;

//     stepping = 0;
//     steppingWait = 0;

//     // now queue up command to resynch interpreter
//     //emcTaskQueueTaskPlanSynchCmd();

//     // without emcTaskPlanClose(), a new run command resumes at
//     // aborted line-- feature that may be considered later
//     {
//         int was_open = taskplanopen;
//         emcTaskPlanClose();
//         emcTaskPlanReset();  // Flush any unflushed segments
////         if (emc_debug & EMC_DEBUG_INTERP && was_open) {
////             printf("emcTaskPlanClose() called at %s:%d\n", __FILE__,
////                       __LINE__);
////         }
//     }

     return 0;
 }

 int emcIoAbort()
 {
    //用于发送让PLC停止输出的信号。
 }

// 立即执行，不检查条件
static int emcTaskIssueCommand(RCS_CMD_MSG * cmd)
{
    int retval = 0;
    int execRetval = 0;
   // static char remote_tmpfilename[LINELEN];   // path to temporary file received from remote process

   if (0 == cmd) {
      // if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
           printf("emcTaskIssueCommand() null command\n");
      // }
       return 0;
   }
//    if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
//        // printf("Issuing %s -- \t (%s)\n", emcSymbolLookup(cmd->_type),
//        //           emcCommandBuffer->msg2str(cmd));
//    }
    switch (cmd->_type) {
        // general commands

//    case EMC_OPERATOR_ERROR_TYPE:
//        //retval = emcOperatorError("%s", ((EMC_OPERATOR_ERROR *) cmd)->error);
//        break;

//    case EMC_OPERATOR_TEXT_TYPE:
//        // retval = emcOperatorText("%s", ((EMC_OPERATOR_TEXT *) cmd)->text);
//        break;

//    case EMC_OPERATOR_DISPLAY_TYPE:
//        // retval = emcOperatorDisplay("%s", ((EMC_OPERATOR_DISPLAY *) cmd)->display);
//        break;

//    case EMC_SYSTEM_CMD_TYPE:
//        //   retval = emcSystemCmd(((EMC_SYSTEM_CMD *) cmd)->string);
//        break;

//        // joint commands

   case EMC_JOINT_HOME_TYPE:
      // retval = emcJointHome(cmd.joint);
       break;

   case EMC_JOINT_UNHOME_TYPE:
      // retval = emcJointUnhome(cmd.joint);
       break;

//    case EMC_JOG_CONT_TYPE:
//        jog_cont_msg = (EMC_JOG_CONT *) cmd;
//        retval = emcJogCont(jog_cont_msg->parent.joint_or_axis,
//                            jog_cont_msg->vel,
//                            jog_cont_msg->jjogmode);
//        break;

//    case EMC_JOG_STOP_TYPE:
//        jog_stop_msg = (EMC_JOG_STOP *) cmd;
//        retval = emcJogStop(jog_stop_msg->parent.joint_or_axis,
//                            jog_stop_msg->jjogmode);
//        break;

//    case EMC_JOG_INCR_TYPE:
//        jog_incr_msg = (EMC_JOG_INCR *) cmd;
//        retval = emcJogIncr(jog_incr_msg->parent.joint_or_axis,
//                            jog_incr_msg->incr,
//                            jog_incr_msg->vel,
//                            jog_incr_msg->jjogmode);
//        break;

//    case EMC_JOG_ABS_TYPE:
//        jog_abs_msg = (EMC_JOG_ABS *) cmd;
//        retval = emcJogAbs(jog_abs_msg->parent.joint_or_axis,
//                           jog_abs_msg->pos,
//                           jog_abs_msg->vel,
//                           jog_abs_msg->jjogmode);
//        break;

//    case EMC_JOINT_HALT_TYPE:
//        joint_halt_msg = (EMC_JOINT_HALT *) cmd;
//        retval = emcJointHalt(joint_halt_msg->parent.joint);
//        break;

//    case EMC_JOINT_OVERRIDE_LIMITS_TYPE:
//        joint_lim_msg = (EMC_JOINT_OVERRIDE_LIMITS *) cmd;
//        retval = emcJointOverrideLimits(joint_lim_msg->parent.joint);
//        break;

//    case EMC_JOINT_LOAD_COMP_TYPE:
//        joint_load_comp_msg = (EMC_JOINT_LOAD_COMP *) cmd;
//        retval = emcJointLoadComp(joint_load_comp_msg->parent.joint,
//                                  joint_load_comp_msg->file,
//                                  joint_load_comp_msg->type);
//        break;

//        // traj commands

//    case EMC_TRAJ_SET_SCALE_TYPE:
//        emcTrajSetScaleMsg = (EMC_TRAJ_SET_SCALE *) cmd;
//        retval = emcTrajSetScale(emcTrajSetScaleMsg->scale);
//        break;

//    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
//        emcTrajSetRapidScaleMsg = (EMC_TRAJ_SET_RAPID_SCALE *) cmd;
//        retval = emcTrajSetRapidScale(emcTrajSetRapidScaleMsg->scale);
//        break;

   case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
        // retval = emcTrajSetSpindleScale(cmd->spindle,
        //                                 cmd->scale);
       break;

//    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
//        retval = emcTrajSetFOEnable(((EMC_TRAJ_SET_FO_ENABLE *) cmd)->mode);  // feed override enable/disable
//        break;

//    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
//        retval = emcTrajSetFHEnable(((EMC_TRAJ_SET_FH_ENABLE *) cmd)->mode); //feed hold enable/disable
//        break;

//    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
//        retval = emcTrajSetSOEnable(((EMC_TRAJ_SET_SO_ENABLE *) cmd)->mode); //spindle speed override enable/disable
//        break;



//    case EMC_TRAJ_PAUSE_TYPE:
//        emcStatus->task.task_paused = 1;
//        retval = emcTrajPause();
//        break;

//    case EMC_TRAJ_RESUME_TYPE:
//        emcStatus->task.task_paused = 0;
//        retval = emcTrajResume();
//        break;

//    case EMC_TRAJ_ABORT_TYPE:
//        retval = emcTrajAbort();
//        break;



//    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
//        retval = emcTrajClearProbeTrippedFlag();
//        break;


//    case EMC_SET_DEBUG_TYPE:
//        // /* set the debug level here */
//        // emc_debug = ((EMC_SET_DEBUG *) cmd)->debug;
//        // /* and motion */
//        // emcMotionSetDebug(emc_debug);
//        // /* and reflect it in the status-- this isn't updated continually */
//        // emcStatus->debug = emc_debug;
//        break;

//        // unimplemented ones

//        // IO commands

//    case EMC_SPINDLE_SPEED_TYPE:
//        spindle_speed_msg = (EMC_SPINDLE_SPEED *) cmd;
//        retval = emcSpindleSpeed(spindle_speed_msg->spindle, spindle_speed_msg->speed,
//                                 spindle_speed_msg->factor, spindle_speed_msg->xoffset);
//        break;

//    case EMC_SPINDLE_ORIENT_TYPE:
//        spindle_orient_msg = (EMC_SPINDLE_ORIENT *) cmd;
//        retval = emcSpindleOrient(spindle_orient_msg->spindle, spindle_orient_msg->orientation,
//                                  spindle_orient_msg->mode);
//        break;

//    case EMC_SPINDLE_ON_TYPE:
//        spindle_on_msg = (EMC_SPINDLE_ON *) cmd;
//        retval = emcSpindleOn(spindle_on_msg->spindle, spindle_on_msg->speed,
//                              spindle_on_msg->factor, spindle_on_msg->xoffset, spindle_on_msg->wait_for_spindle_at_speed);
//        break;

//    case EMC_SPINDLE_OFF_TYPE:
//        spindle_off_msg = (EMC_SPINDLE_OFF *) cmd;
//        retval = emcSpindleOff(spindle_off_msg->spindle);
//        break;

//    case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
//        spindle_brake_engage_msg = (EMC_SPINDLE_BRAKE_ENGAGE *) cmd;
//        retval = emcSpindleBrakeEngage(spindle_brake_engage_msg->spindle);
//        break;

//        // task commands

//    case EMC_TASK_ABORT_TYPE:
//        // // abort everything
//        // emcTaskAbort();
//        // // KLUDGE call motion abort before state restore to make absolutely sure no
//        // // stray restore commands make it down to motion
//        // emcMotionAbort();
//        // // Then call state restore to update the interpreter
//        // emcTaskStateRestore();
//        // emcIoAbort(EMC_ABORT::TASK_ABORT);
//        // for (int s = 0; s < emcStatus->motion.traj.spindles; s++) emcSpindleAbort(s);
//        // mdi_execute_abort();
//        // emcAbortCleanup(EMC_ABORT::TASK_ABORT);
//        // retval = 0;
//        break;

//        // mode and state commands

   case EMC_TASK_SET_MODE_TYPE:
       // if (emcStatus->task.mode == EMC_OPERATION_MODE::MEM &&
       //     emcStatus->task.interpState != EMC_TASK_INTERP::IDLE &&
       //     cmd->mode != EMC_OPERATION_MODE::MEM) {
       //     emcOperatorError(("Can't switch mode while mode is MEM and interpreter is not IDLE"));
       // } else { // we can honour the modeswitch
       //     if (cmd->mode == EMC_OPERATION_MODE::MANUAL &&
       //         emcStatus->task.mode != EMC_OPERATION_MODE::MANUAL) {
       //         // leaving auto or mdi mode for manual

       //         /*! \todo FIXME-- duplicate code for abort,
       //     also near end of main, when aborting on subordinate errors,
       //     and in emcTaskExecute() */

       //          // abort motion
       //          emcTaskAbort();
       //          mdi_execute_abort();

       //          // without emcTaskPlanClose(), a new run command resumes at
       //          // aborted line-- feature that may be considered later
       //          {
       //              int was_open = taskplanopen;
       //              emcTaskPlanClose();
       //              if (emc_debug & EMC_DEBUG_INTERP && was_open) {
       //                  printf("emcTaskPlanClose() called at %s:%d\n",
       //                            __FILE__, __LINE__);
       //              }
       //          }

       //          // clear out the pending command
       //          emcTaskCommand.flag = 0;
       //          interp_list.clear();
       //          emcStatus->task.currentLine = 0;

       //          // clear out the interpreter state
       //          emcStatus->task.interpState = EMC_TASK_INTERP::IDLE;
       //          emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
       //          stepping = 0;
       //          steppingWait = 0;

       //          // now queue up command to resynch interpreter
       //          emcTaskQueueTaskPlanSynchCmd();
       //     }
       //     retval = emcTaskSetMode(cmd->mode);
       // }
       break;

    case EMC_TASK_SET_STATE_TYPE:
        retval = emcTaskSetState(cmd->state);
        break;

        // interpreter commands

    case EMC_TASK_PLAN_CLOSE_TYPE:
        // retval = emcTaskPlanClose();
        // if (retval > INTERP_MIN_ERROR) {
        //     emcOperatorError(_("failed to close file"));
        //     retval = -1;
        // } else {
        //     retval = 0;
        // }
        /* delete temporary copy of remote file if it exists */
//        if(remote_tmpfilename[0])
//            unlink(remote_tmpfilename);
//        break;

    case EMC_TASK_PLAN_OPEN_TYPE:

//        /* receive file in chunks from remote process via open_msg->remote_buffer */
//        if(cmd->remote_filesize > 0) {
//            static size_t received;             // amount of bytes of file received, yet
//            static int fd;                      // temporary file

//            /* got empty chunk? */
//            if(cmd->remote_buffersize == 0) {
//                printf_error("EMC_TASK_PLAN_OPEN received empty chunk from remote process.\n");
//                retval = -1;
//                break;
//            }

//            /* this chunk belongs to a new file? */
//            if(received == 0) {
//                /* create new tempfile */
//                //                 printf(remote_tmpfilename, LINELEN, EMC2_TMP_DIR "/%s.XXXXXX", basename(open_msg->file));
//                if((fd = mkstemp(remote_tmpfilename)) < 0) {
//                    printf_error("mkstemp(%s) error: %s", remote_tmpfilename, strerror(errno));
//                    retval = -1;
//                    break;
//                }
//            }

//            /* write chunk to tempfile */
//            size_t bytes_written = write(fd, cmd->remote_buffer, cmd->remote_buffersize);
//            if(bytes_written < cmd->remote_buffersize) {
//                printf_error("fwrite(%s) error: %s", remote_tmpfilename, strerror(errno));
//                retval = -1;
//                break;
//            }
//            /* record data written */
//            received += bytes_written;

//            /* not the last chunk? */
//            if(received < cmd->remote_filesize) {
//                /* we're done here for now */
//                retval = 0;
//                break;
//            }
//            /* all chunks received - reset byte counter */
//            received = 0;
//            /* close file */
//            close(fd);
//            /* change filename to newly written local tmp file */
//            rtapi_strxcpy(cmd->file, remote_tmpfilename);
//        }

//        /* open local file */
//        retval = emcTaskPlanOpen(cmd->file);
//        if (retval > INTERP_MIN_ERROR) {
//            retval = -1;
//        }
//        if (-1 == retval) {
//            emcOperatorError(_("can't open %s"), cmd->file);
//        } else {
//            rtapi_strxcpy(emcStatus->task.file, cmd->file);
//            retval = 0;
//        }
        break;

//    case EMC_TASK_PLAN_EXECUTE_TYPE:
//         stepping = 0;
//         steppingWait = 0;
//         execute_msg = (EMC_TASK_PLAN_EXECUTE *) cmd;
//         if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before MDI
//             emcOperatorError(_("Can't issue MDI command when not homed"));
//             retval = -1;
//             break;
//         }
//        if (emcStatus->task.mode != EMC_OPERATION_MODE::MDI) {
//                 emcOperatorError(_("Must be in MDI mode to issue MDI command"));
//                 retval = -1;
//                 break;
//        }
//         // track interpState also during MDI - it might be an oword sub call
//         emcStatus->task.interpState = EMC_TASK_INTERP::READING;

//         if (execute_msg->command[0] != 0) {
//             char * command = execute_msg->command;
//             if (command[0] == (char) 0xff) {
//                 // Empty command received. Consider it is NULL
//                 command = NULL;
//             } else {
//                 // record initial MDI command
//                 rtapi_strxcpy(emcStatus->task.command, execute_msg->command);
//             }

//             int level = emcTaskPlanLevel();
//        if (emcStatus->task.mode == EMC_OPERATION_MODE::MDI) {
//                     if (mdi_execute_level < 0)
//                         mdi_execute_level = level;
//        }

//             execRetval = emcTaskPlanExecute(command, 0);

//             level = emcTaskPlanLevel();

//        if (emcStatus->task.mode == EMC_OPERATION_MODE::MDI) {
//             if (mdi_execute_level == level) {
//                 mdi_execute_level = -1;
//             } else if (level > 0) {
//                 // Still insude call. Need another execute(0) call
//                 // but only if we didn't encounter an error
//                 if (execRetval == INTERP_ERROR) {
//                     mdi_execute_next = 0;
//                 } else {
//                     mdi_execute_next = 1;
//                 }
//             }
//        }
//             switch (execRetval) {

//             case INTERP_EXECUTE_FINISH:
//                 // Flag MDI wait
//                 mdi_execute_wait = 1;
//                 // need to flush execution, so signify no more reading
//                 // until all is done
//                 emcTaskPlanSetWait();
//                 // and resynch the interpreter WM
//                 emcTaskQueueTaskPlanSynchCmd();
//                 // it's success, so retval really is 0
//                 retval = 0;
//                 break;

//             case INTERP_ERROR:
//                 // emcStatus->task.interpState =  EMC_TASK_INTERP::WAITING;
//                 interp_list.clear();
//                 // abort everything
//                 emcTaskAbort();
//                 emcIoAbort(EMC_ABORT::INTERPRETER_ERROR_MDI);
//                 for (int s = 0; s < emcStatus->motion.traj.spindles; s++) emcSpindleAbort(s);
//                 mdi_execute_abort(); // sets emcStatus->task.interpState to  EMC_TASK_INTERP::IDLE
//                 emcAbortCleanup(EMC_ABORT::INTERPRETER_ERROR_MDI, "interpreter error during MDI");
//                 retval = -1;
//                 break;

//             case INTERP_EXIT:
//             case INTERP_ENDFILE:
//             case INTERP_FILE_NOT_OPEN:
//                 // this caused the error msg on M2 in MDI mode - execRetval == INTERP_EXIT which is would be ok (I think). mah
//                 retval = -1;
//                 break;

//             default:
//                 // other codes are OK
//                 retval = 0;
//             }
//         }
//        break;

//    case EMC_TASK_PLAN_RUN_TYPE:
//         if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before Auto
//             emcOperatorError(_("Can't run a program when not homed"));
//             retval = -1;
//             break;
//         }
//         stepping = 0;
//         steppingWait = 0;
//         if (!taskplanopen && emcStatus->task.file[0] != 0) {
//             emcTaskPlanOpen(emcStatus->task.file);
//         }
//         run_msg = (EMC_TASK_PLAN_RUN *) cmd;
//         programStartLine = run_msg->line;
//         emcStatus->task.interpState = EMC_TASK_INTERP::READING;
//         emcStatus->task.task_paused = 0;
//         retval = 0;
//        break;

//    case EMC_TASK_PLAN_PAUSE_TYPE:
//        emcTrajPause();
//        if (emcStatus->task.interpState != EMC_TASK_INTERP::PAUSED) {
//            interpResumeState = emcStatus->task.interpState;
//        }
//        emcStatus->task.interpState = EMC_TASK_INTERP::PAUSED;
//        emcStatus->task.task_paused = 1;
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
//         if (GET_OPTIONAL_PROGRAM_STOP() == ON) {
//             emcTrajPause();
//             if (emcStatus->task.interpState != EMC_TASK_INTERP::PAUSED) {
//                 interpResumeState = emcStatus->task.interpState;
//             }
//             emcStatus->task.interpState = EMC_TASK_INTERP::PAUSED;
//             emcStatus->task.task_paused = 1;
//         }
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_REVERSE_TYPE:
//        emcTrajReverse();
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_FORWARD_TYPE:
//        emcTrajForward();
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_RESUME_TYPE:
//        emcTrajResume();
//        // emcStatus->task.interpState = interpResumeState;
//        emcStatus->task.task_paused = 0;
//        stepping = 0;
//        steppingWait = 0;
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_END_TYPE:
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_INIT_TYPE:
//        retval = emcTaskPlanInit();
//        break;

//    case EMC_TASK_PLAN_SYNCH_TYPE:
//        // retval = emcTaskPlanSynch();
//        // if (retval > INTERP_MIN_ERROR) {
//        //     retval = -1;
//        // }
//        break;

//    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
//        os_msg = (EMC_TASK_PLAN_SET_OPTIONAL_STOP *) cmd;
//        // emcTaskPlanSetOptionalStop(os_msg->state);
//        retval = 0;
//        break;

//    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
//        bd_msg = (EMC_TASK_PLAN_SET_BLOCK_DELETE *) cmd;
//        //  emcTaskPlanSetBlockDelete(bd_msg->state);
//        retval = 0;
//        break;

    default:
        // unrecognized command
        // if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
        //     printf_error("ignoring issue of unknown command %d:%s\n",
        //                     (int)cmd->_type, emc_symbol_lookup(cmd->_type));
        // }
        retval = 0;		// don't consider this an error
        break;
    }

    // if (retval == -1) {
    //     if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
    //         printf_error("error executing command %d:%s\n", (int)cmd->_type,
    //                         emc_symbol_lookup(cmd->_type));
    //     }
    // }
    /* debug */
    // if ((emc_debug & EMC_DEBUG_TASK_ISSUE) && retval) {
    //     printf("emcTaskIssueCommand() returning: %d\n", retval);
    // }
    return retval;
}

static int emcTaskIssueExecCommand(MotionDataSend * cmd)
{


}
static EMC_TASK_EXEC emcTaskCheckPreconditions(MotionDataSend * cmd)
{
    if (0 == cmd) {
        return EMC_TASK_EXEC::TASK_STATE_DONE;
    }

    switch (cmd->_type) {
    // operator messages, if queued, will go out when everything before
    // them is done
    case EMC_TRAJ_PROBE_TYPE:	// prevent blending of this
    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:	// and this
    case EMC_AUX_INPUT_WAIT_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    case EMC_TRAJ_LINEAR_MOVE_TYPE:
    case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
    case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_IO;
        break;

    case EMC_TRAJ_SET_ROTATION_TYPE:
        // this applies the program origin after previous motions
        return EMC_TASK_EXEC::WAITING_FOR_MOTION;
        break;

    case EMC_SPINDLE_SPEED_TYPE:
    case EMC_SPINDLE_ON_TYPE:
    case EMC_SPINDLE_OFF_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    case EMC_TASK_PLAN_PAUSE_TYPE:
    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
        /* pause on the interp list is queued, so wait until all are done */
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    case EMC_TASK_PLAN_END_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    case EMC_TASK_PLAN_INIT_TYPE:
    case EMC_TASK_PLAN_RUN_TYPE:
    case EMC_TASK_PLAN_SYNCH_TYPE:
    case EMC_TASK_PLAN_EXECUTE_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    case EMC_TRAJ_DELAY_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO;
        break;

    default:
        // // unrecognized command
        // if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
        //     printf_error("preconditions: unrecognized command %d:%s\n",
        //                     (int)cmd->type, emc_symbol_lookup(cmd->type));
        // }
        return EMC_TASK_EXEC::TASK_STATE_ERROR;
        break;
    }

    return EMC_TASK_EXEC::TASK_STATE_DONE;
}

static EMC_TASK_EXEC emcTaskCheckPostconditions(MotionDataSend * cmd)
{
    if (0 == cmd) {
        return EMC_TASK_EXEC::TASK_STATE_DONE;
    }

    switch (cmd->_type) {
    case EMC_OPERATOR_ERROR_TYPE:
    case EMC_OPERATOR_TEXT_TYPE:
    case EMC_OPERATOR_DISPLAY_TYPE:
        return EMC_TASK_EXEC::TASK_STATE_DONE;
        break;

    case EMC_TRAJ_LINEAR_MOVE_TYPE:
    case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
    case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
    case EMC_TRAJ_SET_ROTATION_TYPE:
    case EMC_TRAJ_PROBE_TYPE:
    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
    case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
        return EMC_TASK_EXEC::TASK_STATE_DONE;
        break;

    case EMC_SPINDLE_SPEED_TYPE:
    case EMC_SPINDLE_ON_TYPE:
    case EMC_SPINDLE_OFF_TYPE:
        return EMC_TASK_EXEC::TASK_STATE_DONE;
        break;

    case EMC_TASK_PLAN_RUN_TYPE:
    case EMC_TASK_PLAN_PAUSE_TYPE:
    case EMC_TASK_PLAN_END_TYPE:
    case EMC_TASK_PLAN_INIT_TYPE:
    case EMC_TASK_PLAN_SYNCH_TYPE:
    case EMC_TASK_PLAN_EXECUTE_TYPE:
    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
        return EMC_TASK_EXEC::TASK_STATE_DONE;
        break;

    case EMC_TRAJ_DELAY_TYPE:
    case EMC_AUX_INPUT_WAIT_TYPE:
        return EMC_TASK_EXEC::WAITING_FOR_DELAY;
        break;

    default:
        // unrecognized command
        // if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
        //     printf_error("postconditions: unrecognized command %d:%s\n",
        //                     (int)cmd->type, emc_symbol_lookup(cmd->type));
        // }
        return EMC_TASK_EXEC::TASK_STATE_DONE;
        break;
    }
    return EMC_TASK_EXEC::TASK_STATE_DONE; // unreached
}

/*
  STEPPING_CHECK() is a macro that prefaces a switch-case with a check
  for stepping. If stepping is active, it waits until the step has been
  given, then falls through to the rest of the case statement.
*/

#define STEPPING_CHECK()                                                   \
if (stepping) {                                                            \
        if (! steppingWait) {                                                    \
            steppingWait = 1;                                                      \
            steppedLine = emcStatus->task.currentLine;                             \
    }                                                                        \
        else {                                                                   \
            if (emcStatus->task.currentLine != steppedLine) {                      \
                break;                                                               \
        }                                                                      \
    }                                                                        \
}

// executor function
static int emcTaskExecute(void)
{
    int retval = 0;
    int status;			// status of child from EMC_SYSTEM_CMD
    //pid_t pid;			// pid returned from waitpid()

    // first check for an abandoned system command and abort it
    // if (emcSystemCmdPid != 0 &&
    //     emcStatus->task.execState !=
    //         EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD) {
    //     if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
    //         printf("emcSystemCmd: abandoning process %d\n",
    //                   emcSystemCmdPid);
    //     }
    //     kill(emcSystemCmdPid, SIGINT);
    //     emcSystemCmdPid = 0;
    // }

    switch (emcStatus->task.execState) {
    case EMC_TASK_EXEC::TASK_STATE_ERROR:

        /*! \todo FIXME-- duplicate code for abort,
       also near end of main, when aborting on subordinate errors,
       and in emcTaskIssueCommand() */

        // // abort everything
        emcTaskAbort();
        emcIoAbort();
        // for (int s = 0; s < emcStatus->motion.traj.spindles; s++) emcSpindleAbort(s);
        // mdi_execute_abort();

        // // without emcTaskPlanClose(), a new run command resumes at
        // // aborted line-- feature that may be considered later
        // {
        //     int was_open = taskplanopen;
        //     emcTaskPlanClose();
        //     emcTaskPlanReset();  // Flush any unflushed segments
        //     if (emc_debug & EMC_DEBUG_INTERP && was_open) {
        //         printf("emcTaskPlanClose() called at %s:%d\n", __FILE__,
        //                   __LINE__);
        //     }
        // }

        // // clear out pending command
        // emcTaskCommand.flag = 0;
        // interp_list.clear();
        // emcAbortCleanup(EMC_ABORT_TASK_EXEC_ERROR);
        // emcStatus->task.currentLine = 0;

        // // clear out the interpreter state
        // emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
        // emcStatus->task.execState = EMC_TASK_EXEC_DONE;
        // stepping = 0;
        // steppingWait = 0;

        // // now queue up command to resynch interpreter
        // emcTaskQueueCommand(&taskPlanSynchCmd);

        retval = -1;
        break;

    case EMC_TASK_EXEC::TASK_STATE_DONE:
        if (!emcStatus->motion.traj.queueFull &&
            emcStatus->task.interpState != EMC_TASK_INTERP::PAUSED) {
            if (0 == emcTaskCommand) {
                // need a new command
                motiondatasend = interp_list.takeFirst();
                emcStatus->task.currentLine = motiondatasend.lineNumber;
                emcStatus->task.callLevel = emcTaskPlanLevel();
                // and set it for all subsystems which use queued ids
               // emcTrajSetMotionId(emcStatus->task.currentLine);
                if (emcStatus->motion.traj.queueFull) {
                    emcStatus->task.execState =
                        EMC_TASK_EXEC::WAITING_FOR_MOTION_QUEUE;
                } else {
                    emcStatus->task.execState = emcTaskCheckPreconditions(emcTaskCommand);
                }
            }
            else
            {
                // have an outstanding command
                if (0 != emcTaskIssueExecCommand(emcTaskCommand)) {
                    emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_ERROR;
                    retval = -1;
                } else {
                    emcStatus->task.execState = emcTaskCheckPostconditions(emcTaskCommand);
                    emcTaskEager = 1;
                }
                emcTaskCommand = 0;	// reset it
            }
        }
        break;

    case EMC_TASK_EXEC::WAITING_FOR_MOTION_QUEUE:
        if (!emcStatus->motion.traj.queueFull) {
            if (0 != emcTaskCommand) {
                emcStatus->task.execState = emcTaskCheckPreconditions(emcTaskCommand);
                emcTaskEager = 1;
            } else {
                emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
                emcTaskEager = 1;
            }
        }
        break;

    case EMC_TASK_EXEC::WAITING_FOR_MOTION:
        if (emcStatus->motion.status == RCS_ERROR) {
            // emcOperatorError(0, "error in motion controller");
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_ERROR;
        } else if (emcStatus->motion.status == RCS_DONE) {
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
            emcTaskEager = 1;
        }
        break;

    case EMC_TASK_EXEC::WAITING_FOR_IO:
        if (emcStatus->io.status == RCS_ERROR) {
            // emcOperatorError(0, "error in IO controller");
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_ERROR;
        } else if (emcStatus->io.status == RCS_DONE) {
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
            emcTaskEager = 1;
        }
        break;

    case EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO:
        if (emcStatus->motion.status == RCS_ERROR) {
            // emcOperatorError(0, "error in motion controller");
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_ERROR;
        } else if (emcStatus->io.status == RCS_ERROR) {
            // emcOperatorError(0, "error in IO controller");
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_ERROR;
        } else if (emcStatus->motion.status == RCS_DONE &&
                   emcStatus->io.status == RCS_DONE) {
            emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
            emcTaskEager = 1;
        }
        break;


    case EMC_TASK_EXEC::WAITING_FOR_DELAY:
        // // check if delay has passed
        // emcStatus->task.delayLeft = taskExecDelayTimeout - etime();
        // if (etime() >= taskExecDelayTimeout) {
        //     emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
        //     emcStatus->task.delayLeft = 0;
        //     if (emcStatus->task.input_timeout != 0)
        //         emcStatus->task.input_timeout = 1; // timeout occurred
        //     emcTaskEager = 1;
        // }
        // // delay can be also be because we wait for an input
        // // if the index is set (not -1)
        // if (emcAuxInputWaitIndex >= 0) {
        //     switch (emcAuxInputWaitType) {
        //     case WAIT_MODE_HIGH:
        //         if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] != 0) {
        //             emcStatus->task.input_timeout = 0; // clear timeout flag
        //             emcAuxInputWaitIndex = -1;
        //             emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
        //             emcStatus->task.delayLeft = 0;
        //         }
        //         break;

        //     case WAIT_MODE_RISE:
        //         if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] == 0) {
        //             emcAuxInputWaitType = WAIT_MODE_HIGH;
        //         }
        //         break;

        //     case WAIT_MODE_LOW:
        //         if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] == 0) {
        //             emcStatus->task.input_timeout = 0; // clear timeout flag
        //             emcAuxInputWaitIndex = -1;
        //             emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
        //             emcStatus->task.delayLeft = 0;
        //         }
        //         break;

        //     case WAIT_MODE_FALL: //FIXME: implement different fall mode if needed
        //         if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] != 0) {
        //             emcAuxInputWaitType = WAIT_MODE_LOW;
        //         }
        //         break;

        //     case WAIT_MODE_IMMEDIATE:
        //         emcStatus->task.input_timeout = 0; // clear timeout flag
        //         emcAuxInputWaitIndex = -1;
        //         emcStatus->task.execState = EMC_TASK_EXEC::TASK_STATE_DONE;
        //         emcStatus->task.delayLeft = 0;
        //         break;

        //     default:
        //         emcOperatorError(0, "Unknown Wait Mode");
        //     }
        //}
        break;



    // default:
    //     // coding error
    //     if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
    //         printf("invalid execState");
    //     }
    //     retval = -1;
    //     break;
    }
    return retval;
}
