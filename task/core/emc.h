#ifndef EMC_HH
#define EMC_HH

//#include "motion_types.h"
#include <stdint.h>
//#include "modal_state.h"
#include "status_msg.h"

#ifdef __cplusplus
extern "C" {
#endif
// --------------
// EMC VOCABULARY
// --------------

// NML formatting function
//extern int emcFormat(NMLTYPE type, void *buffer, CMS * cms);

// NML Symbol Lookup Function
extern const char *emc_symbol_lookup(uint32_t type);
#define emcSymbolLookup(a) emc_symbol_lookup(a)

// decls for command line args-- mains are responsible for setting these
// so that other modules can get cmd line args for ad hoc processing
extern int Argc;
extern char **Argv;

// ------------------------
// IMPLEMENTATION FUNCTIONS
// ------------------------

// implementation functions for EMC error, message types
// intended to be implemented in main() file, by writing to NML buffer

// print an error
extern int emcOperatorError(const char *fmt, ...) __attribute__((format(printf,1,2)));

// print general text
extern int emcOperatorText(const char *fmt, ...) __attribute__((format(printf,1,2)));

// print note to operator
extern int emcOperatorDisplay(const char *fmt, ...) __attribute__((format(printf,1,2)));

//// implementation functions for EMC_AXIS types

//extern int emcAxisSetMinPositionLimit(int axis, double limit);
//extern int emcAxisSetMaxPositionLimit(int axis, double limit);
//extern int emcAxisSetMaxVelocity(int axis, double vel, double ext_offset_vel);
//extern int emcAxisSetMaxAcceleration(int axis, double acc, double ext_offset_acc);
//extern double emcAxisGetMaxVelocity(int axis);
//extern double emcAxisGetMaxAcceleration(int axis);
//extern int emcAxisSetLockingJoint(int axis,int joint);

//extern int emcAxisUpdate(EMC_AXIS_STAT stat[], int numAxes);

//// implementation functions for EMC_JOINT types

//extern int emcJointSetType(int joint, unsigned char jointType);
//extern int emcJointSetUnits(int joint, double units);
//extern int emcJointSetBacklash(int joint, double backlash);
//extern int emcJointSetMinPositionLimit(int joint, double limit);
//extern int emcJointSetMaxPositionLimit(int joint, double limit);
//extern int emcJointSetMotorOffset(int joint, double offset);
//extern int emcJointSetFerror(int joint, double ferror);
//extern int emcJointSetMinFerror(int joint, double ferror);
//extern int emcJointSetHomingParams(int joint, double home, double offset, double home_vel,
//                                   double search_vel, double latch_vel,
//                                   int use_index, int encoder_does_not_reset, int ignore_limits,
//                                   int is_shared, int home_sequence, int volatile_home, int locking_indexer,
//                                   int absolute_encoder);
//extern int emcJointUpdateHomingParams(int joint, double home, double offset, int sequence);
//extern int emcJointSetMaxVelocity(int joint, double vel);
//extern int emcJointSetMaxAcceleration(int joint, double acc);

//extern int emcJointInit(int joint);
//extern int emcJointHalt(int joint);
extern int emcJointHome(int joint);
extern int emcJointUnhome(int joint);
//extern int emcJointActivate(int joint);
//extern int emcJointDeactivate(int joint);
//extern int emcJointOverrideLimits(int joint);
//extern int emcJointLoadComp(int joint, const char *file, int type);
//extern int emcJogStop(int nr, int jjogmode);
//extern int emcJogCont(int nr, double vel, int jjogmode);
//extern int emcJogIncr(int nr, double incr, double vel, int jjogmode);
//extern int emcJogAbs(int nr, double pos, double vel, int jjogmode);


//extern int emcJointUpdate(EMC_JOINT_STAT stat[], int numJoints);


//// implementation functions for EMC_SPINDLE types

//extern int emcSpindleSetParams(int spindle, double max_pos, double min_pos, double max_neg,
//                               double min_neg, double search_vel, double home_angle, int sequence, double increment);

//// implementation functions for EMC_TRAJ types

//extern double emcTrajGetLinearUnits();
//extern double emcTrajGetAngularUnits();

//extern int emcTrajInit();
//extern int emcTrajHalt();
//extern int emcTrajEnable();
//extern int emcTrajDisable();
//extern int emcTrajAbort();
//extern int emcTrajPause();
//extern int emcTrajReverse();
//extern int emcTrajForward();
//extern int emcTrajStep();
//extern int emcTrajResume();
//extern int emcTrajDelay(double delay);
//extern int emcTrajLinearMove(const EmcPose& end, int type, double vel,
//                             double ini_maxvel, double acc, int indexer_jnum);
//extern int emcTrajCircularMove(const EmcPose& end, const PM_CARTESIAN& center, const PM_CARTESIAN&
//                                                                                   normal, int turn, int type, double vel, double ini_maxvel, double acc);
//extern int emcTrajSetTermCond(int cond, double tolerance);
//extern int emcTrajSetSpindleSync(int spindle, double feed_per_revolution, bool wait_for_index);
//extern int emcTrajSetOffset(const EmcPose& tool_offset);
//extern int emcTrajSetHome(const EmcPose& home);
//extern int emcTrajClearProbeTrippedFlag();
//extern int emcTrajProbe(const EmcPose& pos, int type, double vel,
//                        double ini_maxvel, double acc, unsigned char probe_type);
//extern int emcTrajRigidTap(const EmcPose& pos, double vel, double ini_maxvel, double acc, double scale);

//extern int emcTrajUpdate(EMC_TRAJ_STAT * stat);

//// implementation functions for EMC_MOTION aggregate types

//extern int emcMotionInit();
//extern int emcMotionHalt();
//extern int emcMotionAbort();
//extern int emcMotionSetDebug(int debug);
//extern int emcMotionSetAout(unsigned char index, double start, double end,
//                            unsigned char now);
//extern int emcMotionSetDout(unsigned char index, unsigned char start,
//                            unsigned char end, unsigned char now);

//extern int emcMotionUpdate(EMC_MOTION_STAT * stat);

////extern int emcAbortCleanup(EMC_ABORT reason,const char *message = "");

//// implementation functions for EMC_TOOL types

//extern int emcToolPrepare(int tool);
//extern int emcToolLoad();
//extern int emcToolUnload();
//extern int emcToolLoadToolTable(const char *file);
//extern int emcToolSetOffset(int pocket, int toolno, const EmcPose& offset, double diameter,
//                            double frontangle, double backangle, int orientation);
//extern int emcToolSetNumber(int number);

//// implementation functions for EMC_AUX types

//extern int emcAuxEstopOn();
//extern int emcAuxEstopOff();

//// implementation functions for EMC_SPINDLE types

//extern int emcSpindleAbort(int spindle);
//extern int emcSpindleSpeed(int spindle, double speed, double factor, double xoffset);
//extern int emcSpindleOn(int spindle, double speed, double factor, double xoffset,int wait_for_atspeed = 1);
//extern int emcSpindleOrient(int spindle, double orientation, int direction);
//extern int emcSpindleOff(int spindle);
//extern int emcSpindleIncrease(int spindle);
//extern int emcSpindleDecrease(int spindle);
//extern int emcSpindleConstant(int spindle);
//extern int emcSpindleBrakeRelease(int spindle);
//extern int emcSpindleBrakeEngage(int spindle);

//extern int emcSpindleUpdate(EMC_SPINDLE_STAT stat[], int num_spindles);

//// implementation functions for EMC_COOLANT types

//// extern int emcCoolantMistOn();
//// extern int emcCoolantMistOff();
//// extern int emcCoolantFloodOn();
//// extern int emcCoolantFloodOff();

//// implementation functions for EMC_IO types

//extern int emcIoInit();
////extern int emcIoAbort(EMC_ABORT reason);

//// implementation functions for EMC aggregate types

//int emcSetMaxFeedOverride(double maxFeedScale);
//int emcSetupArcBlends(int arcBlendEnable,
//                      int arcBlendFallbackEnable,
//                      int arcBlendOptDepth,
//                      int arcBlendGapCycles,
//                      double arcBlendRampFreq,
//                      double arcBlendTangentKinkRatio);
//int emcSetProbeErrorInhibit(int j_inhibit, int h_inhibit);
//int emcGetExternalOffsetApplied(void);
//EmcPose emcGetExternalOffsets(void);

extern int emcUpdate(EMC_STAT * stat);
// full EMC status
extern EMC_STAT *emcStatus;

// EMC IO status
//extern EMC_IO_STAT *emcIoStatus;

// EMC MOTION status
//extern EMC_MOTION_STAT *emcMotionStatus;

// values for EMC_JOINT_SET_JOINT, jointType
enum EmcJointType {
    EMC_LINEAR             = 1,
    EMC_ANGULAR            = 2,
};

/**
 * Set the units conversion factor.
 * @see EMC_JOINT_SET_INPUT_SCALE
 */
//using EmcLinearUnits = double;
//using EmcAngularUnits = double;

#ifdef __cplusplus
}
#endif

#endif // EMC_H
