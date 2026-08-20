#ifndef CNC_SERVO_SAMPLE_EVENTS_H
#define CNC_SERVO_SAMPLE_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Task event handlers call these hooks; they never start machine motion. */
void cncServoSampleNotifyReset(void);
void cncServoSampleNotifyCycleStart(void);

#ifdef __cplusplus
}
#endif

#endif
