/********************************************************************
 * @file        trace_publisher.h
 * @brief       Publish trajectory blocks for the HMI trace view
 ********************************************************************/

#ifndef CNC_TRACE_PUBLISHER_H
#define CNC_TRACE_PUBLISHER_H

#include "plan_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void trace_publisher_begin_program(int channel);
void trace_publisher_reset(int channel);
void trace_publisher_activate(int channel, const PLAN_BLOCK *block, int isMotionBlock);
void trace_publisher_publish_current(int channel);
void trace_publisher_complete(int channel);

#ifdef __cplusplus
}
#endif

#endif // CNC_TRACE_PUBLISHER_H
