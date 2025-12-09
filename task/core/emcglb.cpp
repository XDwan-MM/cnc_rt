
/********************************************************************
* Description: emcglb.c
*   Globals initialized to values in emccfg.h
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

// #include "config.h"
#include "emcglb.h"		/* these decls */
#include "emccfg.h"		/* their initial values */

unsigned emc_debug = 0;		/* initially no debug messages */

double emc_task_cycle_time = DEFAULT_EMC_TASK_CYCLE_TIME;

int emc_task_interp_max_len = DEFAULT_EMC_TASK_INTERP_MAX_LEN;

int taskplanopen = 0;
