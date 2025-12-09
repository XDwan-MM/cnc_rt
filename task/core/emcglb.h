#ifndef EMCGLB_H
#define EMCGLB_H

//#include "greecnc.h"           /* LINELEN */
#include "math.h"		/* M_PI */

#ifdef __cplusplus
extern "C" {
#endif
extern unsigned emc_debug;

extern double emc_task_cycle_time;

extern int emc_task_interp_max_len;

extern int taskplanopen;

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif

#endif // EMCGLB_H
