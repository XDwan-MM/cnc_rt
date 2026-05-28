#ifndef __COMP_DATA_H__
#define __COMP_DATA_H__

#include "pitch_comp.h"

#ifdef __cplusplus
extern "C" {
#endif

// 3-axis pitch compensation runtime, lookup table, and synced parameters
extern PitchCompRuntime pitch_rt[3];
extern pitch_comp_table_t pitch_table[3];
extern LIB_PAR_ACMP acmp_params[3];

// One-time init: call pitch_comp_init() for each axis, initial param sync
void comp_data_init(void);

// Per-cycle: copy pitch_comp fields from par_acmp[] to acmp_params[]
void comp_data_sync_params(void);

// Build pitch compensation tables from shared memory data for all axes
// Must be called after comp_data_sync_params()
void establish_pitch_comp_table(void);

#ifdef __cplusplus
}
#endif

#endif