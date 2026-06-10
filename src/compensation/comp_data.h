#ifndef __COMP_DATA_H__
#define __COMP_DATA_H__

#include "pitch_comp.h"
#include "backlash_comp.h"
#include "perp_comp.h"

#ifdef __cplusplus
extern "C" {
#endif

// 螺距误差补偿数据
extern PitchCompState pitch_comp_state[3];       // XYZ轴 螺距补偿状态机
extern BacklashCompState backlash_comp_state[3]; // XYZ轴 反向间隙补偿状态机
extern PerpCompState perp_comp_state[3];         // XYZ轴 垂直度补偿状态机

extern pitch_comp_table_t pitch_comp_table[3];      // XYZ轴 螺距补偿表
extern PitchTableStatus pitch_comp_table_status[3]; // 螺距补偿表建立状态

// 误差补偿库中的补偿参数结构体实例
extern CompParams par_comp_lib[3];


// One-time init: call pitch_comp_init() for each axis, initial param sync
void comp_data_init(void);

// Per-cycle: copy pitch_comp fields from par_acmp[] to par_comp_lib[]
void sync_comp_params(void);

// Build pitch compensation tables from shared memory data for all axes
// Must be called after comp_data_sync_params()
void establish_pitch_comp_table(void);

#ifdef __cplusplus
}
#endif

#endif