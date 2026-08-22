#ifndef COMP_DATA_H
#define COMP_DATA_H

#include "backlash_comp.h"
#include "datatype.h"
#include "pitch_comp.h"
#include "pitch_comp_seqlist.h"
#include "straightness_comp.h"
#include "straightness_comp_seqlist.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 误差补偿库中的补偿参数结构体声明
extern CompParams par_comp_lib[3];

// 补偿算法状态机声明
extern PitchCompState pitch_comp_state[3];                // XYZ轴 螺距补偿状态机
extern BacklashCompState backlash_comp_state[3];          // XYZ轴 反向间隙补偿状态机
extern StraightnessCompState straightness1_comp_state[3]; // XYZ轴 第一组直线度误差补偿状态机
extern StraightnessCompState straightness2_comp_state[3]; // XYZ轴 第二组直线度误差补偿状态机

// 各个补偿表声明
extern PitchCompSeqList_t pitch_comp_table[3];                // XYZ轴 螺距补偿表
extern StraightnessCompSeqList_t straightness1_comp_table[3]; // XYZ轴 第一组直线度误差补偿表
extern StraightnessCompSeqList_t straightness2_comp_table[3]; // XYZ轴 第二组直线度误差补偿表

// 各个补偿表创建结果返回声明
extern OP_RES pitch_comp_table_status[3];         // 螺距补偿表建立结果返回
extern OP_RES straightness1_comp_table_status[3]; // 第一组直线度补偿表建立结果返回
extern OP_RES straightness2_comp_table_status[3]; // 第二组直线度补偿表建立结果返回

// 外部接口声明
extern void comp_data_init(void);

extern int comp_data_load_saved_compensation(void);

extern int comp_data_save_current_compensation(void);

extern void sync_comp_params(void);

extern void establish_pitch_comp_table(void);

extern void establish_straightness_comp_table(void);

extern void comp_handle_event(SYS_EVENT event);

#ifdef __cplusplus
}
#endif

#endif /* COMP_DATA_H */
