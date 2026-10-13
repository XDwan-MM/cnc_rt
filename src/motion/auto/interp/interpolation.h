#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

POS auto_cal_interp_pos(POS_CMD_DATA* pos_cmd_data, double accum_distance, double target_distance);

#ifdef __cplusplus
}
#endif

#endif    // INTERPOLATION_H
