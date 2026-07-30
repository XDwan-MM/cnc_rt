#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

POS auto_cal_interp_pos(POS current_pos, double accum_distance);

#ifdef __cplusplus
}
#endif

#endif    // INTERPOLATION_H
