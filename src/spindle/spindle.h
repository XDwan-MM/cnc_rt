#ifndef SPINDLE_H
#define SPINDLE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "math.h"
#include <stdint.h>
#include "hal_driver.h"
#include "global.h"

int32_t Spindle_Cw(void);
int32_t Spindle_Ccw(void);

void Spindle_Stop(void);
void emcmcodedeal();
void emcSpindleControl();

extern double emot_speed;
extern double speed;
extern int sp_sign;
extern bool sp_stop_sem;

#ifdef __cplusplus
}
#endif


#endif // SPINDLE_H
