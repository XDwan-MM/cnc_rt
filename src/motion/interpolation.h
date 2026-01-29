#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#ifdef __cplusplus
extern "C" {
#endif
#include "global.h"
#include "mem.h"

#define pmSq(x) ((x) * (x))
#define PM_ERR -1      /* unspecified error */
#define PM_IMPL_ERR -2 /* not implemented */
#define PM_NORM_ERR -3 /* arg should have been norm */
#define PM_DIV_ERR -4  /* divide by zero error */
#define PM_PI 3.141592653589793 /* pi */

int tcLine(const COORD_TP *const tp, double length , POS *const point);
int pmCartScalMult(const POS *const v1, double d, POS *const vout);
int pmCartScalMultEq(POS *const v, double d);
int pmCartCartAdd(const POS *const v1, const POS *const v2, POS *const vout);
int tcCircle(const COORD_TP *const circle, double progress, POS *pos);
double saturate(double x, double max);
double pmSqrt(double x);
extern int pmErrno;

void interpolationExec(double interp_pin);

#ifdef __cplusplus
}
#endif

#endif    // INTERPOLATION_H
