#ifndef NURBSFIT_H
#define NURBSFIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>   13 | #include "trajectory.h"
#include <string.h>
#include "global.h"
#include "cnc_math.h"
#include <hmisysapi.h>
#include <alarmapi.h>

//xxt 逼近
#define ELEMENT(M,col,i,j)	*((M)+(col)*(i)+(j))

#define SQUARE(a) (a)*(a)


int m_mul(double*left, int row_left, int col_left, double*right, int row_right, int col_right, double*out);

int m_trans(double*M, int row, int col, double *M_tran);

int inv(double *X, int n);

typedef double* basefunction;

//%  通过积累弦长法算出节点矢量(三次样条)
int getUk(double *Uk, POS *data_point, int row_length);

void getKnot(double* nurbs_U, double* Uk, int interpnum, int data_length, int *U_global_length, int nurbs_p);

void findSpan(int* span, int length, int nurbs_p, double uki, double nurbs_U[]);

int curvePoint(POS *point_C, int length, int nurbs_p, double* nurbs_U, POS* nurbs_P, double uki);

int getFittingError(double* fittingError, POS *data_point, POS *controlpoint ,int k, double *U, double* uk, int row_length, int P_length);

int onebasisFun(double* onebasisfuntemp, int nurbs_p, double* nurbs_U, int span_i, double uki, int nurbs_U_length);

int getControlPoint(POS* point, POS* data_point, double* uk, int data_row, double* nurbs_U, int interpnum, int nurbs_U_length, int nurbs_p);

int approximate(double fittingErrorThr, double sagittaErrorThr, POS* data, int m, POS T0, POS T1,POS* out_P, double** out_knots, int* out_ctrl_cnt,int k);

// cy 插补
int findspan(double *U, int n, double u);

int basisfunc(int span, double u, int k, double *U, double *N);

int evalNURBS(POS *CtrlPts, int k, double *U, double u, int nCtrl, POS *P);

double getlength(double *u_vals, int num_samples, POS *CtrlPts, int k, double *U, int nCtrl,double *w);

double basis(int i, int p, double u, const double *U, int nCtrl);

double basis_derivative(int i, int p, double u, const double *U, int nCtrl);

double nurbs_curve_curvature(double u, const POS *P, const double *w, const double *U, int nCtrl, int p);

double get_max_curvature(const POS *controlPoint, const double *w, const double *U, int nCtrl, int p, int precise);

int cal_nurbs_controlpoint(PLAN_QUEUE*nurbs_pretreat_queue, PLAN_BLOCK* pre_nurbs_block, double fit_error,double sagittaErrorThr);

POS interpolate_nurbs(double deltaS, double *u,double *u_next);





// double nurbs_speed(double u, const POS *P, const double *w, const double *U,int nCtrl, int p);

// double simpson(double a, double b, double fa, double fm, double fb);

// double adaptive_simpson(
//     const POS *P, const double *w, const double *U,
//     int nCtrl, int p,
//     double a, double b, double fa, double fm, double fb,
//     double whole, double eps, int depth);

// double nurbs_length(
//     const POS *P, const double *w, const double *U,
//     int nCtrl, int p);

// void getArcLength(double* towpoint_arclength,POS *point,int num_ctrl, double *U, int u_length,double uStart,double uEnd,double error);


#ifdef __cplusplus
}
#endif

#endif // NURBSFIT_H