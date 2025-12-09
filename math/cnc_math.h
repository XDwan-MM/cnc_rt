#ifndef CNC_MATH_H
#define CNC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        cnc_math.h
 * @brief       数学工具
 * @version     V1.00
 * @date        2025/10/14
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "./datadef/common.h"

#define EPS 1.0e-30
#define ZERO 1.0e-30
#define INF 1.0e30
#define DOUBLE_EPSILON 1E-6 //浮点数误差
#define MAXMATRICSCALE 300   //可求逆的最大矩阵规模
#define INV_EPSILON 1E-10 //防止除0

#define ELEMENT(M, col, i, j) *((M) + (col) * (i) + (j))

bool IsNaN(double value);

bool IsEqualTo(double value1, double value2);//判断是否等于

bool IsGreaterThan(double value1, double value2);//判断是否大于

bool IsLessThan(double value1, double value2);//判断是否小于

bool IsGreaterThanOrEqual(double value1, double value2);//判断是否大于等于

bool IsLessThanOrEqual(double value1, double value2);//判断是否小于等于

int Factorial(int number);// 计算整数阶乘

double Binomial(int number, int i);// 计算二项式系数

int min(int a, int b);//计算最小值

int max(int a, int b);//计算最大值

double min_double_array(double array[], int n);

// 矩阵转置
void Matrix_Transpose(const double *M, int row, int col, double *M_tran); 

// 矩阵相乘
GNC_RES Matirx_Multiply(const double *left, int row_left, int col_left, const double *right, int row_right, int col_right, double *out);

// 矩阵求逆
GNC_RES Matrix_Inv(double *X, int n);

// 求解一元二次方程
int solve_quadratic_equation(float a, float b, float c, float x[]);

// 求解一元三次方程
int solve_cubic_equation(float a, float b, float c, float d, float x[]);

// 求解一元四次方程
int solve_quartic_equation(float a, float b, float c, float d, float e, float x[]);

// 计算最小非负数实数根
float get_min_nonnegative_real_root(int rootCount, float roots[]);

#ifdef __cplusplus
}
#endif

#endif // CNC_MATH_H