#include <stdio.h>
#include <math.h>
#include "cnc_math.h"

bool IsNaN(double value)
{
    return value != value;
}

bool IsEqualTo(double value1, double value2){
    if (IsNaN(value1) || IsNaN(value2))
        return false;
    double eps = ( fabs(value1) + fabs(value2) + 10 )* DOUBLE_EPSILON;
    double delta = value1 - value2;
    return (-eps < delta) && (eps > delta);
}

bool IsGreaterThan(double value1, double value2){
    if (IsNaN(value1) || IsNaN(value2))
        return false;
    return value1 > value2 && !IsEqualTo(value1, value2);
}

bool IsLessThan(double value1, double value2){
    if (IsNaN(value1) || IsNaN(value2))
        return false;
    return value1 < value2 && !IsEqualTo(value1, value2);
}

bool IsGreaterThanOrEqual(double value1, double value2){
    if (IsNaN(value1) || IsNaN(value2))
        return false;
    return (value1 - value2 > DOUBLE_EPSILON) || IsEqualTo(value1, value2);
}
bool IsLessThanOrEqual(double value1, double value2){
    if (IsNaN(value1) || IsNaN(value2))
        return false;
    return value1 < value2 || IsEqualTo(value1, value2);
}

int Factorial(int number){
    if (number == 0)
        return 1;
    else
        return number * Factorial(number - 1);
}

double Binomial(int number, int i){
    return Factorial(number) / (Factorial(i) * Factorial(number - i));
}

int int_min(int a, int b){
    return a < b? a : b;
}

int int_max(int a, int b){
    return a > b? a : b;
}

double double_min(double a, double b){
    return a < b? a : b;
}

double min_double_array(double array[], int n){
    double min = array[0];
    for (int i = 1; i < n; i++){
        min = fmin(min, array[i]);
    }
    return min;
}

/// @brief 矩阵相乘
/// @param left 左矩阵
/// @param row_left 矩阵行数
/// @param col_left 矩阵列数
/// @param right 右矩阵
/// @param row_right 矩阵行数
/// @param col_right 矩阵列数
/// @param out 矩阵相乘结果
/// @return 
/// @author 许晓婷
/// @version 1.0
/// @date 2025/10/15
GNC_RES Matirx_Multiply(const double *left, int row_left, int col_left, const double *right, int row_right, int col_right, double *out)
{
    GNC_RES GNC_RES = GNC_CAL_SUCCESS;
    int i = 0;
    int j = 0;
    int k = 0;
    memset(out, 0, row_left * col_right *sizeof(double));
    if(col_left != row_right)
    {
        GNC_RES = GNC_CAL_ERROR;
        return GNC_RES;
    }

    for(i = 0; i < row_left; i++)
    {
        for(j = 0; j < col_left; j++)
        {
            for(k = 0; k < col_right; k++)
            {
                ELEMENT(out, col_right, i, k) += ELEMENT(left, col_left, i, j) * ELEMENT(right, col_right, j, k);
            }
        }
    }
    return GNC_RES;
}


/// @brief 矩阵转置
/// @param *M   - 待转置矩阵
/// @param row    - 矩阵的行数 
/// @param col    - 矩阵的列数
/// @return *M_tran -  转置结果
/// @author 许晓婷
/// @version 1.0
/// @date 2025/10/15
void Matrix_Transpose(const double *M, int row, int col, double *M_tran)
{
    int i, j;
    for(i = 0; i < row; i++)
    {
        for(j = 0; j < col; j++)
        {
            ELEMENT(M_tran, row, j, i) = ELEMENT(M, col, i, j);
        }
    }
}


/// @brief 矩阵求逆
/// @param X == two dimensional array matrix
/// @param n 矩阵维度
/// @return 
/// @author 许晓婷
/// @version 1.0
/// @date 2025/10/15
GNC_RES Matrix_Inv(double *X, int n)
{
    GNC_RES GNC_RES = GNC_CAL_SUCCESS;
    unsigned int  i, j, k;
    static double a[MAXMATRICSCALE][MAXMATRICSCALE * 2], t;

    if (n > MAXMATRICSCALE){
        GNC_RES = GNC_PARAM_UNVALID;
        return GNC_RES;
    }

    for(i = 0; i < n; i++)
    {
        memcpy(a[i], &X[i * n], sizeof(double) * n);
    }

    for(i = 0; i < n; i++)
    {
        for(j = n; j < 2 * n; j++)
        {
            if(i == j - n)
            {
                a[i][j] = 1;
            }
            else
            {
                a[i][j] = 0;
            }
        }
    }

    for(i = 0; i < n; i++)
    {
        t = a[i][i];       
        if(IsNaN(t)){
            //printf("error: 求解矩阵奇异 %f.\n", t);
            GNC_RES = GNC_CAL_ERROR;
            return GNC_RES;
        }
        if(fabs(t) < INV_EPSILON)
        {
            //printf("error: Are divided by a very small number %f.\n", t);
            t = INV_EPSILON;
            GNC_RES = GNC_CAL_ERROR;
            return GNC_RES;
        }
        for(j = i; j < 2 * n; j++)
        {
            a[i][j] = a[i][j] / t;
        }
        for(j = 0; j < n; j++)
        {
            if(i != j)
            {
                t = a[j][i];
                for(k = 0; k < 2 * n; k++)
                {
                    a[j][k] = a[j][k] - t * a[i][k];
                }
            }
        }
    }

    for(i = 0; i < n; i++)
    {
        memcpy(&X[i * n], &a[i][n], sizeof(double) * n);
    }
    
    return GNC_RES;
}

/// @brief 算方程的最小正数根
/// @param rootCount 个数 
/// @param x 实数根
/// @param min_real_roots 最小正数根
/// @return min_real_roots 最小正数根
/// @author 许晓婷
/// @version 1.0
/// @date 2026/06/28
double get_min_positive_real_root(int rootCount, double roots[]){
    // VALIDATE_ARGUMENT(rootCount != 0, "rootCount", "rootCount must be greater than 0");
    double min_root = INF;
    for(int i = 0; i < rootCount; i++){
        if(IsGreaterThan(roots[i], 0.0) && IsLessThan(roots[i], min_root)){
            min_root = roots[i];
        }
    }
    return min_root;
}

/// @brief 计算方程的最小非负数根
/// @param rootCount 个数 
/// @param x 实数根
/// @param min_real_roots 最小非负数根
/// @return min_real_roots 最小非负数根
/// @author 许晓婷
/// @version 1.0
/// @date 2025/10/14
double get_min_nonnegative_real_root(int rootCount, double roots[]){
    // VALIDATE_ARGUMENT(rootCount != 0, "rootCount", "rootCount must be greater than 0");
    double min_root = INF;
    for(int i = 0; i < rootCount; i++){
        if(IsGreaterThanOrEqual(roots[i], 0.0) && IsLessThan(roots[i], min_root)){
            min_root = roots[i];
        }
    }
    return min_root;
}


/*************************************************
Description: 求一元二次方程(a*x^2 + b*x + c = 0)的所有实数根
Input: 方程的系数 a,b,c
Output: 方程的所有实数根x
Return: 实数根的个数
Author: 许晓婷
Version: 1.0
Date: 2025/10/14
*************************************************/
int solve_quadratic_equation(double a, double b, double c, double x[])
{
    double delta, sqrtDelta;

    if (fabs(a - 0.0) < EPS)
    {
        if (fabs(b - 0.0) < EPS)
        {
            return 0;
        }
        else
        {
            x[0] = -c / b;
            return 1;
        }
    }
    else
    {
        delta = b * b - 4.0 * a * c;
        if (delta > ZERO)
        {
            if (fabs(c - 0.0) < EPS)    //若c = 0,由于计算误差,sqrt(b*b - 4*a*c）不等于b
            {
                x[0] = 0.0;
                x[1] = -b / a;
            }
            else
            {
                sqrtDelta = sqrt(delta);
                if (b > 0.0)
                {
                    x[0] = (-2.0 * c) / (b + sqrtDelta);    //避免两个很接近的数相减,导致精度丢失
                    x[1] = (-b - sqrtDelta) / (2.0 * a);
                }
                else
                {
                    x[0] = (-b + sqrtDelta) / (2.0 * a);
                    x[1] = (-2.0 * c) / (b - sqrtDelta);    //避免两个很接近的数相减,导致精度丢失
                }
            }
            return 2;
        }
        else if (fabs(delta - 0.0) < EPS)
        {
            x[0] = x[1] = -b / (2.0 * a);
            return 2;   // 重根，x[0]、x[1] 为两个相等实根
        }
        else
        {
            return 0;
        }
    }
}


/*************************************************
Description: 盛金公式求一元三次方程(a*x^3 + b*x^2 + c*x + d = 0)的所有实数根
             A = b * b - 3.0 * a * c;
             B = b * c - 9.0 * a * d;
             C = c * c - 3.0 * b * d;
             (1)当A = B = 0时，方程有一个三重实根
             (2)当Δ = B^2－4AC>0时，方程有一个实根和一对共轭虚根
             (3)当Δ = B^2－4AC = 0时，方程有三个实根，其中有一个两重根
             (4)当Δ = B^2－4AC<0时，方程有三个不相等的实根
Input: 方程的系数 a,b,c,d
Output: 方程的所有实数根x
Return: 实数根的个数
Author: 许晓婷
Version: 1.0
Date: 2025/10/14
*************************************************/
int solve_cubic_equation(double a, double b, double c, double d, double x[])
{
    double A, B, C, delta;
    double Y1, Y2, Z1, Z2, K, parm[3] = {0}, roots[2] = {0}, theta, T;

    if (fabs(a - 0.0) < EPS)
    {
        return solve_quadratic_equation(b, c, d, x);
    }
    else
    {
        A = b * b - 3.0 * a * c;
        B = b * c - 9.0 * a * d;
        C = c * c - 3.0 * b * d;

        delta = B * B - 4.0 * A * C;

        if (fabs(A - 0.0) < EPS && fabs(B - 0.0) < EPS)
        {
            x[0] = x[1] = x[2] = -b / (3.0 * a);
            return 3;
        }

        if (delta > ZERO)
        {
            parm[2] = 1.0;
            parm[1] = B;
            parm[0] = A * C;

            solve_quadratic_equation(parm[2], parm[1], parm[0], roots);
            Z1 = roots[0];
            Z2 = roots[1];

            Y1 = A * b + 3.0 * a * Z1;
            Y2 = A * b + 3.0 * a * Z2;

            if (Y1 < 0.0 && Y2 < 0.0)    //pow函数的底数必须为非负数,必须分类讨论
            {
                x[0] = (-b + pow(-Y1, 1.0 / 3.0) + pow(-Y2, 1.0 / 3.0)) / (3.0*a);
            }
            else if (Y1 < 0.0 && Y2 > 0.0)
            {
                x[0] = (-b + pow(-Y1, 1.0 / 3.0) - pow(Y2, 1.0 / 3.0)) / (3.0*a);
            }
            else if (Y1 > 0.0 && Y2 < 0.0)
            {
                x[0] = (-b - pow(Y1, 1.0 / 3.0) + pow(-Y2, 1.0 / 3.0)) / (3.0*a);
            }
            else
            {
                x[0] = (-b - pow(Y1, 1.0 / 3.0) - pow(Y2, 1.0 / 3.0)) / (3.0*a);
            }
            return 1;
        }
        else if (fabs(delta - 0.0) < EPS)
        {
            if (fabs(A - 0.0) > EPS)
            {
                K = B / A;
                x[0] = -b / a + K;
                x[1] = x[2] = -0.5 * K;
                return 3;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            if (A > 0.0)
            {
                T = (2.0 * A * b - 3.0 * a * B) / (2.0 * pow(A, 3.0 / 2.0));
                if (T > 1.0)    //由于计算误差,T的值可能略大于1(如1.0000001)
                {
                    T = 1.0;
                }
                if (T < -1.0)
                {
                    T = -1.0;
                }
                theta = acos(T);
                x[0] = (-b - 2.0 * sqrt(A) * cos(theta / 3.0)) / (3.0 * a);
                x[1] = (-b + sqrt(A) * (cos(theta / 3.0) + sqrt(3.0) * sin(theta / 3.0))) / (3.0 * a);
                x[2] = (-b + sqrt(A) * (cos(theta / 3.0) - sqrt(3.0) * sin(theta / 3.0))) / (3.0 * a);
                return 3;
            }
            else
            {
                return 0;
            }
        }
    }
}


/*************************************************
Description: 费拉里法求一元四次方程(a*x^4 + b*x^3 + c*x^2 + d*x + e = 0)的所有实数根
Input: 方程的系数 a,b,c,d,e
Output: 方程的所有实数根x
Return: 实数根的个数
Author: 许晓婷
Version: 1.0
Date: 2025/10/14
*************************************************/
int solve_quartic_equation(double a, double b, double c, double d, double e, double x[])
{
    double parm[4] = {0}, roots[3] = {0};
    double y, M, N;
    double x1[2] = {0}, x2[2] = {0};
    int rootCount1, rootCount2, rootCount, i;
    double MSquareTemp, MSquare, yTemp;

    if (fabs(a - 0.0) < EPS)
    {
        if (fabs(b - 0.0) < EPS)
        {
            return solve_quadratic_equation(c, d, e, x);
        }
        else
        {
            return solve_cubic_equation(b, c, d, e, x);
        }
    }
    else
    {
        b = b / a;
        c = c / a;
        d = d / a;
        e = e / a;

        parm[3] = 8.0;
        parm[2] = -4.0 * c;
        parm[1] = 2.0 * (b * d - 4.0 * e);
        parm[0] = -e * (b * b - 4.0 * c) - d * d;

        if (rootCount = solve_cubic_equation(parm[3], parm[2], parm[1], parm[0], roots))
        {
            y = roots[0];
            MSquare = 8.0 * y + b * b - 4.0 * c;
            for (i = 1; i < rootCount; i++)
            {
                yTemp = roots[i];
                MSquareTemp = 8.0 * yTemp + b * b - 4.0 * c;
                if (MSquareTemp > MSquare)
                {
                    MSquare = MSquareTemp;
                    y = yTemp;
                }
            }

            if (MSquare > 0.0)
            {
                M = sqrt(MSquare);
                N = b * y - d;
                parm[2] = 2.0;
                parm[1] = b + M;
                parm[0] = 2.0 * (y + N / M);
                rootCount1 = solve_quadratic_equation(parm[2], parm[1], parm[0], x1);

                parm[2] = 2.0;
                parm[1] = b - M;
                parm[0] = 2.0 * (y - N / M);
                rootCount2 = solve_quadratic_equation(parm[2], parm[1], parm[0], x2);

                if (rootCount1 == 2)
                {
                    x[0] = x1[0];
                    x[1] = x1[1];
                    x[2] = x2[0];
                    x[3] = x2[1];
                }
                else
                {
                    x[0] = x2[0];
                    x[1] = x2[1];
                    x[2] = x1[0];
                    x[3] = x1[1];
                }
                return rootCount1 + rootCount2;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
}


// void main()
// {
    // double x[2], xx[3], xxx[4], p[5];
    // int rootCount;
    // double breakPointHere, x1, x2, a, b, c, d, e;

    //(1)一元二次方程测试
    //x^2 - 1000000.000001*x + 1 = 0
    //0*x^2 - 10*x + 1 = 0
    //0 * x ^ 2 - 10 * x + 1 = 0
    //x^2 - 10000000*x + 0.01 = 0
    //1.0e-20*x^2 - 2.0e-20*x + 1.0e-20 = 0

    // a = 1;
    // b = -1000000.000001;
    // c = 1;
    // p[0] = c;
    // p[1] = b;
    // p[2] = a;
    // rootCount = solve_quadratic_equation(p, x);
    // x1 = (-b + sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    // x2 = (-b - sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    // breakPointHere = 1.0;

    //(2)一元三次方程测试
    //(x-1)*(x^2+1)=0 (x^3 - x^2 + x - 1 = 0)
    //(x-1)^3 = 0 (x^3 - 3*x^2 + 3*x - 1 = 0)
    //(x-1)^2*(x-2)=0 (x^3 - 4*x^2 + 5*x - 2 = 0)
    //(x-1)*(x-2)*(x-3) = 0 (x^3 - 6*x^2 + 11*x - 6 = 0)
    //0*x^3 + x^2 - 2*x + 1 = 0
    //0*x^3 + 0*x^2 - 2*x + 1 = 0
    //0*x^3 + 0*x^2 + 0*x + 1 = 0

    // a = 1;
    // b = -6;
    // c = 11;
    // d = -6;
    // p[0] = d;
    // p[1] = c;
    // p[2] = b;
    // p[3] = a;
    // rootCount = solve_cubic_equation(p, xx);
    // breakPointHere = 1.0;

    //(3)一元四次方程测试
    //(x-1)*(x-2)*(x^2 + 1)=0 (x^4 - 3*x^3 + 3*x^2 - 3*x + 2 = 0)
    //(x-1)^2*(x^2 + 1)=0 (x^4 - 2*x^3 + 2*x^2 - 2*x + 1 = 0)
    //(x-1)*(x-2)*(x-3)*(x-4)=0 (x^4 - 10*x^3 + 35*x^2 - 50*x + 24 = 0)
    //(x-1)^2*(x-2)^2 = 0 (x^4 - 6*x^3 + 13*x^2 - 12*x + 4 = 0)
    //0*x^4 + x^3 - 3*x^2 + 3*x - 1 = 0
    //0*x^4 + 0*x^3 + x^2 - 2*x + 1 = 0
    //0*x^4 + 0*x^3 + 0*x^2 - 2*x + 1 = 0
    // a = 1;
    // b = -10;
    // c = 35;
    // d = -50;
    // e = 24;
    // p[0] = e;
    // p[1] = d;
    // p[2] = c;
    // p[3] = b;
    // p[4] = a;
    // rootCount = solve_quartic_equation(p, xxx);
    // breakPointHere = 1.0;

    // a = 1.167644e-08;
    // b = -0.14794;
    // c = -0.312678;
    // d = 0.704048;
    // e = 136790.688;
    // p[0] = e;
    // p[1] = d;
    // p[2] = c;
    // p[3] = b;
    // p[4] = a;
    // rootCount = solve_quartic_equation(p, xxx);
    // breakPointHere = 1.0; 
// }
