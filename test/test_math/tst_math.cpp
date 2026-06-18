#include <gtest/gtest.h>

#include "cnc_math.h"
#include "datatype.h"

/********************************************************************
 * @file        tst_math.cpp
 * @brief       src/utils/math/cnc_math 单元测试
 *              覆盖比较、阶乘/二项式、min/max、矩阵运算、方程求解等纯函数
********************************************************************/

class TestMath : public ::testing::Test {
};

// ---------------- 浮点比较 ----------------

TEST_F(TestMath, IsNan)
{
    EXPECT_TRUE(IsNaN(NAN));
    EXPECT_TRUE(!IsNaN(0.0));
    EXPECT_TRUE(!IsNaN(1.0e30));
}

TEST_F(TestMath, IsEqual)
{
    EXPECT_TRUE(IsEqualTo(1.0, 1.0));
    EXPECT_TRUE(IsEqualTo(1.0, 1.0 + 1e-9));   // 误差范围内视为相等
    EXPECT_TRUE(!IsEqualTo(1.0, 1.1));
    EXPECT_TRUE(!IsEqualTo(NAN, NAN));         // NaN 永不相等
}

TEST_F(TestMath, Compare)
{
    EXPECT_TRUE(IsGreaterThan(2.0, 1.0));
    EXPECT_TRUE(!IsGreaterThan(1.0, 1.0));
    EXPECT_TRUE(IsLessThan(1.0, 2.0));
    EXPECT_TRUE(!IsLessThan(2.0, 1.0));
    EXPECT_TRUE(IsGreaterThanOrEqual(2.0, 1.0));
    EXPECT_TRUE(IsGreaterThanOrEqual(1.0, 1.0));
    EXPECT_TRUE(IsLessThanOrEqual(1.0, 2.0));
    EXPECT_TRUE(IsLessThanOrEqual(1.0, 1.0));
    // NaN 参与比较一律为 false
    EXPECT_TRUE(!IsGreaterThan(NAN, 1.0));
    EXPECT_TRUE(!IsLessThan(NAN, 1.0));
}

// ---------------- 整数/浮点运算 ----------------

TEST_F(TestMath, Factorial)
{
    EXPECT_EQ(Factorial(0), 1);
    EXPECT_EQ(Factorial(1), 1);
    EXPECT_EQ(Factorial(5), 120);
    EXPECT_EQ(Factorial(7), 5040);
}

TEST_F(TestMath, Binomial)
{
    // C(5,2) = 10, C(6,0) = 1, C(4,4) = 1
    EXPECT_EQ(Binomial(5, 2), 10.0);
    EXPECT_EQ(Binomial(6, 0), 1.0);
    EXPECT_EQ(Binomial(4, 4), 1.0);
}

TEST_F(TestMath, IntMinMax)
{
    EXPECT_EQ(int_min(3, 5), 3);
    EXPECT_EQ(int_min(-2, 4), -2);
    EXPECT_EQ(int_min(7, 7), 7);
    EXPECT_EQ(int_max(3, 5), 5);
    EXPECT_EQ(int_max(-2, 4), 4);
    EXPECT_EQ(int_max(7, 7), 7);
}

TEST_F(TestMath, DoubleMin)
{
    EXPECT_EQ(double_min(1.5, 2.5), 1.5);
    EXPECT_EQ(double_min(-1.0, -2.0), -2.0);
}

TEST_F(TestMath, MinDoubleArray)
{
    double arr[] = {3.2, 1.1, 5.6, -0.4, 2.0};
    EXPECT_EQ(min_double_array(arr, 5), -0.4);
    double single[] = {9.9};
    EXPECT_EQ(min_double_array(single, 1), 9.9);
}

// ---------------- 矩阵 ----------------

TEST_F(TestMath, MatrixTranspose)
{
    // 2x3 矩阵转置为 3x2
    double m[2][3] = {{1, 2, 3}, {4, 5, 6}};
    double mt[3][2] = {{0}};
    Matrix_Transpose(&m[0][0], 2, 3, &mt[0][0]);
    EXPECT_EQ(mt[0][0], 1.0); EXPECT_EQ(mt[0][1], 4.0);
    EXPECT_EQ(mt[1][0], 2.0); EXPECT_EQ(mt[1][1], 5.0);
    EXPECT_EQ(mt[2][0], 3.0); EXPECT_EQ(mt[2][1], 6.0);
}

TEST_F(TestMath, MatrixMultiply)
{
    // [1 2; 3 4] * [5 6; 7 8] = [19 22; 43 50]
    double a[2][2] = {{1, 2}, {3, 4}};
    double b[2][2] = {{5, 6}, {7, 8}};
    double out[2][2] = {{0}};
    GNC_RES res = Matirx_Multiply(&a[0][0], 2, 2, &b[0][0], 2, 2, &out[0][0]);
    EXPECT_EQ(res, GNC_CAL_SUCCESS);
    EXPECT_EQ(out[0][0], 19.0); EXPECT_EQ(out[0][1], 22.0);
    EXPECT_EQ(out[1][0], 43.0); EXPECT_EQ(out[1][1], 50.0);
}

TEST_F(TestMath, MatrixMultiplyDimMismatch)
{
    // 左矩阵列数 != 右矩阵行数 -> 计算失败
    double a[2][3] = {{1, 2, 3}, {4, 5, 6}};
    double b[2][2] = {{1, 0}, {0, 1}};
    double out[2][2] = {{0}};
    GNC_RES res = Matirx_Multiply(&a[0][0], 2, 3, &b[0][0], 2, 2, &out[0][0]);
    EXPECT_EQ(res, GNC_CAL_ERROR);
}

TEST_F(TestMath, MatrixInverse)
{
    // [4 7; 2 6] 的逆为 [0.6 -0.7; -0.2 0.4]
    double x[2][2] = {{4, 7}, {2, 6}};
    GNC_RES res = Matrix_Inv(&x[0][0], 2);
    EXPECT_EQ(res, GNC_CAL_SUCCESS);
    EXPECT_TRUE(IsEqualTo(x[0][0], 0.6));
    EXPECT_TRUE(IsEqualTo(x[0][1], -0.7));
    EXPECT_TRUE(IsEqualTo(x[1][0], -0.2));
    EXPECT_TRUE(IsEqualTo(x[1][1], 0.4));
}

// ---------------- 方程求解 ----------------

// 对返回的根集合做无序断言：每个期望根都能在结果中找到
static bool roots_contain(const double *roots, int n, double expect)
{
    for (int i = 0; i < n; i++) {
        if (IsEqualTo(roots[i], expect)) return true;
    }
    return false;
}

TEST_F(TestMath, QuadraticTwoRoots)
{
    // x^2 - 3x + 2 = 0 => x = 1, 2
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, -3.0, 2.0, x);
    EXPECT_EQ(n, 2);
    EXPECT_TRUE(roots_contain(x, n, 1.0));
    EXPECT_TRUE(roots_contain(x, n, 2.0));
}

TEST_F(TestMath, QuadraticLinear)
{
    // a=0 退化为一次方程 -2x + 4 = 0 => x = 2
    double x[2] = {0};
    int n = solve_quadratic_equation(0.0, -2.0, 4.0, x);
    EXPECT_EQ(n, 1);
    EXPECT_TRUE(IsEqualTo(x[0], 2.0));
}

TEST_F(TestMath, QuadraticDoubleRoot)
{
    // x^2 - 2x + 1 = (x-1)^2 = 0 => 重根 x = 1（返回 2，x[0]、x[1] 均为 1）
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, -2.0, 1.0, x);
    EXPECT_EQ(n, 2);
    EXPECT_TRUE(IsEqualTo(x[0], 1.0));
    EXPECT_TRUE(IsEqualTo(x[1], 1.0));
}

TEST_F(TestMath, QuadraticNoRoot)
{
    // x^2 + 1 = 0 无实根
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, 0.0, 1.0, x);
    EXPECT_EQ(n, 0);
    // a=0 且 b=0 时无解
    n = solve_quadratic_equation(0.0, 0.0, 5.0, x);
    EXPECT_EQ(n, 0);
}

TEST_F(TestMath, CubicTripleRoot)
{
    // (x-1)^3 = x^3 - 3x^2 + 3x - 1 = 0 => 三重根 x = 1
    double x[3] = {0};
    int n = solve_cubic_equation(1.0, -3.0, 3.0, -1.0, x);
    EXPECT_EQ(n, 3);
    EXPECT_TRUE(IsEqualTo(x[0], 1.0));
    EXPECT_TRUE(IsEqualTo(x[1], 1.0));
    EXPECT_TRUE(IsEqualTo(x[2], 1.0));
}

TEST_F(TestMath, QuarticFourRoots)
{
    // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24 = 0
    double x[4] = {0};
    int n = solve_quartic_equation(1.0, -10.0, 35.0, -50.0, 24.0, x);
    EXPECT_EQ(n, 4);
    EXPECT_TRUE(roots_contain(x, n, 1.0));
    EXPECT_TRUE(roots_contain(x, n, 2.0));
    EXPECT_TRUE(roots_contain(x, n, 3.0));
    EXPECT_TRUE(roots_contain(x, n, 4.0));
}

TEST_F(TestMath, MinNonnegativeRoot)
{
    double roots1[] = {-1.0, 3.0, 0.5, 2.0};
    EXPECT_EQ(get_min_nonnegative_real_root(4, roots1), 0.5);
    // 全为负数时返回 INF
    double roots2[] = {-1.0, -3.0};
    EXPECT_EQ(get_min_nonnegative_real_root(2, roots2), INF);
}
