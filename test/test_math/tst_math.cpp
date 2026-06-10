#include <QtTest>
#include "test_registry.h"

extern "C" {
#include "cnc_math.h"
#include "datatype.h"
}

/********************************************************************
 * @file        tst_math.cpp
 * @brief       src/utils/math/cnc_math 单元测试
 *              覆盖比较、阶乘/二项式、min/max、矩阵运算、方程求解等纯函数
********************************************************************/

class TestMath : public QObject
{
    Q_OBJECT

private slots:
    // 浮点比较
    void test_is_nan();
    void test_is_equal();
    void test_compare();
    // 整数/浮点运算
    void test_factorial();
    void test_binomial();
    void test_int_min_max();
    void test_double_min();
    void test_min_double_array();
    // 矩阵
    void test_matrix_transpose();
    void test_matrix_multiply();
    void test_matrix_multiply_dim_mismatch();
    void test_matrix_inverse();
    // 方程求解
    void test_quadratic_two_roots();
    void test_quadratic_linear();
    void test_quadratic_double_root();
    void test_quadratic_no_root();
    void test_cubic_triple_root();
    void test_quartic_four_roots();
    void test_min_nonnegative_root();
};

// ---------------- 浮点比较 ----------------

void TestMath::test_is_nan()
{
    QVERIFY(IsNaN(NAN));
    QVERIFY(!IsNaN(0.0));
    QVERIFY(!IsNaN(1.0e30));
}

void TestMath::test_is_equal()
{
    QVERIFY(IsEqualTo(1.0, 1.0));
    QVERIFY(IsEqualTo(1.0, 1.0 + 1e-9));   // 误差范围内视为相等
    QVERIFY(!IsEqualTo(1.0, 1.1));
    QVERIFY(!IsEqualTo(NAN, NAN));         // NaN 永不相等
}

void TestMath::test_compare()
{
    QVERIFY(IsGreaterThan(2.0, 1.0));
    QVERIFY(!IsGreaterThan(1.0, 1.0));
    QVERIFY(IsLessThan(1.0, 2.0));
    QVERIFY(!IsLessThan(2.0, 1.0));
    QVERIFY(IsGreaterThanOrEqual(2.0, 1.0));
    QVERIFY(IsGreaterThanOrEqual(1.0, 1.0));
    QVERIFY(IsLessThanOrEqual(1.0, 2.0));
    QVERIFY(IsLessThanOrEqual(1.0, 1.0));
    // NaN 参与比较一律为 false
    QVERIFY(!IsGreaterThan(NAN, 1.0));
    QVERIFY(!IsLessThan(NAN, 1.0));
}

// ---------------- 整数/浮点运算 ----------------

void TestMath::test_factorial()
{
    QCOMPARE(Factorial(0), 1);
    QCOMPARE(Factorial(1), 1);
    QCOMPARE(Factorial(5), 120);
    QCOMPARE(Factorial(7), 5040);
}

void TestMath::test_binomial()
{
    // C(5,2) = 10, C(6,0) = 1, C(4,4) = 1
    QCOMPARE(Binomial(5, 2), 10.0);
    QCOMPARE(Binomial(6, 0), 1.0);
    QCOMPARE(Binomial(4, 4), 1.0);
}

void TestMath::test_int_min_max()
{
    QCOMPARE(int_min(3, 5), 3);
    QCOMPARE(int_min(-2, 4), -2);
    QCOMPARE(int_min(7, 7), 7);
    QCOMPARE(int_max(3, 5), 5);
    QCOMPARE(int_max(-2, 4), 4);
    QCOMPARE(int_max(7, 7), 7);
}

void TestMath::test_double_min()
{
    QCOMPARE(double_min(1.5, 2.5), 1.5);
    QCOMPARE(double_min(-1.0, -2.0), -2.0);
}

void TestMath::test_min_double_array()
{
    double arr[] = {3.2, 1.1, 5.6, -0.4, 2.0};
    QCOMPARE(min_double_array(arr, 5), -0.4);
    double single[] = {9.9};
    QCOMPARE(min_double_array(single, 1), 9.9);
}

// ---------------- 矩阵 ----------------

void TestMath::test_matrix_transpose()
{
    // 2x3 矩阵转置为 3x2
    double m[2][3] = {{1, 2, 3}, {4, 5, 6}};
    double mt[3][2] = {{0}};
    Matrix_Transpose(&m[0][0], 2, 3, &mt[0][0]);
    QCOMPARE(mt[0][0], 1.0); QCOMPARE(mt[0][1], 4.0);
    QCOMPARE(mt[1][0], 2.0); QCOMPARE(mt[1][1], 5.0);
    QCOMPARE(mt[2][0], 3.0); QCOMPARE(mt[2][1], 6.0);
}

void TestMath::test_matrix_multiply()
{
    // [1 2; 3 4] * [5 6; 7 8] = [19 22; 43 50]
    double a[2][2] = {{1, 2}, {3, 4}};
    double b[2][2] = {{5, 6}, {7, 8}};
    double out[2][2] = {{0}};
    GNC_RES res = Matirx_Multiply(&a[0][0], 2, 2, &b[0][0], 2, 2, &out[0][0]);
    QCOMPARE(res, GNC_CAL_SUCCESS);
    QCOMPARE(out[0][0], 19.0); QCOMPARE(out[0][1], 22.0);
    QCOMPARE(out[1][0], 43.0); QCOMPARE(out[1][1], 50.0);
}

void TestMath::test_matrix_multiply_dim_mismatch()
{
    // 左矩阵列数 != 右矩阵行数 -> 计算失败
    double a[2][3] = {{1, 2, 3}, {4, 5, 6}};
    double b[2][2] = {{1, 0}, {0, 1}};
    double out[2][2] = {{0}};
    GNC_RES res = Matirx_Multiply(&a[0][0], 2, 3, &b[0][0], 2, 2, &out[0][0]);
    QCOMPARE(res, GNC_CAL_ERROR);
}

void TestMath::test_matrix_inverse()
{
    // [4 7; 2 6] 的逆为 [0.6 -0.7; -0.2 0.4]
    double x[2][2] = {{4, 7}, {2, 6}};
    GNC_RES res = Matrix_Inv(&x[0][0], 2);
    QCOMPARE(res, GNC_CAL_SUCCESS);
    QVERIFY(IsEqualTo(x[0][0], 0.6));
    QVERIFY(IsEqualTo(x[0][1], -0.7));
    QVERIFY(IsEqualTo(x[1][0], -0.2));
    QVERIFY(IsEqualTo(x[1][1], 0.4));
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

void TestMath::test_quadratic_two_roots()
{
    // x^2 - 3x + 2 = 0 => x = 1, 2
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, -3.0, 2.0, x);
    QCOMPARE(n, 2);
    QVERIFY(roots_contain(x, n, 1.0));
    QVERIFY(roots_contain(x, n, 2.0));
}

void TestMath::test_quadratic_linear()
{
    // a=0 退化为一次方程 -2x + 4 = 0 => x = 2
    double x[2] = {0};
    int n = solve_quadratic_equation(0.0, -2.0, 4.0, x);
    QCOMPARE(n, 1);
    QVERIFY(IsEqualTo(x[0], 2.0));
}

void TestMath::test_quadratic_double_root()
{
    // x^2 - 2x + 1 = (x-1)^2 = 0 => 重根 x = 1（返回 2，x[0]、x[1] 均为 1）
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, -2.0, 1.0, x);
    QCOMPARE(n, 2);
    QVERIFY(IsEqualTo(x[0], 1.0));
    QVERIFY(IsEqualTo(x[1], 1.0));
}

void TestMath::test_quadratic_no_root()
{
    // x^2 + 1 = 0 无实根
    double x[2] = {0};
    int n = solve_quadratic_equation(1.0, 0.0, 1.0, x);
    QCOMPARE(n, 0);
    // a=0 且 b=0 时无解
    n = solve_quadratic_equation(0.0, 0.0, 5.0, x);
    QCOMPARE(n, 0);
}

void TestMath::test_cubic_triple_root()
{
    // (x-1)^3 = x^3 - 3x^2 + 3x - 1 = 0 => 三重根 x = 1
    double x[3] = {0};
    int n = solve_cubic_equation(1.0, -3.0, 3.0, -1.0, x);
    QCOMPARE(n, 3);
    QVERIFY(IsEqualTo(x[0], 1.0));
    QVERIFY(IsEqualTo(x[1], 1.0));
    QVERIFY(IsEqualTo(x[2], 1.0));
}

void TestMath::test_quartic_four_roots()
{
    // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24 = 0
    double x[4] = {0};
    int n = solve_quartic_equation(1.0, -10.0, 35.0, -50.0, 24.0, x);
    QCOMPARE(n, 4);
    QVERIFY(roots_contain(x, n, 1.0));
    QVERIFY(roots_contain(x, n, 2.0));
    QVERIFY(roots_contain(x, n, 3.0));
    QVERIFY(roots_contain(x, n, 4.0));
}

void TestMath::test_min_nonnegative_root()
{
    double roots1[] = {-1.0, 3.0, 0.5, 2.0};
    QCOMPARE(get_min_nonnegative_real_root(4, roots1), 0.5);
    // 全为负数时返回 INF
    double roots2[] = {-1.0, -3.0};
    QCOMPARE(get_min_nonnegative_real_root(2, roots2), INF);
}

REGISTER_TEST(TestMath)

#include "tst_math.moc"
