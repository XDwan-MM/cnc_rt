/******************************************************************** 
 * @file        test_cnc_math.c
 * @brief       cnc_math 模块单元测试
 * @version     V1.00
 * @date        2025/11/29
 * @description 测试数学工具函数
 ********************************************************************/

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../math/cnc_math.h"

#define EPSILON 1E-5
#define ASSERT_DOUBLE_EQ(a, b) assert(fabs((a) - (b)) < EPSILON)
#define ASSERT_FLOAT_EQ(a, b) assert(fabsf((a) - (b)) < 1E-4f)
#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_START(name) printf("[TEST] %s ... ", name)

//=============================================================================
// Test: IsNaN - NaN检测
//=============================================================================
void test_IsNaN() {
    TEST_START("IsNaN - NaN检测");
    
    assert(IsNaN(0.0 / 0.0) == true);  // NaN
    assert(IsNaN(1.0) == false);
    assert(IsNaN(0.0) == false);
    assert(IsNaN(-1.0) == false);
    assert(IsNaN(INFINITY) == false);
    
    TEST_PASS("IsNaN - NaN检测");
}

//=============================================================================
// Test: IsEqualTo - 浮点数相等比较
//=============================================================================
void test_IsEqualTo() {
    TEST_START("IsEqualTo - 浮点数相等比较");
    
    assert(IsEqualTo(1.0, 1.0) == true);
    assert(IsEqualTo(0.0, 0.0) == true);
    assert(IsEqualTo(1.0, 1.0 + 1E-10) == true);  // 在误差范围内
    assert(IsEqualTo(1.0, 2.0) == false);
    assert(IsEqualTo(-1.0, -1.0) == true);
    
    TEST_PASS("IsEqualTo - 浮点数相等比较");
}

//=============================================================================
// Test: IsGreaterThan - 大于比较
//=============================================================================
void test_IsGreaterThan() {
    TEST_START("IsGreaterThan - 大于比较");
    
    assert(IsGreaterThan(2.0, 1.0) == true);
    assert(IsGreaterThan(1.0, 2.0) == false);
    assert(IsGreaterThan(1.0, 1.0) == false);
    assert(IsGreaterThan(1.0, 1.0 + 1E-10) == false);  // 相等情况
    
    TEST_PASS("IsGreaterThan - 大于比较");
}

//=============================================================================
// Test: IsLessThan - 小于比较
//=============================================================================
void test_IsLessThan() {
    TEST_START("IsLessThan - 小于比较");
    
    assert(IsLessThan(1.0, 2.0) == true);
    assert(IsLessThan(2.0, 1.0) == false);
    assert(IsLessThan(1.0, 1.0) == false);
    
    TEST_PASS("IsLessThan - 小于比较");
}

//=============================================================================
// Test: IsGreaterThanOrEqual - 大于等于比较
//=============================================================================
void test_IsGreaterThanOrEqual() {
    TEST_START("IsGreaterThanOrEqual - 大于等于比较");
    
    assert(IsGreaterThanOrEqual(2.0, 1.0) == true);
    assert(IsGreaterThanOrEqual(1.0, 1.0) == true);
    assert(IsGreaterThanOrEqual(1.0, 2.0) == false);
    
    TEST_PASS("IsGreaterThanOrEqual - 大于等于比较");
}

//=============================================================================
// Test: IsLessThanOrEqual - 小于等于比较
//=============================================================================
void test_IsLessThanOrEqual() {
    TEST_START("IsLessThanOrEqual - 小于等于比较");
    
    assert(IsLessThanOrEqual(1.0, 2.0) == true);
    assert(IsLessThanOrEqual(1.0, 1.0) == true);
    assert(IsLessThanOrEqual(2.0, 1.0) == false);
    
    TEST_PASS("IsLessThanOrEqual - 小于等于比较");
}

//=============================================================================
// Test: Factorial - 阶乘
//=============================================================================
void test_Factorial() {
    TEST_START("Factorial - 阶乘计算");
    
    assert(Factorial(0) == 1);
    assert(Factorial(1) == 1);
    assert(Factorial(2) == 2);
    assert(Factorial(3) == 6);
    assert(Factorial(4) == 24);
    assert(Factorial(5) == 120);
    assert(Factorial(6) == 720);
    
    TEST_PASS("Factorial - 阶乘计算");
}

//=============================================================================
// Test: Binomial - 二项式系数
//=============================================================================
void test_Binomial() {
    TEST_START("Binomial - 二项式系数");
    
    // C(n, 0) = 1
    ASSERT_DOUBLE_EQ(Binomial(5, 0), 1.0);
    // C(n, n) = 1
    ASSERT_DOUBLE_EQ(Binomial(5, 5), 1.0);
    // C(5, 2) = 10
    ASSERT_DOUBLE_EQ(Binomial(5, 2), 10.0);
    // C(4, 2) = 6
    ASSERT_DOUBLE_EQ(Binomial(4, 2), 6.0);
    
    TEST_PASS("Binomial - 二项式系数");
}

//=============================================================================
// Test: min/max - 最小最大值
//=============================================================================
void test_min_max() {
    TEST_START("min/max - 最小最大值");
    
    assert(min(1, 2) == 1);
    assert(min(2, 1) == 1);
    assert(min(-1, 1) == -1);
    
    assert(max(1, 2) == 2);
    assert(max(2, 1) == 2);
    assert(max(-1, 1) == 1);
    
    TEST_PASS("min/max - 最小最大值");
}

//=============================================================================
// Test: min_double_array - 数组最小值
//=============================================================================
void test_min_double_array() {
    TEST_START("min_double_array - 数组最小值");
    
    double arr1[] = {3.0, 1.0, 4.0, 1.5, 9.0};
    ASSERT_DOUBLE_EQ(min_double_array(arr1, 5), 1.0);
    
    double arr2[] = {-3.0, -1.0, -4.0};
    ASSERT_DOUBLE_EQ(min_double_array(arr2, 3), -4.0);
    
    double arr3[] = {5.0};
    ASSERT_DOUBLE_EQ(min_double_array(arr3, 1), 5.0);
    
    TEST_PASS("min_double_array - 数组最小值");
}

//=============================================================================
// Test: Matrix_Transpose - 矩阵转置
//=============================================================================
void test_Matrix_Transpose() {
    TEST_START("Matrix_Transpose - 矩阵转置");
    
    // 2x3矩阵
    double M[6] = {1, 2, 3, 4, 5, 6};
    double M_tran[6];
    
    Matrix_Transpose(M, 2, 3, M_tran);
    
    // 期望结果：3x2矩阵
    ASSERT_DOUBLE_EQ(M_tran[0], 1.0);  // [0][0]
    ASSERT_DOUBLE_EQ(M_tran[1], 4.0);  // [0][1]
    ASSERT_DOUBLE_EQ(M_tran[2], 2.0);  // [1][0]
    ASSERT_DOUBLE_EQ(M_tran[3], 5.0);  // [1][1]
    ASSERT_DOUBLE_EQ(M_tran[4], 3.0);  // [2][0]
    ASSERT_DOUBLE_EQ(M_tran[5], 6.0);  // [2][1]
    
    TEST_PASS("Matrix_Transpose - 矩阵转置");
}

//=============================================================================
// Test: Matirx_Multiply - 矩阵乘法
//=============================================================================
void test_Matrix_Multiply() {
    TEST_START("Matirx_Multiply - 矩阵乘法");
    
    // 2x3 * 3x2 = 2x2
    double A[6] = {1, 2, 3, 4, 5, 6};  // 2x3
    double B[6] = {7, 8, 9, 10, 11, 12};  // 3x2
    double C[4];  // 2x2
    
    GNC_RES res = Matirx_Multiply(A, 2, 3, B, 3, 2, C);
    
    assert(res == GNC_CAL_SUCCESS);
    // C = A * B
    // [1,2,3] * [7,8]   = [1*7+2*9+3*11, 1*8+2*10+3*12] = [58, 64]
    // [4,5,6]   [9,10]    [4*7+5*9+6*11, 4*8+5*10+6*12]   [139, 154]
    //           [11,12]
    ASSERT_DOUBLE_EQ(C[0], 58.0);
    ASSERT_DOUBLE_EQ(C[1], 64.0);
    ASSERT_DOUBLE_EQ(C[2], 139.0);
    ASSERT_DOUBLE_EQ(C[3], 154.0);
    
    TEST_PASS("Matirx_Multiply - 矩阵乘法");
}

void test_Matrix_Multiply_dimension_mismatch() {
    TEST_START("Matirx_Multiply - 维度不匹配");
    
    double A[6] = {1, 2, 3, 4, 5, 6};  // 2x3
    double B[4] = {1, 2, 3, 4};  // 2x2 (不匹配)
    double C[6];
    
    GNC_RES res = Matirx_Multiply(A, 2, 3, B, 2, 2, C);
    
    assert(res == GNC_CAL_ERROR);
    
    TEST_PASS("Matirx_Multiply - 维度不匹配");
}

//=============================================================================
// Test: Matrix_Inv - 矩阵求逆
//=============================================================================
void test_Matrix_Inv() {
    TEST_START("Matrix_Inv - 矩阵求逆");
    
    // 2x2可逆矩阵 [4, 7; 2, 6]
    double M[4] = {4, 7, 2, 6};
    double M_orig[4] = {4, 7, 2, 6};
    
    GNC_RES res = Matrix_Inv(M, 2);
    
    assert(res == GNC_CAL_SUCCESS);
    
    // 验证 M * M_inv = I
    double I[4];
    Matirx_Multiply(M_orig, 2, 2, M, 2, 2, I);
    
    ASSERT_DOUBLE_EQ(I[0], 1.0);
    ASSERT_DOUBLE_EQ(I[1], 0.0);
    ASSERT_DOUBLE_EQ(I[2], 0.0);
    ASSERT_DOUBLE_EQ(I[3], 1.0);
    
    TEST_PASS("Matrix_Inv - 矩阵求逆");
}

void test_Matrix_Inv_3x3() {
    TEST_START("Matrix_Inv - 3x3矩阵求逆");
    
    // 3x3可逆矩阵
    double M[9] = {1, 2, 3, 0, 1, 4, 5, 6, 0};
    double M_orig[9] = {1, 2, 3, 0, 1, 4, 5, 6, 0};
    
    GNC_RES res = Matrix_Inv(M, 3);
    
    assert(res == GNC_CAL_SUCCESS);
    
    // 验证 M * M_inv = I
    double I[9];
    Matirx_Multiply(M_orig, 3, 3, M, 3, 3, I);
    
    ASSERT_DOUBLE_EQ(I[0], 1.0);
    ASSERT_DOUBLE_EQ(I[4], 1.0);
    ASSERT_DOUBLE_EQ(I[8], 1.0);
    
    TEST_PASS("Matrix_Inv - 3x3矩阵求逆");
}

//=============================================================================
// Test: solve_quadratic_equation - 一元二次方程
//=============================================================================
void test_solve_quadratic_equation_two_roots() {
    TEST_START("solve_quadratic - 两个实根");
    
    // x^2 - 5x + 6 = 0 => x = 2, 3
    float x[2];
    int count = solve_quadratic_equation(1, -5, 6, x);
    
    assert(count == 2);
    // 检查根（可能顺序不同）
    assert((fabsf(x[0] - 2.0f) < 1E-4 && fabsf(x[1] - 3.0f) < 1E-4) ||
           (fabsf(x[0] - 3.0f) < 1E-4 && fabsf(x[1] - 2.0f) < 1E-4));
    
    TEST_PASS("solve_quadratic - 两个实根");
}

void test_solve_quadratic_equation_one_root() {
    TEST_START("solve_quadratic - 一个重根");
    
    // x^2 - 2x + 1 = 0 => x = 1 (重根)
    float x[2];
    int count = solve_quadratic_equation(1, -2, 1, x);
    
    assert(count >= 1);
    ASSERT_FLOAT_EQ(x[0], 1.0f);
    
    TEST_PASS("solve_quadratic - 一个重根");
}

void test_solve_quadratic_equation_no_real_root() {
    TEST_START("solve_quadratic - 无实根");
    
    // x^2 + 1 = 0 => 无实根
    float x[2];
    int count = solve_quadratic_equation(1, 0, 1, x);
    
    assert(count == 0);
    
    TEST_PASS("solve_quadratic - 无实根");
}

void test_solve_quadratic_equation_linear() {
    TEST_START("solve_quadratic - 退化为线性");
    
    // 0*x^2 - 2x + 4 = 0 => x = 2
    float x[2];
    int count = solve_quadratic_equation(0, -2, 4, x);
    
    assert(count == 1);
    ASSERT_FLOAT_EQ(x[0], 2.0f);
    
    TEST_PASS("solve_quadratic - 退化为线性");
}

//=============================================================================
// Test: solve_cubic_equation - 一元三次方程
//=============================================================================
void test_solve_cubic_equation_three_roots() {
    TEST_START("solve_cubic - 三个实根");
    
    // (x-1)(x-2)(x-3) = 0 => x^3 - 6x^2 + 11x - 6 = 0
    float x[3];
    int count = solve_cubic_equation(1, -6, 11, -6, x);
    
    assert(count == 3);
    
    TEST_PASS("solve_cubic - 三个实根");
}

void test_solve_cubic_equation_one_root() {
    TEST_START("solve_cubic - 一个实根");
    
    // (x-1)(x^2+1) = 0 => x^3 - x^2 + x - 1 = 0
    float x[3];
    int count = solve_cubic_equation(1, -1, 1, -1, x);
    
    assert(count == 1);
    ASSERT_FLOAT_EQ(x[0], 1.0f);
    
    TEST_PASS("solve_cubic - 一个实根");
}

void test_solve_cubic_equation_triple_root() {
    TEST_START("solve_cubic - 三重根");
    
    // (x-1)^3 = 0 => x^3 - 3x^2 + 3x - 1 = 0
    float x[3];
    int count = solve_cubic_equation(1, -3, 3, -1, x);
    
    assert(count == 3);
    for (int i = 0; i < count; i++) {
        ASSERT_FLOAT_EQ(x[i], 1.0f);
    }
    
    TEST_PASS("solve_cubic - 三重根");
}

//=============================================================================
// Test: solve_quartic_equation - 一元四次方程
//=============================================================================
void test_solve_quartic_equation_four_roots() {
    TEST_START("solve_quartic - 四个实根");
    
    // (x-1)(x-2)(x-3)(x-4) = 0 => x^4 - 10x^3 + 35x^2 - 50x + 24 = 0
    float x[4];
    int count = solve_quartic_equation(1, -10, 35, -50, 24, x);
    
    assert(count == 4);
    
    TEST_PASS("solve_quartic - 四个实根");
}

void test_solve_quartic_equation_two_roots() {
    TEST_START("solve_quartic - 两个实根");
    
    // (x-1)(x-2)(x^2+1) = 0 => x^4 - 3x^3 + 3x^2 - 3x + 2 = 0
    float x[4];
    int count = solve_quartic_equation(1, -3, 3, -3, 2, x);
    
    assert(count == 2);
    
    TEST_PASS("solve_quartic - 两个实根");
}

//=============================================================================
// Test: get_min_nonnegative_real_root - 最小非负实根
//=============================================================================
void test_get_min_nonnegative_real_root() {
    TEST_START("get_min_nonnegative_real_root - 最小非负实根");
    
    float roots1[] = {3.0f, 1.0f, 5.0f};
    ASSERT_FLOAT_EQ(get_min_nonnegative_real_root(3, roots1), 1.0f);
    
    float roots2[] = {-1.0f, 2.0f, 0.0f};
    ASSERT_FLOAT_EQ(get_min_nonnegative_real_root(3, roots2), 0.0f);
    
    float roots3[] = {-3.0f, -1.0f, -5.0f};
    float result = get_min_nonnegative_real_root(3, roots3);
    assert(result > 1E29);  // 应返回INF
    
    TEST_PASS("get_min_nonnegative_real_root - 最小非负实根");
}

//=============================================================================
// Main Test Runner
//=============================================================================
int main() {
    printf("\n========================================\n");
    printf("  CNC Math 模块单元测试\n");
    printf("========================================\n\n");
    
    // 基础比较函数测试
    test_IsNaN();
    test_IsEqualTo();
    test_IsGreaterThan();
    test_IsLessThan();
    test_IsGreaterThanOrEqual();
    test_IsLessThanOrEqual();
    
    // 数学运算测试
    test_Factorial();
    test_Binomial();
    test_min_max();
    test_min_double_array();
    
    // 矩阵运算测试
    test_Matrix_Transpose();
    test_Matrix_Multiply();
    test_Matrix_Multiply_dimension_mismatch();
    test_Matrix_Inv();
    test_Matrix_Inv_3x3();
    
    // 方程求解测试
    test_solve_quadratic_equation_two_roots();
    test_solve_quadratic_equation_one_root();
    test_solve_quadratic_equation_no_real_root();
    test_solve_quadratic_equation_linear();
    test_solve_cubic_equation_three_roots();
    test_solve_cubic_equation_one_root();
    test_solve_cubic_equation_triple_root();
    test_solve_quartic_equation_four_roots();
    test_solve_quartic_equation_two_roots();
    test_get_min_nonnegative_real_root();
    
    printf("\n========================================\n");
    printf("  所有测试通过!\n");
    printf("========================================\n\n");
    
    return 0;
}

