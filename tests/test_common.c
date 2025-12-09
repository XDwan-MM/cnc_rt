/******************************************************************** 
 * @file        test_common.c
 * @brief       common 模块单元测试
 * @version     V1.00
 * @date        2025/11/29
 * @description 测试 POS 数据结构及其相关操作函数
 ********************************************************************/

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../datadef/common.h"

#define EPSILON 1E-6

// 辅助断言宏
#define ASSERT_DOUBLE_EQ(a, b) assert(fabs((a) - (b)) < EPSILON)
#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_START(name) printf("[TEST] %s ... ", name)

//=============================================================================
// Test: POS_Distance - 两点间距离计算
//=============================================================================
void test_POS_Distance_same_point() {
    TEST_START("POS_Distance - 同一点距离为0");
    POS p1 = {0.0, 0.0, 0.0};
    POS p2 = {0.0, 0.0, 0.0};
    double dist = POS_Distance(&p1, &p2);
    ASSERT_DOUBLE_EQ(dist, 0.0);
    TEST_PASS("POS_Distance - 同一点距离为0");
}

void test_POS_Distance_unit_distance() {
    TEST_START("POS_Distance - 单位距离");
    POS p1 = {0.0, 0.0, 0.0};
    POS p2 = {1.0, 0.0, 0.0};
    double dist = POS_Distance(&p1, &p2);
    ASSERT_DOUBLE_EQ(dist, 1.0);
    TEST_PASS("POS_Distance - 单位距离");
}

void test_POS_Distance_3d_diagonal() {
    TEST_START("POS_Distance - 3D对角线距离");
    POS p1 = {0.0, 0.0, 0.0};
    POS p2 = {1.0, 1.0, 1.0};
    double dist = POS_Distance(&p1, &p2);
    ASSERT_DOUBLE_EQ(dist, sqrt(3.0));
    TEST_PASS("POS_Distance - 3D对角线距离");
}

void test_POS_Distance_negative_coords() {
    TEST_START("POS_Distance - 负坐标");
    POS p1 = {-1.0, -1.0, -1.0};
    POS p2 = {1.0, 1.0, 1.0};
    double dist = POS_Distance(&p1, &p2);
    ASSERT_DOUBLE_EQ(dist, sqrt(12.0));  // 2*sqrt(3)
    TEST_PASS("POS_Distance - 负坐标");
}

//=============================================================================
// Test: POS_VECTOR - 向量计算
//=============================================================================
void test_POS_VECTOR_zero_vector() {
    TEST_START("POS_VECTOR - 零向量");
    POS p1 = {1.0, 2.0, 3.0};
    POS p2 = {1.0, 2.0, 3.0};
    POS vec = POS_VECTOR(&p1, &p2);
    ASSERT_DOUBLE_EQ(vec.x, 0.0);
    ASSERT_DOUBLE_EQ(vec.y, 0.0);
    ASSERT_DOUBLE_EQ(vec.z, 0.0);
    TEST_PASS("POS_VECTOR - 零向量");
}

void test_POS_VECTOR_unit_x() {
    TEST_START("POS_VECTOR - X方向单位向量");
    POS p1 = {0.0, 0.0, 0.0};
    POS p2 = {1.0, 0.0, 0.0};
    POS vec = POS_VECTOR(&p1, &p2);
    ASSERT_DOUBLE_EQ(vec.x, 1.0);
    ASSERT_DOUBLE_EQ(vec.y, 0.0);
    ASSERT_DOUBLE_EQ(vec.z, 0.0);
    TEST_PASS("POS_VECTOR - X方向单位向量");
}

void test_POS_VECTOR_diagonal() {
    TEST_START("POS_VECTOR - 对角向量");
    POS p1 = {1.0, 2.0, 3.0};
    POS p2 = {4.0, 6.0, 8.0};
    POS vec = POS_VECTOR(&p1, &p2);
    ASSERT_DOUBLE_EQ(vec.x, 3.0);
    ASSERT_DOUBLE_EQ(vec.y, 4.0);
    ASSERT_DOUBLE_EQ(vec.z, 5.0);
    TEST_PASS("POS_VECTOR - 对角向量");
}

//=============================================================================
// Test: POS_Norm - 向量模长
//=============================================================================
void test_POS_Norm_zero_vector() {
    TEST_START("POS_Norm - 零向量模长");
    POS p = {0.0, 0.0, 0.0};
    double norm = POS_Norm(p);
    ASSERT_DOUBLE_EQ(norm, 0.0);
    TEST_PASS("POS_Norm - 零向量模长");
}

void test_POS_Norm_unit_vector() {
    TEST_START("POS_Norm - 单位向量模长");
    POS p = {1.0, 0.0, 0.0};
    double norm = POS_Norm(p);
    ASSERT_DOUBLE_EQ(norm, 1.0);
    TEST_PASS("POS_Norm - 单位向量模长");
}

void test_POS_Norm_3d_vector() {
    TEST_START("POS_Norm - 3D向量模长");
    POS p = {3.0, 4.0, 0.0};  // 3-4-5三角形
    double norm = POS_Norm(p);
    ASSERT_DOUBLE_EQ(norm, 5.0);
    TEST_PASS("POS_Norm - 3D向量模长");
}

//=============================================================================
// Test: POS_Dot - 向量点积
//=============================================================================
void test_POS_Dot_perpendicular() {
    TEST_START("POS_Dot - 垂直向量点积为0");
    POS p1 = {1.0, 0.0, 0.0};
    POS p2 = {0.0, 1.0, 0.0};
    double dot = POS_Dot(p1, p2);
    ASSERT_DOUBLE_EQ(dot, 0.0);
    TEST_PASS("POS_Dot - 垂直向量点积为0");
}

void test_POS_Dot_parallel() {
    TEST_START("POS_Dot - 平行向量点积");
    POS p1 = {1.0, 0.0, 0.0};
    POS p2 = {2.0, 0.0, 0.0};
    double dot = POS_Dot(p1, p2);
    ASSERT_DOUBLE_EQ(dot, 2.0);
    TEST_PASS("POS_Dot - 平行向量点积");
}

void test_POS_Dot_general() {
    TEST_START("POS_Dot - 一般向量点积");
    POS p1 = {1.0, 2.0, 3.0};
    POS p2 = {4.0, 5.0, 6.0};
    double dot = POS_Dot(p1, p2);
    ASSERT_DOUBLE_EQ(dot, 32.0);  // 1*4 + 2*5 + 3*6
    TEST_PASS("POS_Dot - 一般向量点积");
}

//=============================================================================
// Main Test Runner
//=============================================================================
int main() {
    printf("\n========================================\n");
    printf("  Common 模块单元测试\n");
    printf("========================================\n\n");
    
    // POS_Distance 测试
    test_POS_Distance_same_point();
    test_POS_Distance_unit_distance();
    test_POS_Distance_3d_diagonal();
    test_POS_Distance_negative_coords();
    
    // POS_VECTOR 测试
    test_POS_VECTOR_zero_vector();
    test_POS_VECTOR_unit_x();
    test_POS_VECTOR_diagonal();
    
    // POS_Norm 测试
    test_POS_Norm_zero_vector();
    test_POS_Norm_unit_vector();
    test_POS_Norm_3d_vector();
    
    // POS_Dot 测试
    test_POS_Dot_perpendicular();
    test_POS_Dot_parallel();
    test_POS_Dot_general();
    
    printf("\n========================================\n");
    printf("  所有测试通过!\n");
    printf("========================================\n\n");
    
    return 0;
}

