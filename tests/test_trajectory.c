/******************************************************************** 
 * @file        test_trajectory.c
 * @brief       trajectory 轨迹规划模块单元测试
 * @version     V1.00
 * @date        2025/11/29
 * @description 测试速度曲线计算和轨迹规划函数
 ********************************************************************/

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "../kinematics/trajectory.h"

#define EPSILON 1E-4
#define ASSERT_DOUBLE_EQ(a, b) assert(fabs((a) - (b)) < EPSILON)
#define ASSERT_DOUBLE_GT(a, b) assert((a) > (b))
#define ASSERT_DOUBLE_GE(a, b) assert((a) >= (b) - EPSILON)
#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_START(name) printf("[TEST] %s ... ", name)

// 外部变量声明（测试时需要初始化）
PARAMS test_params;
PARAMS *params = &test_params;

//=============================================================================
// 测试初始化辅助函数
//=============================================================================
void init_test_params() {
    params->cycle_time = 0.001;  // 1ms
    params->feed_vel = 1000.0;   // mm/min
    params->feed_acc = 5000.0;   // mm/s^2
    params->feed_jerk = 50000.0; // mm/s^3
    params->rapid_vel = 5000.0;
    params->rapid_acc = 10000.0;
    params->rapid_jerk = 100000.0;
    params->velplan_type = T_SHAPE;
}

PLAN_BLOCK* create_test_block(INTERP_TYPE type, double length, double start_vel, 
                               double end_vel, double feed_vel) {
    PLAN_BLOCK* block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    block->type = type;
    block->length = length;
    block->start_vel = start_vel;
    block->end_vel = end_vel;
    block->feed_vel = feed_vel;
    block->max_vel = params->feed_vel;
    block->max_acc = params->feed_acc;
    block->max_jerk = params->feed_jerk;
    return block;
}

//=============================================================================
// Test: cal_cri_length - 临界长度计算
//=============================================================================
void test_cal_cri_length_t_shape() {
    TEST_START("cal_cri_length - T型加减速临界长度");
    init_test_params();
    
    // 从0加速到100mm/s，加速度1000mm/s^2
    // 距离 = v^2 / (2*a) = 100^2 / (2*1000) = 5mm
    double vs = 0.0;
    double ve = 100.0;
    double amax = 1000.0;
    double jmax = 10000.0;  // T型不使用
    
    double length = cal_cri_length(T_SHAPE, vs, ve, amax, jmax);
    
    ASSERT_DOUBLE_EQ(length, 5.0);
    
    TEST_PASS("cal_cri_length - T型加减速临界长度");
}

void test_cal_cri_length_zero_delta_v() {
    TEST_START("cal_cri_length - 速度差为0");
    init_test_params();
    
    double length = cal_cri_length(T_SHAPE, 100.0, 100.0, 1000.0, 10000.0);
    
    ASSERT_DOUBLE_EQ(length, 0.0);
    
    TEST_PASS("cal_cri_length - 速度差为0");
}

void test_cal_cri_length_s_shape() {
    TEST_START("cal_cri_length - S型加减速临界长度");
    init_test_params();
    
    // S型曲线计算复杂，只验证返回值合理性
    double vs = 0.0;
    double ve = 100.0;
    double amax = 1000.0;
    double jmax = 10000.0;
    
    double length = cal_cri_length(S_SHAPE, vs, ve, amax, jmax);
    
    // S型的距离应该大于等于T型
    double t_length = cal_cri_length(T_SHAPE, vs, ve, amax, jmax);
    ASSERT_DOUBLE_GE(length, t_length);
    
    TEST_PASS("cal_cri_length - S型加减速临界长度");
}

//=============================================================================
// Test: cal_end_vel - 减速段终点速度计算
//=============================================================================
void test_cal_end_vel_t_shape() {
    TEST_START("cal_end_vel - T型减速终点速度");
    init_test_params();
    
    // 从100mm/s减速，减速度1000mm/s^2，距离5mm
    // ve = sqrt(vs^2 - 2*a*s) = sqrt(10000 - 10000) = 0
    double vs = 100.0;
    double amax = 1000.0;
    double jmax = 10000.0;
    double decline_s = 5.0;
    
    double ve = cal_end_vel(T_SHAPE, vs, amax, jmax, decline_s);
    
    ASSERT_DOUBLE_EQ(ve, 0.0);
    
    TEST_PASS("cal_end_vel - T型减速终点速度");
}

void test_cal_end_vel_partial_decel() {
    TEST_START("cal_end_vel - 部分减速");
    init_test_params();
    
    // 从100mm/s减速，减速度1000mm/s^2，距离2.5mm
    // ve = sqrt(100^2 - 2*1000*2.5) = sqrt(5000) ≈ 70.71
    double vs = 100.0;
    double amax = 1000.0;
    double jmax = 10000.0;
    double decline_s = 2.5;
    
    double ve = cal_end_vel(T_SHAPE, vs, amax, jmax, decline_s);
    
    ASSERT_DOUBLE_EQ(ve, sqrt(5000.0));
    
    TEST_PASS("cal_end_vel - 部分减速");
}

void test_cal_end_vel_zero_distance() {
    TEST_START("cal_end_vel - 距离为0");
    init_test_params();
    
    double ve = cal_end_vel(T_SHAPE, 100.0, 1000.0, 10000.0, 0.0);
    
    ASSERT_DOUBLE_EQ(ve, 100.0);  // 速度不变
    
    TEST_PASS("cal_end_vel - 距离为0");
}

//=============================================================================
// Test: cal_vel_profile - 速度曲线计算
//=============================================================================
void test_cal_vel_profile_t_shape_basic() {
    TEST_START("cal_vel_profile - T型基本速度曲线");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(LINE, 100.0, 0.0, 0.0, 500.0);
    block->max_vel = 500.0;
    block->max_acc = 5000.0;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    // 验证速度曲线的基本属性
    ASSERT_DOUBLE_EQ(profile->vs, 0.0);
    ASSERT_DOUBLE_EQ(profile->ve, 0.0);
    ASSERT_DOUBLE_GE(profile->vpeak, 0.0);
    
    free(profile);
    free(block);
    TEST_PASS("cal_vel_profile - T型基本速度曲线");
}

void test_cal_vel_profile_constant_velocity() {
    TEST_START("cal_vel_profile - 匀速运动");
    init_test_params();
    
    // 起始和终止速度相同，且等于目标速度
    PLAN_BLOCK* block = create_test_block(LINE, 100.0, 100.0, 100.0, 100.0);
    block->max_vel = 100.0;
    block->max_acc = 5000.0;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    ASSERT_DOUBLE_EQ(profile->vs, 100.0);
    ASSERT_DOUBLE_EQ(profile->ve, 100.0);
    ASSERT_DOUBLE_EQ(profile->vpeak, 100.0);
    
    free(profile);
    free(block);
    TEST_PASS("cal_vel_profile - 匀速运动");
}

void test_cal_vel_profile_rapid_move() {
    TEST_START("cal_vel_profile - 快速移动");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(RAPID, 50.0, 0.0, 0.0, 0.0);
    block->max_vel = params->rapid_vel;
    block->max_acc = params->rapid_acc;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    // 快速移动应该使用快速进给参数
    ASSERT_DOUBLE_EQ(profile->vs, 0.0);
    ASSERT_DOUBLE_EQ(profile->ve, 0.0);
    
    free(profile);
    free(block);
    TEST_PASS("cal_vel_profile - 快速移动");
}

//=============================================================================
// Test: cal_traj_distance - 轨迹距离计算
//=============================================================================
void test_cal_traj_distance_zero_time() {
    TEST_START("cal_traj_distance - 零时间");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(LINE, 100.0, 0.0, 0.0, 500.0);
    block->max_acc = 5000.0;
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    double distance = cal_traj_distance(T_SHAPE, profile, 0);
    
    ASSERT_DOUBLE_EQ(distance, 0.0);
    
    free(profile);
    free(block);
    TEST_PASS("cal_traj_distance - 零时间");
}

void test_cal_traj_distance_progression() {
    TEST_START("cal_traj_distance - 距离递增");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(LINE, 100.0, 0.0, 0.0, 500.0);
    block->max_acc = 5000.0;
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    double prev_distance = 0.0;
    for (int cycle = 1; cycle <= 100; cycle += 10) {
        double distance = cal_traj_distance(T_SHAPE, profile, cycle);
        ASSERT_DOUBLE_GE(distance, prev_distance);
        prev_distance = distance;
    }
    
    free(profile);
    free(block);
    TEST_PASS("cal_traj_distance - 距离递增");
}

//=============================================================================
// Test: VEL_PROFILE 结构体验证
//=============================================================================
void test_vel_profile_structure() {
    TEST_START("VEL_PROFILE - 结构体验证");
    
    VEL_PROFILE profile = {0};
    
    // 验证结构体大小和字段
    assert(sizeof(profile.t) == 7 * sizeof(double));
    assert(sizeof(profile.v) == 7 * sizeof(double));
    assert(sizeof(profile.s) == 7 * sizeof(double));
    
    // 设置测试值
    profile.vs = 0.0;
    profile.ve = 100.0;
    profile.vpeak = 200.0;
    profile.ata = 1000.0;
    profile.atd = 1000.0;
    profile.t_cycle = 0.001;
    
    ASSERT_DOUBLE_EQ(profile.vs, 0.0);
    ASSERT_DOUBLE_EQ(profile.ve, 100.0);
    ASSERT_DOUBLE_EQ(profile.vpeak, 200.0);
    
    TEST_PASS("VEL_PROFILE - 结构体验证");
}

//=============================================================================
// Test: 加减速类型枚举
//=============================================================================
void test_velplan_types() {
    TEST_START("VELPLAN_TYPE - 加减速类型");
    
    assert(T_SHAPE == 0);
    assert(S_SHAPE == 1);
    assert(SIN_SHAPE == 2);
    
    TEST_PASS("VELPLAN_TYPE - 加减速类型");
}

//=============================================================================
// Test: 边界条件测试
//=============================================================================
void test_edge_case_very_short_distance() {
    TEST_START("edge_case - 极短距离");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(LINE, 0.001, 0.0, 0.0, 500.0);
    block->max_acc = 5000.0;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    // 极短距离应该能正常处理
    
    free(profile);
    free(block);
    TEST_PASS("edge_case - 极短距离");
}

void test_edge_case_high_velocity() {
    TEST_START("edge_case - 高速运动");
    init_test_params();
    
    PLAN_BLOCK* block = create_test_block(RAPID, 1000.0, 0.0, 0.0, 0.0);
    block->max_vel = 10000.0;  // 10m/s
    block->max_acc = 50000.0;  // 50m/s^2
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    
    free(profile);
    free(block);
    TEST_PASS("edge_case - 高速运动");
}

void test_edge_case_asymmetric_velocity() {
    TEST_START("edge_case - 非对称速度");
    init_test_params();
    
    // 起始速度不为0，终止速度为0
    PLAN_BLOCK* block = create_test_block(LINE, 50.0, 100.0, 0.0, 200.0);
    block->max_acc = 5000.0;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    ASSERT_DOUBLE_EQ(profile->vs, 100.0);
    ASSERT_DOUBLE_EQ(profile->ve, 0.0);
    
    free(profile);
    free(block);
    TEST_PASS("edge_case - 非对称速度");
}

//=============================================================================
// Test: 圆弧运动速度规划
//=============================================================================
void test_circular_motion_velocity() {
    TEST_START("circular_motion - 圆弧运动速度规划");
    init_test_params();
    
    PLAN_BLOCK* block = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));
    block->type = CIRCLE_CCW;
    block->circular_block.center_pos = (POS){.x = 0.0, .y = 0.0, .z = 0.0};
    block->circular_block.radius = 50.0;
    block->circular_block.plan = PMC_PLANE_XY;
    block->circular_block.dir = COUNTERCLOCK;
    block->length = PI * 50.0 / 2;  // 1/4圆
    block->start_vel = 0.0;
    block->end_vel = 0.0;
    block->feed_vel = 200.0;
    block->max_vel = 200.0;
    block->max_acc = 5000.0;
    block->max_jerk = 50000.0;
    
    VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
    
    assert(profile != NULL);
    
    free(profile);
    free(block);
    TEST_PASS("circular_motion - 圆弧运动速度规划");
}

//=============================================================================
// Test: 连续运动场景
//=============================================================================
void test_continuous_motion_scenario() {
    TEST_START("scenario - 连续运动");
    init_test_params();
    
    // 模拟连续的几段运动
    double segment_lengths[] = {50.0, 100.0, 75.0, 25.0};
    double segment_end_vels[] = {50.0, 80.0, 30.0, 0.0};
    
    double current_vel = 0.0;
    
    for (int i = 0; i < 4; i++) {
        PLAN_BLOCK* block = create_test_block(LINE, segment_lengths[i], 
                                              current_vel, segment_end_vels[i], 100.0);
        block->max_acc = 5000.0;
        
        VEL_PROFILE* profile = cal_vel_profile(T_SHAPE, block);
        
        assert(profile != NULL);
        ASSERT_DOUBLE_EQ(profile->vs, current_vel);
        ASSERT_DOUBLE_EQ(profile->ve, segment_end_vels[i]);
        
        current_vel = segment_end_vels[i];
        
        free(profile);
        free(block);
    }
    
    TEST_PASS("scenario - 连续运动");
}

//=============================================================================
// Main Test Runner
//=============================================================================
int main() {
    printf("\n========================================\n");
    printf("  Trajectory 轨迹规划模块单元测试\n");
    printf("========================================\n\n");
    
    // 临界长度计算测试
    test_cal_cri_length_t_shape();
    test_cal_cri_length_zero_delta_v();
    test_cal_cri_length_s_shape();
    
    // 终点速度计算测试
    test_cal_end_vel_t_shape();
    test_cal_end_vel_partial_decel();
    test_cal_end_vel_zero_distance();
    
    // 速度曲线计算测试
    test_cal_vel_profile_t_shape_basic();
    test_cal_vel_profile_constant_velocity();
    test_cal_vel_profile_rapid_move();
    
    // 轨迹距离计算测试
    test_cal_traj_distance_zero_time();
    test_cal_traj_distance_progression();
    
    // 结构体和枚举测试
    test_vel_profile_structure();
    test_velplan_types();
    
    // 边界条件测试
    test_edge_case_very_short_distance();
    test_edge_case_high_velocity();
    test_edge_case_asymmetric_velocity();
    
    // 圆弧运动测试
    test_circular_motion_velocity();
    
    // 场景测试
    test_continuous_motion_scenario();
    
    printf("\n========================================\n");
    printf("  所有测试通过!\n");
    printf("========================================\n\n");
    
    return 0;
}

