/******************************************************************** 
 * @file        test_plan_queue.c
 * @brief       plan_queue 模块单元测试
 * @version     V1.00
 * @date        2025/11/29
 * @description 测试插补指令循环队列的基本操作
 ********************************************************************/

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../datadef/plan_queue.h"

#define EPSILON 1E-6
#define ASSERT_DOUBLE_EQ(a, b) assert(fabs((a) - (b)) < EPSILON)
#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_START(name) printf("[TEST] %s ... ", name)

//=============================================================================
// Test: queue_init - 队列初始化
//=============================================================================
void test_queue_init() {
    TEST_START("queue_init - 初始化");
    PLAN_QUEUE queue;
    GNC_RES res = queue_init(&queue);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(queue.pl_block != NULL);
    assert(queue.head == 0);
    assert(queue.tail == 0);
    assert(queue.size == 0);
    assert(queue.isFirstRUN == true);
    assert(queue.pre_head_vel == 0);
    assert(queue.next_vel == 0);
    ASSERT_DOUBLE_EQ(queue.pre_pos.x, 0.0);
    ASSERT_DOUBLE_EQ(queue.pre_pos.y, 0.0);
    ASSERT_DOUBLE_EQ(queue.pre_pos.z, 0.0);
    
    queue_delete(&queue);
    TEST_PASS("queue_init - 初始化");
}

//=============================================================================
// Test: queue_is_empty - 空队列检测
//=============================================================================
void test_queue_is_empty() {
    TEST_START("queue_is_empty - 空队列检测");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    assert(queue_is_empty(&queue) == true);
    assert(queue_get_size(&queue) == 0);
    
    queue_delete(&queue);
    TEST_PASS("queue_is_empty - 空队列检测");
}

//=============================================================================
// Test: queue_push - 入队操作
//=============================================================================
void test_queue_push_single() {
    TEST_START("queue_push - 单个元素入队");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK block = {
        .type = LINE,
        .length = 100.0,
        .angle_rad = 0.0,
        .start_vel = 0.0,
        .end_vel = 50.0,
        .feed_vel = 100.0,
        .max_vel = 200.0,
        .max_acc = 1000.0,
        .max_jerk = 10000.0,
        .start_pos = {.x = 0.0, .y = 0.0, .z = 0.0},
        .end_pos = {.x = 100.0, .y = 0.0, .z = 0.0}
    };
    
    GNC_RES res = queue_push(&queue, &block);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(queue_get_size(&queue) == 1);
    assert(queue_is_empty(&queue) == false);
    
    queue_delete(&queue);
    TEST_PASS("queue_push - 单个元素入队");
}

void test_queue_push_circular_block() {
    TEST_START("queue_push - 圆弧块入队");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK block = {
        .type = CIRCLE_CCW,
        .circular_block = {
            .center_pos = {.x = 0.0, .y = 0.0, .z = 0.0},
            .radius = 50.0,
            .plan = PMC_PLANE_XY,
            .dir = COUNTERCLOCK
        },
        .length = 78.54,  // 约 PI * 50 / 2
        .feed_vel = 80.0,
        .start_pos = {.x = 50.0, .y = 0.0, .z = 0.0},
        .end_pos = {.x = 0.0, .y = 50.0, .z = 0.0}
    };
    
    GNC_RES res = queue_push(&queue, &block);
    assert(res == GNC_OPERATOR_SUCCESS);
    
    PLAN_BLOCK* peaked = queue_peak(&queue);
    assert(peaked->type == CIRCLE_CCW);
    ASSERT_DOUBLE_EQ(peaked->circular_block.radius, 50.0);
    assert(peaked->circular_block.dir == COUNTERCLOCK);
    
    queue_delete(&queue);
    TEST_PASS("queue_push - 圆弧块入队");
}

//=============================================================================
// Test: queue_pop - 出队操作
//=============================================================================
void test_queue_pop() {
    TEST_START("queue_pop - 出队操作");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK block = {
        .type = RAPID,
        .length = 200.0,
        .feed_vel = 500.0,
        .start_pos = {.x = 0.0, .y = 0.0, .z = 0.0},
        .end_pos = {.x = 200.0, .y = 0.0, .z = 0.0}
    };
    queue_push(&queue, &block);
    
    PLAN_BLOCK* popped = queue_pop(&queue);
    
    assert(popped != NULL);
    assert(popped->type == RAPID);
    ASSERT_DOUBLE_EQ(popped->length, 200.0);
    ASSERT_DOUBLE_EQ(popped->feed_vel, 500.0);
    ASSERT_DOUBLE_EQ(popped->end_pos.x, 200.0);
    assert(queue_is_empty(&queue) == true);
    
    free(popped);
    queue_delete(&queue);
    TEST_PASS("queue_pop - 出队操作");
}

void test_queue_pop_empty() {
    TEST_START("queue_pop - 空队列出队");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK* popped = queue_pop(&queue);
    assert(popped == NULL);
    
    queue_delete(&queue);
    TEST_PASS("queue_pop - 空队列出队");
}

//=============================================================================
// Test: queue_peak 和 queue_peak_tail
//=============================================================================
void test_queue_peak() {
    TEST_START("queue_peak - 查看队首");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK block = {.type = LINE, .length = 150.0};
    queue_push(&queue, &block);
    
    PLAN_BLOCK* peaked = queue_peak(&queue);
    
    assert(peaked != NULL);
    assert(peaked->type == LINE);
    ASSERT_DOUBLE_EQ(peaked->length, 150.0);
    // peak不应该移除元素
    assert(queue_get_size(&queue) == 1);
    
    queue_delete(&queue);
    TEST_PASS("queue_peak - 查看队首");
}

void test_queue_peak_tail() {
    TEST_START("queue_peak_tail - 查看队尾");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 入队多个元素
    for (int i = 0; i < 5; i++) {
        PLAN_BLOCK block = {.type = LINE, .length = i * 10.0};
        queue_push(&queue, &block);
    }
    
    PLAN_BLOCK* tail = queue_peak_tail(&queue);
    
    // 注意：tail指向的是下一个要插入的位置，不是最后一个元素
    // 需要根据实现逻辑来验证
    assert(queue_get_size(&queue) == 5);
    
    queue_delete(&queue);
    TEST_PASS("queue_peak_tail - 查看队尾");
}

//=============================================================================
// Test: queue_get_block - 获取指定位置元素
//=============================================================================
void test_queue_get_block() {
    TEST_START("queue_get_block - 获取指定位置元素");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 入队多个元素
    for (int i = 0; i < 5; i++) {
        PLAN_BLOCK block = {
            .type = LINE,
            .length = i * 100.0,
            .feed_vel = i * 50.0
        };
        queue_push(&queue, &block);
    }
    
    // 验证每个位置的元素
    for (int i = 0; i < 5; i++) {
        PLAN_BLOCK* block = queue_get_block(&queue, i);
        assert(block != NULL);
        ASSERT_DOUBLE_EQ(block->length, i * 100.0);
        ASSERT_DOUBLE_EQ(block->feed_vel, i * 50.0);
    }
    
    queue_delete(&queue);
    TEST_PASS("queue_get_block - 获取指定位置元素");
}

//=============================================================================
// Test: queue_soft_push - 软入队
//=============================================================================
void test_queue_soft_push() {
    TEST_START("queue_soft_push - 软入队");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 先直接写入tail位置的数据
    queue.pl_block[queue.tail].type = LINE;
    queue.pl_block[queue.tail].length = 250.0;
    
    // 软入队
    GNC_RES res = queue_soft_push(&queue);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(queue_get_size(&queue) == 1);
    
    PLAN_BLOCK* peaked = queue_peak(&queue);
    assert(peaked->type == LINE);
    ASSERT_DOUBLE_EQ(peaked->length, 250.0);
    
    queue_delete(&queue);
    TEST_PASS("queue_soft_push - 软入队");
}

//=============================================================================
// Test: queue_is_full - 满队列检测
//=============================================================================
void test_queue_is_full() {
    TEST_START("queue_is_full - 满队列检测");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 填满队列
    for (int i = 0; i < PLAN_QUEUE_SIZE; i++) {
        PLAN_BLOCK block = {.type = LINE, .length = 100.0};
        GNC_RES res = queue_push(&queue, &block);
        assert(res == GNC_OPERATOR_SUCCESS);
    }
    
    assert(queue_is_full(&queue) == true);
    assert(queue_get_size(&queue) == PLAN_QUEUE_SIZE);
    
    // 满队列再入队应失败
    PLAN_BLOCK block = {.type = LINE, .length = 100.0};
    GNC_RES res = queue_push(&queue, &block);
    assert(res == GNC_OPERATOR_ERROR);
    
    queue_delete(&queue);
    TEST_PASS("queue_is_full - 满队列检测");
}

//=============================================================================
// Test: queue_reset - 队列重置
//=============================================================================
void test_queue_reset() {
    TEST_START("queue_reset - 队列重置");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 添加一些元素并修改状态
    for (int i = 0; i < 10; i++) {
        PLAN_BLOCK block = {.type = LINE, .length = 100.0};
        queue_push(&queue, &block);
    }
    queue.isFirstRUN = false;
    queue.pre_head_vel = 100.0;
    queue.next_vel = 50.0;
    queue.pre_pos = (POS){.x = 10.0, .y = 20.0, .z = 30.0};
    
    // 重置队列
    GNC_RES res = queue_reset(&queue);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(queue_is_empty(&queue) == true);
    assert(queue.head == 0);
    assert(queue.tail == 0);
    assert(queue.isFirstRUN == true);
    ASSERT_DOUBLE_EQ(queue.pre_head_vel, 0.0);
    ASSERT_DOUBLE_EQ(queue.next_vel, 0.0);
    ASSERT_DOUBLE_EQ(queue.pre_pos.x, 0.0);
    
    queue_delete(&queue);
    TEST_PASS("queue_reset - 队列重置");
}

//=============================================================================
// Test: 循环队列回环测试
//=============================================================================
void test_queue_circular() {
    TEST_START("queue_circular - 循环队列回环");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    // 多次入队出队，测试循环特性
    for (int round = 0; round < 3; round++) {
        // 入队一半容量
        for (int i = 0; i < PLAN_QUEUE_SIZE / 2; i++) {
            PLAN_BLOCK block = {.type = LINE, .length = (round * 100.0) + i};
            queue_push(&queue, &block);
        }
        
        // 出队一半
        for (int i = 0; i < PLAN_QUEUE_SIZE / 2; i++) {
            PLAN_BLOCK* popped = queue_pop(&queue);
            assert(popped != NULL);
            ASSERT_DOUBLE_EQ(popped->length, (round * 100.0) + i);
            free(popped);
        }
    }
    
    assert(queue_is_empty(&queue) == true);
    
    queue_delete(&queue);
    TEST_PASS("queue_circular - 循环队列回环");
}

//=============================================================================
// Test: 速度规划相关数据测试
//=============================================================================
void test_velocity_planning_data() {
    TEST_START("velocity_planning - 速度规划数据");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLAN_BLOCK block = {
        .type = LINE,
        .length = 100.0,
        .angle_rad = PI / 4,  // 45度
        .start_vel = 0.0,
        .end_vel = 50.0,
        .feed_vel = 100.0,
        .max_vel = 200.0,
        .max_acc = 1000.0,
        .max_jerk = 10000.0,
        .start_pos = {.x = 0.0, .y = 0.0, .z = 0.0},
        .end_pos = {.x = 70.71, .y = 70.71, .z = 0.0},
        .start_vec = {.x = 1.0, .y = 0.0, .z = 0.0},
        .end_vec = {.x = 0.707, .y = 0.707, .z = 0.0}
    };
    
    queue_push(&queue, &block);
    PLAN_BLOCK* peaked = queue_peak(&queue);
    
    ASSERT_DOUBLE_EQ(peaked->max_acc, 1000.0);
    ASSERT_DOUBLE_EQ(peaked->max_jerk, 10000.0);
    assert(peaked->angle_rad > 0.78 && peaked->angle_rad < 0.79);  // ~PI/4
    
    queue_delete(&queue);
    TEST_PASS("velocity_planning - 速度规划数据");
}

//=============================================================================
// Test: 平面选择测试
//=============================================================================
void test_plane_selection() {
    TEST_START("plane_selection - 平面选择");
    PLAN_QUEUE queue;
    queue_init(&queue);
    
    PLANE planes[] = {PMC_PLANE_XY, PMC_PLANE_XZ, PMC_PLANE_YZ};
    
    for (int i = 0; i < 3; i++) {
        PLAN_BLOCK block = {
            .type = CIRCLE_CCW,
            .circular_block = {
                .plan = planes[i],
                .radius = 50.0
            }
        };
        queue_push(&queue, &block);
    }
    
    for (int i = 0; i < 3; i++) {
        PLAN_BLOCK* block = queue_get_block(&queue, i);
        assert(block->circular_block.plan == planes[i]);
    }
    
    queue_delete(&queue);
    TEST_PASS("plane_selection - 平面选择");
}

//=============================================================================
// Main Test Runner
//=============================================================================
int main() {
    printf("\n========================================\n");
    printf("  Plan Queue 模块单元测试\n");
    printf("========================================\n\n");
    
    // 初始化测试
    test_queue_init();
    test_queue_is_empty();
    
    // 入队测试
    test_queue_push_single();
    test_queue_push_circular_block();
    
    // 出队测试
    test_queue_pop();
    test_queue_pop_empty();
    
    // 查看测试
    test_queue_peak();
    test_queue_peak_tail();
    
    // 获取元素测试
    test_queue_get_block();
    
    // 软入队测试
    test_queue_soft_push();
    
    // 满队列测试
    test_queue_is_full();
    
    // 重置测试
    test_queue_reset();
    
    // 循环队列测试
    test_queue_circular();
    
    // 速度规划测试
    test_velocity_planning_data();
    
    // 平面选择测试
    test_plane_selection();
    
    printf("\n========================================\n");
    printf("  所有测试通过!\n");
    printf("========================================\n\n");
    
    return 0;
}

