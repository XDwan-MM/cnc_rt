/******************************************************************** 
 * @file        test_gcode_queue.c
 * @brief       gcode_queue 模块单元测试
 * @version     V1.00
 * @date        2025/11/29
 * @description 测试 G代码循环队列的基本操作
 ********************************************************************/

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "../datadef/gcode_queue.h"

#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_START(name) printf("[TEST] %s ... ", name)

//=============================================================================
// Test: gcode_queue_init - 队列初始化
//=============================================================================
void test_gcode_queue_init() {
    TEST_START("gcode_queue_init - 初始化");
    GCODE_QUEUE queue;
    GNC_RES res = gcode_queue_init(&queue);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(queue.block != NULL);
    assert(queue.head == 0);
    assert(queue.tail == 0);
    assert(queue.size == 0);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_init - 初始化");
}

//=============================================================================
// Test: gcode_queue_is_empty - 空队列检测
//=============================================================================
void test_gcode_queue_is_empty() {
    TEST_START("gcode_queue_is_empty - 空队列检测");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    assert(gcode_queue_is_empty(&queue) == true);
    assert(gcode_queue_get_size(&queue) == 0);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_is_empty - 空队列检测");
}

//=============================================================================
// Test: gcode_queue_push - 入队操作
//=============================================================================
void test_gcode_queue_push_single() {
    TEST_START("gcode_queue_push - 单个元素入队");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    GCODE_BLOCK block = {
        .type = LINE,
        .feed_vel = 100.0,
        .pos = {.x = 10.0, .y = 20.0, .z = 5.0}
    };
    
    GNC_RES res = gcode_queue_push(&queue, &block);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(gcode_queue_get_size(&queue) == 1);
    assert(gcode_queue_is_empty(&queue) == false);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_push - 单个元素入队");
}

void test_gcode_queue_push_multiple() {
    TEST_START("gcode_queue_push - 多个元素入队");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    for (int i = 0; i < 10; i++) {
        GCODE_BLOCK block = {
            .type = LINE,
            .feed_vel = 100.0 * i,
            .pos = {.x = i * 1.0, .y = i * 2.0, .z = i * 3.0}
        };
        GNC_RES res = gcode_queue_push(&queue, &block);
        assert(res == GNC_OPERATOR_SUCCESS);
    }
    
    assert(gcode_queue_get_size(&queue) == 10);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_push - 多个元素入队");
}

//=============================================================================
// Test: gcode_queue_pop - 出队操作
//=============================================================================
void test_gcode_queue_pop() {
    TEST_START("gcode_queue_pop - 出队操作");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    // 入队
    GCODE_BLOCK block = {
        .type = RAPID,
        .feed_vel = 500.0,
        .pos = {.x = 100.0, .y = 200.0, .z = 50.0}
    };
    gcode_queue_push(&queue, &block);
    
    // 出队
    GCODE_BLOCK* popped = gcode_queue_pop(&queue);
    
    assert(popped != NULL);
    assert(popped->type == RAPID);
    assert(popped->feed_vel == 500.0);
    assert(popped->pos.x == 100.0);
    assert(popped->pos.y == 200.0);
    assert(popped->pos.z == 50.0);
    assert(gcode_queue_is_empty(&queue) == true);
    
    free(popped);
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_pop - 出队操作");
}

void test_gcode_queue_pop_empty() {
    TEST_START("gcode_queue_pop - 空队列出队");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    GCODE_BLOCK* popped = gcode_queue_pop(&queue);
    assert(popped == NULL);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_pop - 空队列出队");
}

//=============================================================================
// Test: gcode_queue_peak - 查看队首
//=============================================================================
void test_gcode_queue_peak() {
    TEST_START("gcode_queue_peak - 查看队首");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    GCODE_BLOCK block = {
        .type = CIRCLE_CCW,
        .feed_vel = 150.0,
        .pos = {.x = 50.0, .y = 50.0, .z = 0.0}
    };
    gcode_queue_push(&queue, &block);
    
    GCODE_BLOCK* peaked = gcode_queue_peak(&queue);
    
    assert(peaked != NULL);
    assert(peaked->type == CIRCLE_CCW);
    assert(peaked->feed_vel == 150.0);
    // peak不应该移除元素
    assert(gcode_queue_get_size(&queue) == 1);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_peak - 查看队首");
}

//=============================================================================
// Test: gcode_queue_get_block - 获取指定位置元素
//=============================================================================
void test_gcode_queue_get_block() {
    TEST_START("gcode_queue_get_block - 获取指定位置元素");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    // 入队多个元素
    for (int i = 0; i < 5; i++) {
        GCODE_BLOCK block = {
            .type = LINE,
            .feed_vel = i * 100.0,
            .pos = {.x = i * 10.0, .y = i * 20.0, .z = i * 30.0}
        };
        gcode_queue_push(&queue, &block);
    }
    
    // 验证每个位置的元素
    for (int i = 0; i < 5; i++) {
        GCODE_BLOCK* block = gcode_queue_get_block(&queue, i);
        assert(block != NULL);
        assert(block->feed_vel == i * 100.0);
        assert(block->pos.x == i * 10.0);
    }
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_get_block - 获取指定位置元素");
}

void test_gcode_queue_get_block_invalid_index() {
    TEST_START("gcode_queue_get_block - 无效索引");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    GCODE_BLOCK block = {.type = LINE, .feed_vel = 100.0};
    gcode_queue_push(&queue, &block);
    
    // 测试无效索引
    assert(gcode_queue_get_block(&queue, -1) == NULL);
    assert(gcode_queue_get_block(&queue, 5) == NULL);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_get_block - 无效索引");
}

//=============================================================================
// Test: gcode_queue_is_full - 满队列检测
//=============================================================================
void test_gcode_queue_is_full() {
    TEST_START("gcode_queue_is_full - 满队列检测");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    // 填满队列
    for (int i = 0; i < DCD_BUF_SIZE; i++) {
        GCODE_BLOCK block = {.type = LINE, .feed_vel = 100.0};
        GNC_RES res = gcode_queue_push(&queue, &block);
        assert(res == GNC_OPERATOR_SUCCESS);
    }
    
    assert(gcode_queue_is_full(&queue) == true);
    assert(gcode_queue_get_size(&queue) == DCD_BUF_SIZE);
    
    // 满队列再入队应失败
    GCODE_BLOCK block = {.type = LINE, .feed_vel = 100.0};
    GNC_RES res = gcode_queue_push(&queue, &block);
    assert(res == GNC_OPERATOR_ERROR);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_is_full - 满队列检测");
}

//=============================================================================
// Test: gcode_queue_reset - 队列重置
//=============================================================================
void test_gcode_queue_reset() {
    TEST_START("gcode_queue_reset - 队列重置");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    // 添加一些元素
    for (int i = 0; i < 10; i++) {
        GCODE_BLOCK block = {.type = LINE, .feed_vel = 100.0};
        gcode_queue_push(&queue, &block);
    }
    
    assert(gcode_queue_get_size(&queue) == 10);
    
    // 重置队列
    GNC_RES res = gcode_queue_reset(&queue);
    
    assert(res == GNC_OPERATOR_SUCCESS);
    assert(gcode_queue_is_empty(&queue) == true);
    assert(queue.head == 0);
    assert(queue.tail == 0);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_reset - 队列重置");
}

//=============================================================================
// Test: 循环队列回环测试
//=============================================================================
void test_gcode_queue_circular() {
    TEST_START("gcode_queue_circular - 循环队列回环");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    // 多次入队出队，测试循环特性
    for (int round = 0; round < 3; round++) {
        // 入队一半容量
        for (int i = 0; i < DCD_BUF_SIZE / 2; i++) {
            GCODE_BLOCK block = {.type = LINE, .feed_vel = (round * 100.0) + i};
            gcode_queue_push(&queue, &block);
        }
        
        // 出队一半
        for (int i = 0; i < DCD_BUF_SIZE / 2; i++) {
            GCODE_BLOCK* popped = gcode_queue_pop(&queue);
            assert(popped != NULL);
            assert(popped->feed_vel == (round * 100.0) + i);
            free(popped);
        }
    }
    
    assert(gcode_queue_is_empty(&queue) == true);
    
    gcode_queue_delete(&queue);
    TEST_PASS("gcode_queue_circular - 循环队列回环");
}

//=============================================================================
// Test: INTERP_TYPE 枚举值测试
//=============================================================================
void test_interp_types() {
    TEST_START("INTERP_TYPE - 插补类型枚举");
    GCODE_QUEUE queue;
    gcode_queue_init(&queue);
    
    INTERP_TYPE types[] = {RAPID, LINE, CIRCLE_CCW, CIRCLE_CW, HELIX, SPLINE, RIGID_TAP, PROBE};
    
    for (int i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        GCODE_BLOCK block = {.type = types[i], .feed_vel = 100.0};
        gcode_queue_push(&queue, &block);
    }
    
    for (int i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        GCODE_BLOCK* popped = gcode_queue_pop(&queue);
        assert(popped->type == types[i]);
        free(popped);
    }
    
    gcode_queue_delete(&queue);
    TEST_PASS("INTERP_TYPE - 插补类型枚举");
}

//=============================================================================
// Main Test Runner
//=============================================================================
int main() {
    printf("\n========================================\n");
    printf("  GCode Queue 模块单元测试\n");
    printf("========================================\n\n");
    
    // 初始化测试
    test_gcode_queue_init();
    test_gcode_queue_is_empty();
    
    // 入队测试
    test_gcode_queue_push_single();
    test_gcode_queue_push_multiple();
    
    // 出队测试
    test_gcode_queue_pop();
    test_gcode_queue_pop_empty();
    
    // 查看测试
    test_gcode_queue_peak();
    
    // 获取元素测试
    test_gcode_queue_get_block();
    test_gcode_queue_get_block_invalid_index();
    
    // 满队列测试
    test_gcode_queue_is_full();
    
    // 重置测试
    test_gcode_queue_reset();
    
    // 循环队列测试
    test_gcode_queue_circular();
    
    // 枚举类型测试
    test_interp_types();
    
    printf("\n========================================\n");
    printf("  所有测试通过!\n");
    printf("========================================\n\n");
    
    return 0;
}

