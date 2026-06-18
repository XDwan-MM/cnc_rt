#include <gtest/gtest.h>

#include "plan_queue.h"

class TestPlanQueue : public ::testing::Test
{
protected:
    static PLAN_BLOCK makeBlock(int lineNumber)
    {
        PLAN_BLOCK block = {};
        block.type = LINE;
        block.line_number = lineNumber;
        block.length = 100.0 + lineNumber;
        block.max_vel = 10.0;
        block.max_acc = 20.0;
        block.max_jerk = 30.0;
        block.end_pos.x = static_cast<double>(lineNumber);
        return block;
    }
};

TEST_F(TestPlanQueue, NullAndUninitializedGuards)
{
    PLAN_BLOCK block = makeBlock(1);

    EXPECT_EQ(queue_init(nullptr), GNC_DATA_NULL);
    EXPECT_EQ(queue_push(nullptr, &block), GNC_DATA_NULL);
    EXPECT_EQ(queue_soft_push(nullptr), GNC_DATA_NULL);
    EXPECT_EQ(queue_pop(nullptr), GNC_DATA_NULL);
    EXPECT_EQ(queue_peak(nullptr), nullptr);
    EXPECT_EQ(queue_peak_tail(nullptr), nullptr);
    EXPECT_EQ(queue_get_block(nullptr, 0), nullptr);
    EXPECT_EQ(queue_get_size(nullptr), -1);
    EXPECT_TRUE(queue_is_empty(nullptr));
    EXPECT_FALSE(queue_is_full(nullptr));

    PLAN_QUEUE queue = {};
    EXPECT_EQ(queue_push(&queue, &block), GNC_DATA_NULL);
    EXPECT_EQ(queue_soft_push(&queue), GNC_DATA_NULL);
    EXPECT_EQ(queue_pop(&queue), GNC_DATA_NULL);
    EXPECT_EQ(queue_peak(&queue), nullptr);
    EXPECT_EQ(queue_peak_tail(&queue), nullptr);
    EXPECT_EQ(queue_get_block(&queue, 0), nullptr);
    EXPECT_EQ(queue_get_size(&queue), -1);
    EXPECT_TRUE(queue_is_empty(&queue));
    EXPECT_FALSE(queue_is_full(&queue));
}

TEST_F(TestPlanQueue, EmptyQueuePeekAndSoftPush)
{
    PLAN_QUEUE queue = {};
    ASSERT_EQ(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    EXPECT_TRUE(queue_is_empty(&queue));
    EXPECT_FALSE(queue_is_full(&queue));
    EXPECT_EQ(queue_get_size(&queue), 0);
    EXPECT_EQ(queue_peak(&queue), nullptr);
    EXPECT_EQ(queue_get_block(&queue, 0), nullptr);
    EXPECT_EQ(queue_pop(&queue), GNC_DATA_EMPTY);

    PLAN_BLOCK *slot = queue_peak_tail(&queue);
    ASSERT_NE(slot, nullptr);
    *slot = makeBlock(42);
    EXPECT_EQ(queue_soft_push(&queue), GNC_OPERATOR_SUCCESS);

    EXPECT_EQ(queue_get_size(&queue), 1);
    EXPECT_FALSE(queue_is_empty(&queue));
    EXPECT_EQ(queue_peak(&queue), slot);
    ASSERT_NE(queue_peak(&queue), nullptr);
    EXPECT_EQ(queue_peak(&queue)->line_number, 42);
    ASSERT_NE(queue_get_block(&queue, 0), nullptr);
    EXPECT_EQ(queue_get_block(&queue, 0)->line_number, 42);

    queue_delete(&queue);
}

TEST_F(TestPlanQueue, FifoOrderAndWraparound)
{
    PLAN_QUEUE queue = {};
    ASSERT_EQ(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK block = makeBlock(i);
        ASSERT_EQ(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    }

    EXPECT_TRUE(queue_is_full(&queue));
    EXPECT_EQ(queue_get_size(&queue), PLAN_QUEUE_SIZE);
    EXPECT_EQ(queue_peak_tail(&queue), nullptr);

    PLAN_BLOCK overflow = makeBlock(PLAN_QUEUE_SIZE);
    EXPECT_EQ(queue_push(&queue, &overflow), GNC_DATA_FULL);
    EXPECT_EQ(queue_soft_push(&queue), GNC_DATA_FULL);
    EXPECT_EQ(queue_get_size(&queue), PLAN_QUEUE_SIZE);

    const int popCount = 75;
    for (int i = 0; i < popCount; ++i) {
        PLAN_BLOCK *head = queue_peak(&queue);
        ASSERT_NE(head, nullptr);
        EXPECT_EQ(head->line_number, i);
        ASSERT_EQ(queue_pop(&queue), GNC_OPERATOR_SUCCESS);
    }

    for (int i = 0; i < popCount; ++i) {
        PLAN_BLOCK block = makeBlock(PLAN_QUEUE_SIZE + i);
        ASSERT_EQ(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    }

    EXPECT_TRUE(queue_is_full(&queue));
    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK *block = queue_get_block(&queue, i);
        ASSERT_NE(block, nullptr);
        EXPECT_EQ(block->line_number, popCount + i);
    }

    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK *head = queue_peak(&queue);
        ASSERT_NE(head, nullptr);
        EXPECT_EQ(head->line_number, popCount + i);
        ASSERT_EQ(queue_pop(&queue), GNC_OPERATOR_SUCCESS);
    }

    EXPECT_TRUE(queue_is_empty(&queue));
    EXPECT_EQ(queue_get_size(&queue), 0);
    EXPECT_EQ(queue_peak(&queue), nullptr);
    EXPECT_EQ(queue_pop(&queue), GNC_DATA_EMPTY);

    queue_delete(&queue);
}

TEST_F(TestPlanQueue, ResetClearsState)
{
    PLAN_QUEUE queue = {};
    ASSERT_EQ(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK block = makeBlock(7);
    ASSERT_EQ(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    queue.pre_pos = (POS){.x = 1.0, .y = 2.0, .z = 3.0};
    queue.pre_vec = (VECTOR){.x = 4.0, .y = 5.0, .z = 6.0};
    queue.next_vel = 9.0;
    queue.isFirstRUN = false;
    queue.pre_head_type = LINE;
    queue.pre_head_vel = 8.0;

    EXPECT_EQ(queue_reset(&queue), GNC_OPERATOR_SUCCESS);

    EXPECT_EQ(queue.head, 0);
    EXPECT_EQ(queue.tail, 0);
    EXPECT_EQ(queue_get_size(&queue), 0);
    EXPECT_TRUE(queue_is_empty(&queue));
    EXPECT_TRUE(queue.isFirstRUN);
    EXPECT_EQ(queue.pre_head_type, RAPID);
    EXPECT_EQ(queue.pre_head_vel, 0.0);
    EXPECT_EQ(queue.next_vel, 0.0);
    EXPECT_EQ(queue.pre_pos.x, 0.0);
    EXPECT_EQ(queue.pre_pos.y, 0.0);
    EXPECT_EQ(queue.pre_pos.z, 0.0);
    EXPECT_EQ(queue.pre_vec.x, 0.0);
    EXPECT_EQ(queue.pre_vec.y, 0.0);
    EXPECT_EQ(queue.pre_vec.z, 0.0);
    EXPECT_EQ(queue_peak(&queue), nullptr);

    queue_delete(&queue);
}

TEST_F(TestPlanQueue, CopyBlockBounds)
{
    PLAN_QUEUE queue = {};
    ASSERT_EQ(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK source = makeBlock(11);
    ASSERT_EQ(queue_push(&queue, &source), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK copy = makeBlock(99);
    queue_copy_block(&queue, -1, &copy);
    EXPECT_EQ(copy.line_number, 99);

    queue_copy_block(&queue, 1, &copy);
    EXPECT_EQ(copy.line_number, 99);

    queue_copy_block(&queue, 0, nullptr);

    queue_copy_block(&queue, 0, &copy);
    EXPECT_EQ(copy.line_number, 11);
    EXPECT_EQ(copy.length, source.length);
    EXPECT_EQ(copy.end_pos.x, source.end_pos.x);

    queue_delete(&queue);
}
