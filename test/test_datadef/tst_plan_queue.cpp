#include <QtTest>

#include "test_registry.h"

extern "C" {
#include "plan_queue.h"
}

class TestPlanQueue : public QObject
{
    Q_OBJECT

private:
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

private slots:
    void test_null_and_uninitialized_guards();
    void test_empty_queue_peek_and_soft_push();
    void test_fifo_order_and_wraparound();
    void test_reset_clears_state();
    void test_copy_block_bounds();
};

void TestPlanQueue::test_null_and_uninitialized_guards()
{
    PLAN_BLOCK block = makeBlock(1);

    QCOMPARE(queue_init(nullptr), GNC_DATA_NULL);
    QCOMPARE(queue_push(nullptr, &block), GNC_DATA_NULL);
    QCOMPARE(queue_soft_push(nullptr), GNC_DATA_NULL);
    QCOMPARE(queue_pop(nullptr), GNC_DATA_NULL);
    QVERIFY(queue_peak(nullptr) == nullptr);
    QVERIFY(queue_peak_tail(nullptr) == nullptr);
    QVERIFY(queue_get_block(nullptr, 0) == nullptr);
    QCOMPARE(queue_get_size(nullptr), -1);
    QVERIFY(queue_is_empty(nullptr));
    QVERIFY(!queue_is_full(nullptr));

    PLAN_QUEUE queue = {};
    QCOMPARE(queue_push(&queue, &block), GNC_DATA_NULL);
    QCOMPARE(queue_soft_push(&queue), GNC_DATA_NULL);
    QCOMPARE(queue_pop(&queue), GNC_DATA_NULL);
    QVERIFY(queue_peak(&queue) == nullptr);
    QVERIFY(queue_peak_tail(&queue) == nullptr);
    QVERIFY(queue_get_block(&queue, 0) == nullptr);
    QCOMPARE(queue_get_size(&queue), -1);
    QVERIFY(queue_is_empty(&queue));
    QVERIFY(!queue_is_full(&queue));
}

void TestPlanQueue::test_empty_queue_peek_and_soft_push()
{
    PLAN_QUEUE queue = {};
    QCOMPARE(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    QVERIFY(queue_is_empty(&queue));
    QVERIFY(!queue_is_full(&queue));
    QCOMPARE(queue_get_size(&queue), 0);
    QVERIFY(queue_peak(&queue) == nullptr);
    QVERIFY(queue_get_block(&queue, 0) == nullptr);
    QCOMPARE(queue_pop(&queue), GNC_DATA_EMPTY);

    PLAN_BLOCK *slot = queue_peak_tail(&queue);
    QVERIFY(slot != nullptr);
    *slot = makeBlock(42);
    QCOMPARE(queue_soft_push(&queue), GNC_OPERATOR_SUCCESS);

    QCOMPARE(queue_get_size(&queue), 1);
    QVERIFY(!queue_is_empty(&queue));
    QVERIFY(queue_peak(&queue) == slot);
    QCOMPARE(queue_peak(&queue)->line_number, 42);
    QCOMPARE(queue_get_block(&queue, 0)->line_number, 42);

    queue_delete(&queue);
}

void TestPlanQueue::test_fifo_order_and_wraparound()
{
    PLAN_QUEUE queue = {};
    QCOMPARE(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK block = makeBlock(i);
        QCOMPARE(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    }

    QVERIFY(queue_is_full(&queue));
    QCOMPARE(queue_get_size(&queue), PLAN_QUEUE_SIZE);
    QVERIFY(queue_peak_tail(&queue) == nullptr);

    PLAN_BLOCK overflow = makeBlock(PLAN_QUEUE_SIZE);
    QCOMPARE(queue_push(&queue, &overflow), GNC_DATA_FULL);
    QCOMPARE(queue_soft_push(&queue), GNC_DATA_FULL);
    QCOMPARE(queue_get_size(&queue), PLAN_QUEUE_SIZE);

    const int popCount = 75;
    for (int i = 0; i < popCount; ++i) {
        PLAN_BLOCK *head = queue_peak(&queue);
        QVERIFY(head != nullptr);
        QCOMPARE(head->line_number, i);
        QCOMPARE(queue_pop(&queue), GNC_OPERATOR_SUCCESS);
    }

    for (int i = 0; i < popCount; ++i) {
        PLAN_BLOCK block = makeBlock(PLAN_QUEUE_SIZE + i);
        QCOMPARE(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    }

    QVERIFY(queue_is_full(&queue));
    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK *block = queue_get_block(&queue, i);
        QVERIFY(block != nullptr);
        QCOMPARE(block->line_number, popCount + i);
    }

    for (int i = 0; i < PLAN_QUEUE_SIZE; ++i) {
        PLAN_BLOCK *head = queue_peak(&queue);
        QVERIFY(head != nullptr);
        QCOMPARE(head->line_number, popCount + i);
        QCOMPARE(queue_pop(&queue), GNC_OPERATOR_SUCCESS);
    }

    QVERIFY(queue_is_empty(&queue));
    QCOMPARE(queue_get_size(&queue), 0);
    QVERIFY(queue_peak(&queue) == nullptr);
    QCOMPARE(queue_pop(&queue), GNC_DATA_EMPTY);

    queue_delete(&queue);
}

void TestPlanQueue::test_reset_clears_state()
{
    PLAN_QUEUE queue = {};
    QCOMPARE(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK block = makeBlock(7);
    QCOMPARE(queue_push(&queue, &block), GNC_OPERATOR_SUCCESS);
    queue.pre_pos = (POS){.x = 1.0, .y = 2.0, .z = 3.0};
    queue.pre_vec = (VECTOR){.x = 4.0, .y = 5.0, .z = 6.0};
    queue.next_vel = 9.0;
    queue.isFirstRUN = false;
    queue.pre_head_type = LINE;
    queue.pre_head_vel = 8.0;

    QCOMPARE(queue_reset(&queue), GNC_OPERATOR_SUCCESS);

    QCOMPARE(queue.head, 0);
    QCOMPARE(queue.tail, 0);
    QCOMPARE(queue_get_size(&queue), 0);
    QVERIFY(queue_is_empty(&queue));
    QVERIFY(queue.isFirstRUN);
    QCOMPARE(queue.pre_head_type, RAPID);
    QCOMPARE(queue.pre_head_vel, 0.0);
    QCOMPARE(queue.next_vel, 0.0);
    QCOMPARE(queue.pre_pos.x, 0.0);
    QCOMPARE(queue.pre_pos.y, 0.0);
    QCOMPARE(queue.pre_pos.z, 0.0);
    QCOMPARE(queue.pre_vec.x, 0.0);
    QCOMPARE(queue.pre_vec.y, 0.0);
    QCOMPARE(queue.pre_vec.z, 0.0);
    QVERIFY(queue_peak(&queue) == nullptr);

    queue_delete(&queue);
}

void TestPlanQueue::test_copy_block_bounds()
{
    PLAN_QUEUE queue = {};
    QCOMPARE(queue_init(&queue), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK source = makeBlock(11);
    QCOMPARE(queue_push(&queue, &source), GNC_OPERATOR_SUCCESS);

    PLAN_BLOCK copy = makeBlock(99);
    queue_copy_block(&queue, -1, &copy);
    QCOMPARE(copy.line_number, 99);

    queue_copy_block(&queue, 1, &copy);
    QCOMPARE(copy.line_number, 99);

    queue_copy_block(&queue, 0, nullptr);

    queue_copy_block(&queue, 0, &copy);
    QCOMPARE(copy.line_number, 11);
    QCOMPARE(copy.length, source.length);
    QCOMPARE(copy.end_pos.x, source.end_pos.x);

    queue_delete(&queue);
}

REGISTER_TEST(TestPlanQueue)

#include "tst_plan_queue.moc"
