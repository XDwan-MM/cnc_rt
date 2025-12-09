#include "testshm.h"
#include "shmapi.h"
#include <QtTest/QTest>

void TestShm::testDoubleInit() {
    QCOMPARE(shmInit(), true);
    // 第二次初始化失败
    QCOMPARE(shmInit(), false);
}
