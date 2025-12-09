#include "testreg.h"
#include "regapi.h"
#include "shmapi.h"
#include <QtTest/QTest>

void TestReg::initTestCase() {
}

void TestReg::cleanupTestCase() {
}

void TestReg::testSetRegBit() {
    QCOMPARE(setRegBit(REG_R, 0, 0), REG_API_SUCCESS);
    QCOMPARE(setRegBit(REG_R, 0, 1), REG_API_SUCCESS);
    QCOMPARE(setRegBit(REG_R, 0, 8), REG_API_BIT_ERROR);
    QCOMPARE(setRegBit(REG_R, -1, 0), REG_API_IDX_ERROR);
    QCOMPARE(setRegBit(REG_R, 0, -1), REG_API_BIT_ERROR);
}

void TestReg::testClearRegBit() {
    QCOMPARE(clearRegBit(REG_R, 0, 0), REG_API_SUCCESS);
    QCOMPARE(clearRegBit(REG_R, 0, 1), REG_API_SUCCESS);
    QCOMPARE(clearRegBit(REG_R, 0, 8), REG_API_BIT_ERROR);
    QCOMPARE(clearRegBit(REG_R, -1, 0), REG_API_IDX_ERROR);
    QCOMPARE(clearRegBit(REG_R, 0, -1), REG_API_BIT_ERROR);
}
