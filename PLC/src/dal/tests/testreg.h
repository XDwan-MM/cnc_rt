#pragma once

#include "regapi.h"
#include <QObject>

class TestReg : public QObject {
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();

    void testSetRegBit();
    void testClearRegBit();
    // void testGetRegBit();
    // void testSetRegValue8();
    // void testSetRegValue16();
    // void testSetRegValue32();
    // void testGetRegValue8();
    // void testGetRegValue16();
    // void testGetRegValue32();
    // void testSetTimerCounterCurrent();
    // void testSetTimerCounterPreset();
    // void testGetTimerCounter();
    // void testGetTimerCounterCurrent();
    // void testGetTimerCounterPreset();
};

