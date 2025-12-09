#pragma once

#include "ladapi.h"
#include <QObject>

class TestLad : public QObject {
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
};
