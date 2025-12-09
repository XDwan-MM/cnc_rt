#pragma once

#include "regapi.h"
#include "shmapi.h"
#include <QObject>

class TestShm : public QObject {
    Q_OBJECT

private slots:
    void testDoubleInit();
};
