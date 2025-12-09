#include "testreg.h"
#include "testlad.h"
#include "testshm.h"
#include <QCoreApplication>
#include <QProcess>
#include <QTest>
#include <thread>

int main(int argc, char* argv[]) {
    QCoreApplication a(argc, argv);

    TestShm testShm;
    QTest::qExec(&testShm, argc, argv);

    TestReg testReg;
    QTest::qExec(&testReg, argc, argv);

    TestLad testLad;
    QTest::qExec(&testLad, argc, argv);
}
