#include "testlog.h"
#include <QCoreApplication>
#include <QTest>

int main(int argc, char* argv[]) {
    QCoreApplication a(argc, argv);

    TestLog testLog;

    QTest::qExec(&testLog, argc, argv);

    return 0;
}
