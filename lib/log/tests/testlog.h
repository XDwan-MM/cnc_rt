#ifndef TESTLOG_H
#define TESTLOG_H

#include <QtTest>

class TestLog : public QObject
{
    Q_OBJECT
public:
    TestLog();
    ~TestLog();

private slots:
    void initTestCase();
    void cleanupTestCase();
    void cleanup();

    void testAddLog();
    void testAddLogc();
    void testClearLog();
};

#endif // TESTLOG_H
