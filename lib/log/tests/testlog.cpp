#include "testlog.h"
#include "logmanager.h"
#include "logentry.h"
#include "spdlogmanager.h"
#include "logapi.h"

TestLog::TestLog()
{

}

TestLog::~TestLog()
{
}

void TestLog::initTestCase()
{

}

void TestLog::cleanupTestCase()
{
    clearAllLog();
}

void TestLog::cleanup()
{
//    clearAllLog();
}

void TestLog::testAddLog()
{
    LOG_WORKINFO("[testAddLog] workinfo...");
    LOG_FILE("[testAddLog] file...");
    LOG_PANELKEY("[testAddLog] panelkey...");
    LOG_DEFINE("[testAddLog] define...");
    LOG_SYSTEM("[testAddLog] system...");
    LOG_SAVE();

    QList<LogEntry> allLogs;
    QCOMPARE(0, getAllLog(allLogs));
    QCOMPARE(5, allLogs.count());
    QCOMPARE("[testAddLog] workinfo...", allLogs[0].eventText);
    QCOMPARE("workinfo", allLogs[0].eventType);
    QCOMPARE("[testAddLog] file...", allLogs[1].eventText);
    QCOMPARE("[testAddLog] system...", allLogs[4].eventText);

    // 多输入
    LOG_MULTI(0, "[testAddLog] workinfo a: %d, b: %f", 123, 3.14);
    LOG_SAVE();

    QCOMPARE(0, getAllLog(allLogs));
    QCOMPARE(6, allLogs.count());
    QCOMPARE("[testAddLog] workinfo a: 123, b: 3.140000", allLogs[5].eventText);
    QCOMPARE("workinfo", allLogs[5].eventType);
}

void TestLog::testAddLogc()
{
    LogManager::logc(LOG_TYPE::LOG_WORKINFO, "[testAddLogc] char workinfo");
    LOG_SAVE();

    QList<LogEntry> allLogs;
    QCOMPARE(0, getAllLog(allLogs));
    QCOMPARE(7, allLogs.count());
    QCOMPARE("[testAddLogc] char workinfo", allLogs[6].eventText);
    QCOMPARE("workinfo", allLogs[6].eventType);

    // 多输入
    log_system("[testAddLogc] system test: %d, %f", 123, 3.14);

    QCOMPARE(0, getAllLog(allLogs));
    QCOMPARE(8, allLogs.count());
    QCOMPARE("[testAddLogc] system test: 123, 3.140000", allLogs[7].eventText);
    QCOMPARE("system", allLogs[7].eventType);
}

void TestLog::testClearLog()
{
    QCOMPARE(0, clearAllLog());
    QList<LogEntry> allLogs;
    QCOMPARE(0, getAllLog(allLogs));
    QCOMPARE(0, allLogs.count());
}
