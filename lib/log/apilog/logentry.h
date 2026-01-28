#ifndef LOGENTRY_H
#define LOGENTRY_H

#include <QString>

struct LogEntry
{
    QString timestamp;
    QString eventType;
    QString eventText;
};

/**
 * @brief getAllLog 获取日志文件中的所有日志结构体数据
 * @param[out] allLogs引用值
 * @return
 *  0: 成功
 * -1: 失败
 */
extern int getAllLog(QList<LogEntry> &allLogs);

/**
 * @brief clearAllLog 清除日志文件内所有日志
 * @return
 *  0: 成功
 * -1: 失败
 */
extern int clearAllLog();

#endif // LOGENTRY_H
