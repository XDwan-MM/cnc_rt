#include "logentry.h"
#include <QDebug>
#include <QDir>
#include <QRegularExpression>
#include <QDateTime>
#include <memory>

int getAllLog(QList<LogEntry> &allLogs)
{
    allLogs.clear();
    // 多个日志文件搜寻
    QString logPath = QDir::currentPath() + "/../data/";
    QDir dir(logPath);
    QStringList filters;
    filters << "CNCLOG*.log";
    QStringList logFiles = dir.entryList(filters, QDir::Files);
    if (logFiles.isEmpty()) return -1;

    QVector<QFileInfo> fileInfos;
    for (const QString &fileName : logFiles)
    {
        fileInfos.append(QFileInfo(logPath, fileName));
    }

    // 时间排序
    std::sort(fileInfos.begin(), fileInfos.end(), [](const QFileInfo &a, const QFileInfo &b) {
        return a.lastModified() < b.lastModified();
    });

    auto parseLogFile = [&](const QString &filePath) -> void {
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            return;
        }
        QRegularExpression regex("\\[(.*?)\\] \\[(.*?)\\] (.*)");
        QTextStream in(&file);
        in.setCodec("UTF-8");
        while (!in.atEnd())
        {
            QString line = in.readLine().trimmed();
            if (line.isEmpty()) continue;

            QRegularExpressionMatch match = regex.match(line);
            if (match.hasMatch())
            {
                QString time, type, txt;
                QString timestamp = match.captured(1);

                type = match.captured(2);
                txt = match.captured(3);

                allLogs.append(LogEntry{timestamp, type, txt});
            }
            else
            {
                qDebug() << "无法解析日志行" << line;
            }
        }

        file.close();
    };

    for (const QFileInfo &fileInfo : fileInfos)
    {
        //        qDebug() << "正在解析文件：" << fileInfo.fileName()
        //                 << "(修改时间：" << fileInfo.lastModified().toString("yyyy-MM-dd HH:mm:ss") << ")";
        parseLogFile(fileInfo.absoluteFilePath());
    }
    return 0;
}

int clearAllLog()
{
    QString logPath = QDir::currentPath() + "/../data/";
    QString appName = "CNCLOG.log";
    QFile file(logPath + appName);

    if (!file.exists()) {
        qWarning() << "File does not exist:" << file.fileName();
        return -1;
    }

    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Failed to open file for writing:" << file.fileName();
        return -1;
    }

    file.close();
    return 0;
}
