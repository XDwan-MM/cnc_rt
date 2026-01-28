#ifndef LOGMANAGER_H
#define LOGMANAGER_H

#include <QString>
#include "spdlog/spdlog.h"

/*!  日志类型 */
enum LOG_TYPE
{
    LOG_WORKINFO = 0,       // 加工信息
    LOG_FILE,               // 文件管理
    LOG_PANELKEY,           // 面板按键
    LOG_DEFINE,             // 自定义
    LOG_SYSTEM,             // 系统事件
    LOG_COUNT,
};

class LogManager
{
public:
    static LogManager &instance();
    static void log(int type, const QString &msg);
    static void logc(int type, char *msg);
    static void save();
    void initialize();

private:
    std::shared_ptr<spdlog::logger> m_logger;

    LogManager();
    ~LogManager();
};

#define LOG_WORKINFO(...) LogManager::log(int(LOG_TYPE::LOG_WORKINFO), __VA_ARGS__)
#define LOG_FILE(...) LogManager::log(int(LOG_TYPE::LOG_FILE), __VA_ARGS__)
#define LOG_PANELKEY(...) LogManager::log(int(LOG_TYPE::LOG_PANELKEY), __VA_ARGS__)
#define LOG_DEFINE(...) LogManager::log(int(LOG_TYPE::LOG_DEFINE), __VA_ARGS__)
#define LOG_SYSTEM(...) LogManager::log(int(LOG_TYPE::LOG_SYSTEM), __VA_ARGS__)
#define LOG_SAVE() LogManager::save()

#define LOG_MULTI(type, ...) \
do { \
    QString msg; \
    msg = QString::asprintf(__VA_ARGS__); \
    LogManager::log(type, msg); \
} while (0)

#endif // LOGMANAGER_H
