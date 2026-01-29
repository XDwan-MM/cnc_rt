#include "logmanager.h"
#include <QDir>
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlogmanager.h"

#define LOG_TYPE_WORKINFO std::string("workinfo")
#define LOG_TYPE_FILE std::string("file")
#define LOG_TYPE_PANELKEY std::string("panelkey")
#define LOG_TYPE_DEFINE std::string("define")
#define LOG_TYPE_SYSTEM std::string("system")

#if !defined(LOG_TYPE_NAMES)
    #define LOG_TYPE_NAMES                                      \
    {                                                           \
        LOG_TYPE_WORKINFO, LOG_TYPE_FILE, LOG_TYPE_PANELKEY,    \
        LOG_TYPE_DEFINE, LOG_TYPE_SYSTEM                        \
    }
#endif

static std::string logType[] LOG_TYPE_NAMES;

LogManager &LogManager::instance()
{
    static LogManager instance;
    return instance;
}

void LogManager::log(int type, const QString &msg)
{
    if (instance().m_logger)
    {
        instance().m_logger->log(spdlog::level::info, "[{}] {}", logType[type], msg.toStdString());
    }
}

void LogManager::logc(int type, char *msg)
{
    if (instance().m_logger)
    {
        instance().m_logger->log(spdlog::level::info, "[{}] {}", logType[type], std::string(msg));
    }
}

void LogManager::save()
{
    if (instance().m_logger)
    {
        instance().m_logger->flush();
    }
}

void LogManager::initialize()
{
    QString logPath = QDir::currentPath() + "/../data/";
    QString appName = "CNCLOG.log";
    if (m_logger)
    {
        return;
    }
    QDir().mkpath(logPath);
    std::string fileName = QDir(logPath).absoluteFilePath(appName).toStdString();

//    m_logger = createCustomLogger(fileName, max_size, max_files);
//    m_logger->set_level(spdlog::level::trace);
//    spdlog::register_logger(m_logger);

    m_logger = spdlog::basic_logger_mt("cnc_log", fileName);
    m_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] %v");
    spdlog::set_level(spdlog::level::trace);

    spdlog::set_default_logger(m_logger);

    SpdlogManager::instance().loggers.push_back(m_logger);
}

LogManager::LogManager()
{
    initialize();
}

LogManager::~LogManager()
{
}
