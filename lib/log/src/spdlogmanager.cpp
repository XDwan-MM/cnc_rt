#include "spdlogmanager.h"
#include "spdlog/spdlog.h"
#include <QDebug>
#include <QDir>

SpdlogManager &SpdlogManager::instance()
{
    static SpdlogManager instance;
    return instance;
}

void SpdlogManager::shutdown()
{
    static bool shutting = false;
    if (shutting)
    {
        return;
    }
    shutting = true;
    try
    {
        for (auto logger : loggers)
        {
//            qDebug() << "logger:" << QString::fromStdString(logger->name()) << logger.get();
            logger->flush();
            spdlog::drop(logger->name());
            logger.reset();
        }
        spdlog::shutdown();
    } catch (const spdlog::spdlog_ex &ex)
    {
        qCritical() << "Log shutdown error:" << ex.what();
    }
}

SpdlogManager::SpdlogManager()
{

}

SpdlogManager::~SpdlogManager()
{
    shutdown();
}
