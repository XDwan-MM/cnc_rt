#ifndef SPDLOGMANAGER_H
#define SPDLOGMANAGER_H

#include "spdlog/logger.h"

class SpdlogManager
{
public:
    static SpdlogManager &instance();
    void shutdown();

    std::vector<std::shared_ptr<spdlog::logger>> loggers;

private:
    SpdlogManager();
    ~SpdlogManager();
};

#endif // SPDLOGMANAGER_H
