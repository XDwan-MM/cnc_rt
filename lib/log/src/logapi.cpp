#include "logapi.h"
#include "logmanager.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"

void log_system(const char *format, ...)
{
    if (format == NULL)
    {
        return;
    }

    va_list args;
    char *msg = NULL;
    va_start(args, format);
    int msg_len = vsnprintf(NULL, 0, format, args);
    va_end(args);

    if (msg_len > 0) {
        msg = (char *)malloc(msg_len + 1);
        if (msg == NULL) {
            LogManager::logc(4, "[ERROR] 日志内存分配失败\0");
            LogManager::save();
            return;
        }
        va_start(args, format);
        vsnprintf(msg, msg_len + 1, format, args);
        va_end(args);

        LogManager::logc(4, msg);
        LogManager::save();

        free(msg);
        msg = NULL;
    }
}
