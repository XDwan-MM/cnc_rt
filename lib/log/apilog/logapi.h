#ifndef LOGAPI_H
#define LOGAPI_H


/**
 * @brief log_system 运控层的事件日志接口
 * @param format
 *
 * @example
 * ...
 * log_ststem("event a: %d, b: %f", 123, 1.23);
 * ...
 * @printto
 * "event a: 123, b: 1.230000"
 */
void log_system(const char *format, ...);


#endif // LOGAPI_H
