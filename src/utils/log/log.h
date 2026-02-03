/********************************************************************
 * @file        log.h
 * @brief       模拟日志
 * @version     V1.00
 * @date        2025/12/11
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __LOG_H__
#define __LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <time.h>

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

// 日志级别颜色定义
#define LOG_COLOR_RESET   "\033[0m"
#define LOG_COLOR_RED     "\033[31m"    // ERROR
#define LOG_COLOR_YELLOW  "\033[33m"    // WARNING
#define LOG_COLOR_GREEN   "\033[32m"    // INFO
#define LOG_COLOR_CYAN    "\033[36m"    // DEBUG

#define LOG_LEVEL 4  // 设为0关闭所有日志
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

#define LOG_TO_FILE 1  // 设为1开启文件日志
#define LOG_FILE_NAME "logs/gnc.log"

extern FILE* gnc_log_file;
extern char gnc_log_file_path[256];

typedef enum{
    INFO = 0,
    DEBUG,
    WARN,
    ERROR,
}LOG_LEVEL_TYPE;

// 初始化和关闭日志文件
#define LOG_INIT() \
    do { \
        snprintf(gnc_log_file_path, sizeof(gnc_log_file_path), "%s/%s", PROJECT_DIR, LOG_FILE_NAME); \
        gnc_log_file = fopen(gnc_log_file_path, "a"); \
    } while(0)

#define LOG_CLOSE() do { if(gnc_log_file) { fclose(gnc_log_file); gnc_log_file = NULL; } } while(0)

// 根据日志级别获取颜色
#define LOG_LEVEL_COLOR(lvl) \
    ((lvl) == ERROR ? LOG_COLOR_RED : \
     (lvl) == WARN  ? LOG_COLOR_YELLOW : \
     (lvl) == DEBUG ? LOG_COLOR_CYAN : LOG_COLOR_GREEN)

#define LOG_LEVEL_STR(lvl) \
    ((lvl) == ERROR ? "ERROR" : \
     (lvl) == WARN  ? "WARN " : \
     (lvl) == DEBUG ? "DEBUG" : "INFO ")

#if LOG_TO_FILE
    #define LOG_OUTPUT(lvl, fmt, ...) \
        do { \
            time_t t = time(NULL); \
            struct tm* tm_info = localtime(&t); \
            char time_buf[20]; \
            strftime(time_buf, 20, "%Y-%m-%d %H:%M:%S", tm_info); \
            printf("%s[%s][%s] %s: " fmt LOG_COLOR_RESET "\n", LOG_LEVEL_COLOR(lvl), time_buf, LOG_LEVEL_STR(lvl), __func__, ##__VA_ARGS__); \
            if(gnc_log_file) { \
                fprintf(gnc_log_file, "[%s][%s] %s: " fmt "\n", time_buf, LOG_LEVEL_STR(lvl), __func__, ##__VA_ARGS__); \
                fflush(gnc_log_file); \
            } \
        } while(0)
#else
#   define LOG_OUTPUT(lvl, fmt, ...) \
        printf("%s[%s] %s: " fmt LOG_COLOR_RESET "\n", LOG_LEVEL_COLOR(lvl), LOG_LEVEL_STR(lvl), __func__, ##__VA_ARGS__)
#endif


#if LOG_LEVEL >= LOG_LEVEL_ERROR
#   define LOG_ERROR(fmt, ...) \
        printf(LOG_COLOR_RED "[ERROR] %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define LOG_ERROR(fmt, ...)  ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#   define LOG_WARN(fmt, ...) \
        printf(LOG_COLOR_YELLOW "[WARN]  %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define LOG_WARN(fmt, ...)   ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#   define LOG_INFO(fmt, ...) \
        printf(LOG_COLOR_GREEN "[INFO]  %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define LOG_INFO(fmt, ...)   ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#   define LOG_DEBUG(fmt, ...) \
        printf(LOG_COLOR_CYAN "[DEBUG] %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define LOG_DEBUG(fmt, ...)  ((void)0)
#endif

//printf("%s[%s][%s] %s: " fmt LOG_COLOR_RESET "\n", LOG_LEVEL_COLOR(lvl), time_buf, LOG_LEVEL_STR(lvl), __func__, ##__VA_ARGS__); \

// 示例
// void abs_homing_set_state() {
//     LOG_INFO("开始设置回零状态");
//     LOG_DEBUG("当前事件: %d", sys->nc_event);
//     LOG_WARN("换挡模式下不允许操作");
//     LOG_ERROR("回零初始化失败");
// }
// [INFO]  abs_homing_set_state: 开始设置回零状态     (绿色)
// [DEBUG] abs_homing_set_state: 当前事件: 1         (青色)
// [WARN]  abs_homing_set_state: 换挡模式下不允许操作  (黄色)
// [ERROR] abs_homing_set_state: 回零初始化失败       (红色)

#ifdef __cplusplus
}
#endif

#endif
