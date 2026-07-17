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

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>    
#include <string.h>   
#include <unistd.h>   
#include <sys/stat.h> 

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

// 日志级别颜色定义
#define LOG_COLOR_RESET   "\033[0m"
#define LOG_COLOR_RED     "\033[31m"    // ERROR
#define LOG_COLOR_YELLOW  "\033[33m"    // WARNING
#define LOG_COLOR_GREEN   "\033[32m"    // INFO
#define LOG_COLOR_CYAN    "\033[36m"    // DEBUG

#define RT_LOG_LEVEL 4  // 设为0关闭所有日志
#define RT_LOG_LEVEL_ERROR 1
#define RT_LOG_LEVEL_WARN  2
#define RT_LOG_LEVEL_DEBUG 3
#define RT_LOG_LEVEL_INFO  4

#define LOG_TO_FILE 1  // 设为1开启文件日志

extern FILE* gnc_log_file;
extern char gnc_log_file_path[256];

char* get_log_path(void);
int get_rt_version(char *version, size_t version_size);
void printMode();

typedef enum{
    INFO = 0,
    DEBUG,
    WARN,
    ERROR,
}LOG_LEVEL_TYPE;

// 初始化和关闭日志文件
#define RT_LOG_INIT() \
    do { \
        char* log_path = get_log_path(); \
        if (log_path != NULL) { \
            gnc_log_file = fopen(log_path, "a"); \
            if (gnc_log_file == NULL) { \
                fprintf(stderr, "无法打开文件： %s\n", log_path); \
            } \
            /* 如果需要保存路径字符串到全局变量 */ \
            strncpy(gnc_log_file_path, log_path, sizeof(gnc_log_file_path) - 1); \
            gnc_log_file_path[sizeof(gnc_log_file_path) - 1] = '\0'; \
            free(log_path); /* 释放 strdup 分配的内存 */ \
        } else { \
            fprintf(stderr, "无法打开log文件\n"); \
        } \
    } while(0)

// 关闭日志文件
#define RT_LOG_CLOSE() \
    do { \
        if (gnc_log_file != NULL) { \
            fclose(gnc_log_file); \
            gnc_log_file = NULL; \
        } \
    } while(0)

// 根据日志级别获取颜色
#define RT_LOG_LEVEL_COLOR(lvl) \
    ((lvl) == ERROR ? LOG_COLOR_RED : \
     (lvl) == WARN  ? LOG_COLOR_YELLOW : \
     (lvl) == DEBUG ? LOG_COLOR_CYAN : LOG_COLOR_GREEN)

#define RT_LOG_LEVEL_STR(lvl) \
    ((lvl) == ERROR ? "ERROR" : \
     (lvl) == WARN  ? "WARN " : \
     (lvl) == DEBUG ? "DEBUG" : "INFO ")

#define RT_LOG(lvl, fmt, ...) \
    do { \
        /* 1. 获取高精度时间 */ \
        struct timeval tv; \
        gettimeofday(&tv, NULL); \
        \
        /* 2. 格式化日期部分 (YYYY-MM-DD HH:MM:SS) */ \
        struct tm* tm_info = localtime(&tv.tv_sec); \
        char date_buf[20]; \
        strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S", tm_info); \
        \
        /* 3. 计算并格式化毫秒部分 (.xxx) */ \
        /* tv.tv_usec 是微秒，除以 1000 得到毫秒 */ \
        char ms_buf[5]; \
        snprintf(ms_buf, sizeof(ms_buf), ".%03ld", tv.tv_usec / 1000); \
        \
        /* 4. 组合完整的时间字符串 */ \
        char full_time[32]; \
        snprintf(full_time, sizeof(full_time), "%s%s", date_buf, ms_buf); \
        \
        /* 5. 输出到控制台 */ \
        printf("%s[%s][%s]%s: " fmt LOG_COLOR_RESET "\n", \
                RT_LOG_LEVEL_COLOR(lvl), \
                full_time, \
                RT_LOG_LEVEL_STR(lvl), \
                __func__, \
                ##__VA_ARGS__); \
        \
        /* 6. 输出到文件 */ \
        if(gnc_log_file) { \
            fprintf(gnc_log_file, "[%s][%s] %s: " fmt "\n", \
                    full_time, \
                    RT_LOG_LEVEL_STR(lvl), \
                    __func__, \
                    ##__VA_ARGS__); \
            fflush(gnc_log_file); \
        } \
    } while(0)

#if RT_LOG_LEVEL >= RT_LOG_LEVEL_ERROR
#   define RT_LOG_ERROR(fmt, ...) \
        printf(LOG_COLOR_RED "[ERROR] %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define RT_LOG_ERROR(fmt, ...)  ((void)0)
#endif

#if RT_LOG_LEVEL >= RT_LOG_LEVEL_WARN
#   define RT_LOG_WARN(fmt, ...) \
        printf(LOG_COLOR_YELLOW "[WARN]  %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define RT_LOG_WARN(fmt, ...)   ((void)0)
#endif

#if RT_LOG_LEVEL >= RT_LOG_LEVEL_INFO
#   define RT_LOG_INFO(fmt, ...) \
        printf(LOG_COLOR_GREEN "[INFO]  %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define RT_LOG_INFO(fmt, ...)   ((void)0)
#endif

#if RT_LOG_LEVEL >= RT_LOG_LEVEL_DEBUG
#   define RT_LOG_DEBUG(fmt, ...) \
        printf(LOG_COLOR_CYAN "[DEBUG] %s: " fmt LOG_COLOR_RESET "\n", __func__, ##__VA_ARGS__)
#else
#   define RT_LOG_DEBUG(fmt, ...)  ((void)0)
#endif


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