#include "log.h"
#define _POSIX_C_SOURCE 200809L

FILE* gnc_log_file = NULL;
char gnc_log_file_path[256];

static char* get_version_path();

char* get_log_path(void) {
    char cwd[1024];
    char path[512] = ""; // 初始化 buffer
    // 1. 获取当前工作目录并构建完整路径
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        char* last_slash = strrchr(cwd, '/');
        if (last_slash != NULL) {
            *last_slash = '\0'; // 截断文件名部分，只保留目录
            snprintf(path, sizeof(path), "%s/%s", cwd, "rt.log");
        } else {
            // 根目录情况 "/"
            snprintf(path, sizeof(path), "/%s", "rt.log");
        }
    } else {
        // getcwd 失败，使用相对路径
        snprintf(path, sizeof(path), "./%s", "rt.log");
    }
    // 2. 检查文件是否存在，如果不存在则创建
    // 方法一：使用 fopen 尝试以追加模式打开（如果文件存在则成功，不存在则创建）
    FILE* fp = fopen(path, "a"); 
    if (fp == NULL) {
        printf("无法打开log文件 %s\n", path);
    } else {
        fclose(fp); 
    }
    // 3. 返回动态分配的路径字符串
    return strdup(path);
}

static char* get_version_path() {
    char cwd[1024];
    char path[512];
    if(getcwd(cwd, sizeof(cwd)) != NULL){
        char* last_slash = strrchr(cwd, '/');
        if(last_slash != NULL){
            *last_slash = '\0';
            snprintf(path, sizeof(path), "%s/%s", cwd, "version.sh");
        }
    }
    return strdup(path); // 无斜杠则返回当前目录
}

int get_rt_version(char *version, size_t version_size) {
    // 1. 打开文件
    const char *file_path = get_version_path();
    FILE *file = fopen(file_path, "r"); 
    if (file == NULL) {
        RT_LOG(ERROR, "无法打开 %s 文件", file_path);
        return -1;
    }
    // 2. 读取第一行
    char buffer[256];
    if (fgets(buffer, sizeof(buffer), file) == NULL) {
        RT_LOG(ERROR, "读取文件内容失败或文件为空");
        fclose(file);
        // 如果文件存在但读不到内容，视情况决定返回什么，这里返回错误
        return -1; 
    }
    fclose(file);
    // 3. 检查行首是否为 "Version="
    // "Version=" 长度为 8
    if (strncmp(buffer, "Version=", 8) == 0) {
        // 4. 提取版本号
        // buffer + 8 指向版本号起始位置
        // 去除可能存在的换行符 (\n)
        char *start_ptr = buffer + 8;
        // 移除末尾的换行符 (如果存在)
        size_t len = strlen(start_ptr);
        if (len > 0 && start_ptr[len-1] == '\n') {
            start_ptr[len-1] = '\0';
            len--;
        }
        // 移除末尾的回车符 (\r)，防止 Windows 格式文件导致的问题
        if (len > 0 && start_ptr[len-1] == '\r') {
            start_ptr[len-1] = '\0';
            len--;
        }
        // 5. 安全拷贝到输出参数
        if (len >= version_size) {
            RT_LOG(ERROR, "版本号过长，已截断");
            strncpy(version, start_ptr, version_size - 1);
            version[version_size - 1] = '\0';
        } else {
            strcpy(version, start_ptr);
        }
        RT_LOG(INFO, "实时版本号 %s", version);
        return 0;
    } else {
        // 6. 处理错误情况
        strncpy(version, "(版本号错误)", version_size - 1);
        version[version_size - 1] = '\0';
        RT_LOG(INFO, "实时版本号 %s", version);
        return 0; 
    }
}

void printMode() {
    RT_LOG(INFO, "══════════════════════════════════════════");
#ifdef DEBUG_MODE_ENABLED
    RT_LOG(INFO, "         调试模式 (DEBUG MODE)              ");
#else
    RT_LOG(INFO, "         实时模式 (REALTIME MODE)           ");
#endif
    RT_LOG(INFO, "══════════════════════════════════════════");
}
