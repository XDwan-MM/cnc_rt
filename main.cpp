#include <stdio.h>
#include <cstdio>
#include <cerrno>
#include "hal_driver.h"
#include "hal_io.h"
#include "main_demo.h"
#include "shmapi.h"
#include "rt_sem.h"
#include "threads_api.h"
#include "threads_controller.h"
#include "debug_mode.h"
#include "global.h"
#include "parser.h"
#include <hmisysapi.h>
#include "stored_stroke_check.h"
#include "master.h"
#include <QFile>
#include <QTextStream>
#ifdef DEBUG_MODE_ENABLED
#include "keyboard.h"
#endif

static char* get_version_path();
static int get_hmi_version(char *version, size_t version_size);

// Ctrl + C 终止程序触发
void sigint_handler([[maybe_unused]] int sig) {
    exit_flag = 1;
}

void printMode() {
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
#ifdef DEBUG_MODE_ENABLED
    printf("║         调试模式 (DEBUG MODE)            ║\n");
#else
    printf("║         实时模式 (REALTIME MODE)         ║\n");
    #ifdef PLATFORM_1
    printf("║         平台1 (REALTIME MODE)            ║\n");
    #else
    printf("║         平台2 (REALTIME MODE)            ║\n");
    #endif
#endif
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");
}

int main() {
    // 打印当前模式
    printMode();
    // 打印版本信息
    char version[64];
    get_hmi_version(version, sizeof(version));
    // 初始化共享内存
    if(!shmInit()) {
        printf("[start] 共享内存初始化失败\n");
        return -1;
    } else  {
        errno = 0;
    }
    // 初始化实时全局变量
    if(rt_global_init() != GNC_OPERATOR_SUCCESS){
        printf("[start] 全局变量初始化失败\n");
        return -1;
    };
    // 初始化译码器
    parser_new();
    // 初始化硬件抽象层
    Hal_Driver_Manager_Init();
    Hal_Driver_Array_Init();
    Hal_Io_Array_Init();
    //初始化行程检查功能
    Axis_Limit_Manager_Init();
    signal(SIGINT, sigint_handler);
    // 启动主站
#ifndef DEBUG_MODE_ENABLED
    if (Master_Start() < 0) {
        printf("\n[严重错误] EtherCAT 主站启动失败！程序将终止。\n");
        printf("原因: 可能是 DMA 驱动未加载 (/dev/Gree_DMA0...) 或 XML 路径错误。\n");
        return -1; // 直接退出，不再启动线程
    }
#else // 模拟时不启动主站
    if (init_keyboard() < 0) {
        fprintf(stderr, "键盘初始化失败\n");
        return -1;
    }
#endif
    // 创建信号量，需要放在线程启动（threads_start函数）前
    init_rt_sem();
    // 启动各个线程，pdo线程运行master_run，给各实时线程发送信号量
    if (threads_start()) {
        printf("\n[start] 线程创建失败，主线程退出。\n");
        exit(1);
    }
    while (!exit_flag) {
        sleep(1);
    }
    printf("\n[exit] 收到退出信号，正在关闭线程...\n");
    // 唤醒所有阻塞在信号量上的线程，让它们能够检查 exit_flag 并退出
    for (int i = 0; i < 3; i++) {
        post_rt_sem_all();
    }
    // 释放线程
    threads_joint();
    destroy_rt_sem();
    // 关闭主站
    int error_code = ethercat_close();
    if ( error_code < 0) {
        printf("[Warning] EtherCAT 关闭失败，错误码为 %d ！\n", error_code);
        return -1;
    }
    printf("[exit] 程序正常退出\n");
    return 0;
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

static int get_hmi_version(char *version, size_t version_size) {
    // 1. 打开文件
    const char *file_path = get_version_path();
    FILE *file = fopen(file_path, "r"); 

    if (file == NULL) {
        fprintf(stderr, "无法打开 %s 文件\n", file_path);
        return -1;
    }
    // 2. 读取第一行
    char buffer[256];
    if (fgets(buffer, sizeof(buffer), file) == NULL) {
        fprintf(stderr, "读取文件内容失败或文件为空\n");
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
            fprintf(stderr, "警告：版本号过长，已截断\n");
            strncpy(version, start_ptr, version_size - 1);
            version[version_size - 1] = '\0';
        } else {
            strcpy(version, start_ptr);
        }
        printf("软件版本号：%s\n", version);
        return 0;
    } else {
        // 6. 处理错误情况
        strncpy(version, "(版本号错误)", version_size - 1);
        version[version_size - 1] = '\0';
        printf("软件版本号：%s\n", version);
        return 0; // 虽然内容错误，但函数执行流程完成，返回 0 表示逻辑走完了，或者根据需求返回 -1
    }
}