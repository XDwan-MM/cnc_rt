#include "shmapi.h"
#include <cstdio>

int main() {
    // 初始化共享内存
    if(!shmInit()) {
        printf("共享内存初始化失败\n");
    }
    return 0;
}
