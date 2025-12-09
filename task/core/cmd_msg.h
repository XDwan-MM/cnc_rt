#ifndef CMD_MSG_H
#define CMD_MSG_H

#include "emc_type.h"
#include "common.h"

typedef struct {
    int serial_number;
    int line;                // 开始运行的行号
    int _type;
    int flag;
    int joint;
    int spindle;
    double scale;
    NC_MODE mode;
    enum EMC_TASK_STATE state;
    const char *file;

} RCS_CMD_MSG;
//命令消息结构体，用于存储命令消息。

#endif // CMD_MSG_H

