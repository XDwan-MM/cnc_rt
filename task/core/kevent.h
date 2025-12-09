#ifndef KEVENT_H
#define KEVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmd_msg.h"
#include <stdbool.h>

extern RCS_CMD_MSG cmd;

bool kevent_put(RCS_CMD_MSG * cmd);

int kevent_get(RCS_CMD_MSG * cmd);

int kevent_get_qSize();

void kevent_init();

#ifdef __cplusplus
}
#endif

#endif // KEVENT_H
