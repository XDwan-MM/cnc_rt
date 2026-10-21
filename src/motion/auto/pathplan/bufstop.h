#ifndef BUFSTOP_H
#define BUFSTOP_H

/* 阻止缓冲的跨线程接口。
 *
 * 停止块的记录/等待由译码器（路径规划线程）维护；
 * 轨迹线程的 auto 状态机只通过下面几个函数确认"块出队"和发布放行事件。
 *
 * 不要用 parser.h 代替本文件：parser.h 带 ncitpapi.h，含 C++ 默认参数，
 * C 文件（auto.c / pathplan.c）编不过。 */

#include "plan_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 当前等待的停止块 ID（0=无）。停止块走队列时带着同一个 ID，
 * 轨迹线程据此判断"弹出的就是被等待的那一块"。
 * 低 bit = 1 表示 G53，0 表示 M 码。 */
unsigned int parser_buf_stop_target_id(void);

bool parser_buf_stop_active(void);      /* 是否记着停止块 */

/* 发布"某个停止块已运行完"事件（轨迹线程调用，译码器据此放行）。
 * 必须带 ID，译码器只接受与当前目标 ID 一致的事件。 */
void parser_buf_stop_publish_passed(unsigned int id);

/* 译码器是否暂时不会再产出新块（停在 G31/阻止缓冲上，但不是程序结束）。
 * 流水线收尾逻辑用它把停在队列里的那一块放出去执行。 */
bool parser_hasNoMoreBlocks(void);

#ifdef __cplusplus
}
#endif

#endif /* BUFSTOP_H */
