#ifndef BUFSTOP_H
#define BUFSTOP_H


#include "plan_queue.h"

#ifdef __cplusplus
extern "C" {
#endif


unsigned int parser_buf_stop_target_id(void);

bool parser_buf_stop_active(void);      /* 是否记着停止块 */

// 轨迹线程调用，译码器据此放行
void parser_buf_stop_publish_passed(unsigned int id);

// 译码器是否暂时不会再产出新块 
bool parser_hasNoMoreBlocks(void);

#ifdef __cplusplus
}
#endif

#endif /* BUFSTOP_H */
