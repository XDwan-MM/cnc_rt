#ifndef MOTION_CSP_STOP_H
#define MOTION_CSP_STOP_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* 仅由运动线程执行的 CSP 停车执行器，由公共复位流程协调。 */
bool motion_csp_stop_begin(void);
bool motion_csp_stop_pending(void);
void motion_csp_stop_update(void);
void motion_csp_stop_cancel(void);
/* true = 本拍的主轴输出已被接管（包含尝试失败的情况），调用方必须跳过普通
 * 主轴位置下发；这不代表成功。false = 没有待处理的 CSP 停车输出，可以考虑普通输出。 */
bool motion_csp_stop_handle_position_output(void);
#ifdef __cplusplus
}
#endif
#endif
