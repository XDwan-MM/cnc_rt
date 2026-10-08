#ifndef SPL_ORIENT_H
#define SPL_ORIENT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 提交主轴定向请求。成功后定向期间保持驱动器 0x040F。 */
bool spl_orient_request(void);

/* 定向完成状态，完成后保持为 true，直到解除定向。 */
bool spl_orient_is_done(void);

/* 显式解除定向并清除驱动器控制字 bit10；返回驱动器请求是否成功。 */
bool spl_orient_release(void);

/* 急停/Fault 时清除软件定向状态；驱动器物理状态由安全状态机接管。 */
void spl_orient_abort(void);

#ifdef __cplusplus
}
#endif

#endif /* SPL_ORIENT_H */
