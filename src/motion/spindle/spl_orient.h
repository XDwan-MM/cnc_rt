#ifndef SPL_ORIENT_H
#define SPL_ORIENT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 提交主轴定向请求。定向完成前保持驱动器 0x040F。 */
void spl_orient_request(void);

/* 在运动控制周期中更新定向状态。 */
void spl_orient_update(void);

/* 定向完成状态，完成后保持为 true，直到解除定向。 */
bool spl_orient_is_done(void);

/* 显式解除定向并清除驱动器控制字 bit10。 */
void spl_orient_release(void);

#ifdef __cplusplus
}
#endif

#endif /* SPL_ORIENT_H */
