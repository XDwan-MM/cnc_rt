
#ifndef TAPRETURN_H
#define TAPRETURN_H

#include <stdbool.h>
#include <stdint.h>
#include <tapreturndata.h>
#include "gncdatadef.h"
#include "plan_queue.h"
#include "sysprog.h"


#ifdef __cplusplus
extern "C"
{
#endif

/*! 刚性攻丝返回记录状态 —— 这就是**相位序列**，没有额外的 valid 位。
 *
 *    IDLE       无记录
 *    RECORDED   准备相位：参数已备、刀**尚未确认进孔**（块被取到 ≠ 刀已进孔）
 *    TAPPING    切削相位：刀**已确认**进入孔内（反馈Z在 R 平面之下），可回退
 *    RETURNING  退出相位：回退段正在执行、刀仍在孔内（可被 RTNT=0 中止；
 *               中止后退回 TAPPING，记录保持可执行 —— 即"可续退"）
 *    DONE       结束相位：刀**已确认**回到 R 平面及以上（记录留作历史数据，不再动作）
 *
 */

void tapreturn_data_restore(void);

/// 丢弃攻丝状态。只在明确丢弃处调用；复位路径严禁调用
void tapreturn_data_reset(void);

// 运行中记录攻丝块参数
bool tapreturn_record(const PLAN_BLOCK *block);

// 按实际运动**推进记录状态；每周期调用（挂 handle_auto_cmd()），
void tapreturn_update(const PLAN_BLOCK *cur_block);

TAP_RETURN_STATE tapreturn_state(void);
bool tapreturn_ready(void);
bool tapreturn_get_param(double *r_plane_z, double *pitch, int *reverse);
GNC_RES tapreturn_generate(char *buf, int buf_len);

void tapreturn_refresh_return_param(void);

extern const SYS_PROG_DESC tapreturn_sysprog_desc;

void rt_set_rigid_return_param(double r_plane_z, double pitch, int reverse, int valid);
void rt_clear_rigid_return_param(void);

GNC_RES tapreturn_generate(char *buf, int buf_len);
GNC_RES tapreturn_generate_with_speed(int speed, char *buf, int buf_len);
bool tapreturn_page_params_match(double pitch_mm, int m_value);


/*! }*/

#ifdef __cplusplus
}
#endif
#endif /* TAPRETURN_H */
