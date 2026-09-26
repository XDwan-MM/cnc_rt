
#ifndef TAPRETURN_H
#define TAPRETURN_H

#include <stdbool.h>
#include <stdint.h>
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
typedef enum
{
    TAPRET_IDLE = 0,        /* 无记录（不在攻丝中） */
    TAPRET_RECORDED,        /* 准备相位：参数已备、刀尚未确认进孔（非可执行） */
    TAPRET_TAPPING,         /* 切削相位：刀在孔里，可回退 */
    TAPRET_RETURNING,       /* 退出相位：回退段正在执行（可中止，中止后可续退） */
    TAPRET_DONE,            /* 结束相位：刀已回到 R 平面及以上（不再动作） */
} TAP_RETURN_STATE;

typedef struct
{
    double tap_start_z;     // 攻丝起点Z（R点平面，机床坐标）
    double screw_offset;    // 螺距（mm/r）
    int spindle_speed;      // 主轴转速 S（r/min），回退用同转速保持同步比
    int reverse;            // 攻进方向（译码器G33_1包的spindle.reverse）：0=正向攻进 1=逆向攻进
    TAP_RETURN_STATE state; // 记录状态
} TAP_RETURN_DATA;

#define TAPRET_SAVE_MAGIC    0x54525054u    /* "TPRT"：挡住"别的文件被指到这里" */
#define TAPRET_SAVE_VERSION  1u
#define TAPRET_SAVE_DEFAULT  "/tmp/cnc_tapreturn_saved.bin"
#define TAPRET_SAVE_ENV      "CNC_TAPRET_SAVE_FILE"     /* 覆盖块文件路径（台架/测试用） */

typedef struct
{
    uint32_t        magic;
    uint32_t        version;
    uint32_t        size;       /* sizeof(TAP_RETURN_DATA)：结构变了也能挡住旧文件 */
    TAP_RETURN_DATA data;
} TAPRET_SAVE_REC;

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
bool tapreturn_apply_page_speed(void);

extern const SYS_PROG_DESC tapreturn_sysprog_desc;

void rt_set_rigid_return_param(double r_plane_z, double pitch, int reverse, int valid);
void rt_clear_rigid_return_param(void);
/*! }*/

#ifdef __cplusplus
}
#endif
#endif /* TAPRETURN_H */
