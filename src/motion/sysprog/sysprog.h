/********************************************************************
 * @file        sysprog.h
 * @brief       系统程序：NC 生成的程序段在 MDI 通道执行（生产/就绪/收口）
 ********************************************************************/

#ifndef SYSPROG_H
#define SYSPROG_H

#include <stdbool.h>
#include "datatype.h"       // GNC_RES（gncdatadef.h 里只有 C 接口那边的错误码）

#ifdef __cplusplus
extern "C"
{
#endif


#define SYSPROG_CONTENT_MAX   (1024)  // 生成内容的缓冲长度


typedef enum
{
    SYS_PROG_NONE = 0,
    SYS_PROG_RIGID_RETURN,      // 刚性攻丝中断回退（M29 S__ G30 P99）
    
} SYS_PROG_ID;

/// 生命周期状态
typedef enum
{
    SYSPROG_INACTIVE = 0,   // 无：内容不可取用，功能未进行 
    SYSPROG_ARMED,          // 已就绪：内容可取用，操作员还未按运行
    SYSPROG_ACTIVE,         // 已运行过：内容已被取走，功能仍在进行（等 OFF/复位收口）
} SYS_PROG_STATE;

/*
 * 回调责任（按调用顺序）：
 *    guard()   前置条件。**只判条件，不要有副作用**（会被反复调用）。
 *    prepare() 把"带外参数"交给译码器（如回退的 R 平面/螺距/转向）。
 *              **一定早于 build() 的内容被下发** —— 顺序由 sysprog_arm() 固定，。
 *    build()   生成程序文本到 buf（**可多行**）。失败则 arm 失败，且 prepare 已回滚。
 *    cleanup() 清掉 prepare() 留下的东西。。
 */
typedef struct
{
    SYS_PROG_ID  id;
    const char  *name;                          // 日志 / 告警 / HMI 显示
    bool       (*guard)(void);
    bool       (*build)(char *buf, int buf_len);
    void       (*prepare)(void);
    void       (*cleanup)(void);
} SYS_PROG_DESC;

/* 启动初始化 */
GNC_RES sysprog_init(void);

bool sysprog_register(const SYS_PROG_DESC *desc);

/*! 备好一个系统程序：guard → prepare → build → 写入内部缓冲,不启动、不碰 MDI 缓冲**。 */
bool sysprog_arm(SYS_PROG_ID id);

/*! 收口：清状态、清缓冲、调 desc->cleanup()。
 *  调用点：功能结束（RIGID_RETURN_OFF）、复位、急停、报警、被别的场景抢占。 */
void sysprog_cleanup(void);

void sysprog_mark_consumed(void);
bool sysprog_content_ready(void);   // 内容是否可被 MDI 取用（ARMED 或 ACTIVE）
bool sysprog_get_content(char *buf, int buf_len);   // 取生成内容。
bool sysprog_is_armed(void);    // 是否"已就绪但还没跑过"

bool sysprog_active(SYS_PROG_ID id);

SYS_PROG_ID sysprog_running_id(void);

SYS_PROG_STATE sysprog_state(void);



#ifdef __cplusplus
}
#endif
#endif /* SYSPROG_H */
