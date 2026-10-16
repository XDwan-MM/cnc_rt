/**
 * @file ncitpstopdef.h
 * @brief 停止译码 / 阻止缓冲 的共用定义
 *
 *  语义：配了阻止缓冲的 M 代码，控制译码器不预读它后面的程序段，
 *        从而保证执行完这个 M 代码之前，不执行后面的宏语句。
 */

#ifndef NCITPSTOPDEF_H
#define NCITPSTOPDEF_H


/*! 阻止缓冲的 M 代码配置 —— 由各端从参数读出来填好 */
typedef struct {
    int single[10];      /*!< PAR_CH_STOP_BUF_MCODE_1  ~ _10   单值，0 = 未配置 */
    int rangeMin[6];     /*!< PAR_CH_STOP_BUF_MCODE_R1_MIN ~ R6_MIN */
    int rangeMax[6];     /*!< PAR_CH_STOP_BUF_MCODE_R1_MAX ~ R6_MAX */
} ItpBufStopMCodeCfg;

#define ITP_PAR_STOP_BUF_MCODE_1        (507)
#define ITP_PAR_STOP_BUF_MCODE_2        (508)
#define ITP_PAR_STOP_BUF_MCODE_3        (509)
#define ITP_PAR_STOP_BUF_MCODE_4        (510)
#define ITP_PAR_STOP_BUF_MCODE_5        (511)
#define ITP_PAR_STOP_BUF_MCODE_6        (512)
#define ITP_PAR_STOP_BUF_MCODE_7        (513)
#define ITP_PAR_STOP_BUF_MCODE_8        (514)
#define ITP_PAR_STOP_BUF_MCODE_9        (515)
#define ITP_PAR_STOP_BUF_MCODE_10       (516)
#define ITP_PAR_STOP_BUF_MCODE_R1_MIN   (517)
#define ITP_PAR_STOP_BUF_MCODE_R1_MAX   (518)
#define ITP_PAR_STOP_BUF_MCODE_R2_MIN   (519)
#define ITP_PAR_STOP_BUF_MCODE_R2_MAX   (520)
#define ITP_PAR_STOP_BUF_MCODE_R3_MIN   (521)
#define ITP_PAR_STOP_BUF_MCODE_R3_MAX   (522)
#define ITP_PAR_STOP_BUF_MCODE_R4_MIN   (523)
#define ITP_PAR_STOP_BUF_MCODE_R4_MAX   (524)
#define ITP_PAR_STOP_BUF_MCODE_R5_MIN   (525)
#define ITP_PAR_STOP_BUF_MCODE_R5_MAX   (526)
#define ITP_PAR_STOP_BUF_MCODE_R6_MIN   (527)
#define ITP_PAR_STOP_BUF_MCODE_R6_MAX   (528)

/* 无条件阻止缓冲的 M 代码（与参数配置无关） */
#define ITP_BUFSTOP_M00     (0)     /* 程序停止 */
#define ITP_BUFSTOP_M01     (1)     /* 程序选择停止 */
#define ITP_BUFSTOP_M02     (2)     /* 程序结束 */
#define ITP_BUFSTOP_M30     (30)    /* 程序结束并返回开头 */

/* 执行缓冲的 M 代码 —— 与参数设定无关，永远不阻止缓冲
 *
 *  M98/M99、子程序调用用的 M 代码、自定义宏调用用的 M 代码
 *  属于执行缓冲，不能被配成阻止缓冲。
 *      M98 / M99   子程序调用 / 返回
 *      M65 / M66   文件调用
 *      宏程序调用在本系统是 G65（G 码不受影响）
 */
#define ITP_BUFEXEC_M98 (98)
#define ITP_BUFEXEC_M99 (99)
#define ITP_BUFEXEC_M65 (65)
#define ITP_BUFEXEC_M66 (66)

static inline bool itp_isBufExecMCode(int m_code)
{
    return m_code == ITP_BUFEXEC_M98 || m_code == ITP_BUFEXEC_M99
        || m_code == ITP_BUFEXEC_M65 || m_code == ITP_BUFEXEC_M66;
}

/* 判定M 代码是否阻止缓冲停预读*/
static inline bool itp_isBufStopMCode(const ItpBufStopMCodeCfg *cfg, int m_code)
{
    if (m_code < 0)
    {
        return false;
    }

    /* 执行缓冲的 M 代码（M98/M99/M65/M66）：与参数设定无关，先判、直接放行 */
    if (itp_isBufExecMCode(m_code))
    {
        return false;
    }

    /* 无条件名单 */
    if (m_code == ITP_BUFSTOP_M00 || m_code == ITP_BUFSTOP_M01
        || m_code == ITP_BUFSTOP_M02 || m_code == ITP_BUFSTOP_M30)
    {
        return true;
    }

    if (cfg == 0)
    {
        return false;
    }

    /* 参数配的单值（0 表示该条未配置） */
    for (int i = 0; i < 10; ++i)
    {
        if (cfg->single[i] != 0 && cfg->single[i] == m_code)
        {
            return true;
        }
    }

    /* 参数配的 6 组区间（上下限都非 0 才算配置） */
    for (int r = 0; r < 6; ++r)
    {
        if (cfg->rangeMin[r] != 0 && cfg->rangeMax[r] != 0
            && m_code >= cfg->rangeMin[r] && m_code <= cfg->rangeMax[r])
        {
            return true;
        }
    }

    return false;
}

#define ITP_UNMODAL_NONE              (0)
#define ITP_UNMODAL_BACK_TO_REF       (5)    /* G28 返回参考点 */
#define ITP_UNMODAL_MOVE_FROM_REF     (6)    /* G29 从参考点移动 */
#define ITP_UNMODAL_BACK_TO_REF_X     (7)    /* G30 返回指定参考点 */
#define ITP_UNMODAL_STOP_SKIP         (8)    /* G31 跳转（跳段信号） */
#define ITP_UNMODAL_CHILD_COORD_SET   (14)   /* G52 设定局部坐标系 */
#define ITP_UNMODAL_MACHINE_COORD_SET (15)   /* G53 设定机械坐标系 */
#define ITP_UNMODAL_COORD_EXCURSION   (18)   /* G92 工件坐标偏置 */

static inline bool itp_isBufStopGCode(int unmodal)
{
    switch (unmodal)
    {
    case ITP_UNMODAL_STOP_SKIP:         /* G31 */
    case ITP_UNMODAL_MACHINE_COORD_SET: /* G53 */
        return true;
    default:
        return false;
    }
}

#endif /* NCITPSTOPDEF_H */