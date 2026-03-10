/********************************************************************
 * @file        keyboard.h
 * @brief       键盘事件监控模块
 * @version     V1.00
 * @date        2026/01/30
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    KEY_EV_NONE = 0,
    KEY_EV_EDIT,
    KEY_EV_MEM,
    KEY_EV_MDI,
    KEY_EV_DNC,
    KEY_EV_MPG,
    KEY_EV_JOG,
    KEY_EV_RAPID, 
    KEY_EV_HOME,
    KEY_EV_ESTOP,
    KEY_EV_RESET,
    KEY_EV_RERUN,
    KEY_EV_PAUSE,
    KEY_EV_SBL,
    KEY_EV_DRYRUN,
    KEY_EV_X_POS_PRESS,
    KEY_EV_X_POS_RELEASE,
    KEY_EV_X_NEG_PRESS,
    KEY_EV_X_NEG_RELEASE,
    KEY_EV_Y_POS_PRESS,
    KEY_EV_Y_POS_RELEASE,
    KEY_EV_Y_NEG_PRESS,
    KEY_EV_Y_NEG_RELEASE,
    KEY_EV_Z_POS_PRESS,
    KEY_EV_Z_POS_RELEASE,
    KEY_EV_Z_NEG_PRESS,
    KEY_EV_Z_NEG_RELEASE,
    KEY_EV_FO_INC,
    KEY_EV_FO_DEC,
    KEY_EV_RO_INC,
    KEY_EV_RO_DEC,
    KEY_EV_SO_INC,
    KEY_EV_SO_DEC,
    KEY_EV_SPINDLE_FWD,
    KEY_EV_SPINDLE_REV,
    KEY_EV_SPINDLE_STOP,
}KEY_EVENT;

int init_keyboard(void);

/**
 * @brief 键盘监控主函数
 * 在Linux下监控键盘事件，实现以下功能：
 * - 模式切换：1-7 对应 JOG/快速/手轮/回零/编辑/自动/MDI
 * - 倍率调整：Q/A 进给倍率，W/S 快速倍率，E/D 主轴倍率
 * - 点动控制：I/K(X轴)，O/L(Y轴)，P/;(Z轴) - 仅JOG模式有效
 * - 主轴控制：Z正转，X停止，C反转
 * - 运行控制：Enter运行，Space暂停，R复位，Esc急停退出
 * - 冷却控制：8开启，9关闭
 * @note 此函数会阻塞运行，直到按下Esc键或收到终止信号
 */
void hal_keyboard_minitor(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_H__ */
