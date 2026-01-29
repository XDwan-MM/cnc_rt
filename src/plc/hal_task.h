/*****************************************************************
 * Copyright @ GreeElectric Co.,Ltd.2025-2035.All right reserved
 * File name :     cyclescanning.h
 * Description:    1.简单叙述本文的内容，程序完成的主要功能；
 *                 2.与其他模块或函数的接口；
 *                 3.其他内容的说明
 *
 * ***************************************************************
 *作者：     黄松清
 *日期：     2025.9.15
 *版本：     V2.0
 * ***************************************************************/
#ifndef HAL_TASK_H
#define HAL_TASK_H

#include <stddef.h>
#include "main_demo.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TxEntry_Unit MCPDI[11];
extern RxEntry_Unit MCPDO[7];

void* hal_task();

#ifdef __cplusplus
}
#endif

#endif // HAL_TASK_H
