#include "ethercat_driver_hal.h"
#include <stddef.h>

/*=========================================================
 * 读取驱动器反馈
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Read_Feedback(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        printf("drv == NULL\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int ret;

    /* -----------------------------------------------------
     * 6041 从总线反馈PDO读取Statusword，并赋值到抽象驱动器的状态字中
     * ----------------------------------------------------- */

    ret = GM_TxPdoEntry_Read(drv->pdo.statusWord, &(drv->status_word), sizeof(drv->status_word));

    if (ret < 0){
        printf("ret = GM_TxPdoEntry_Read(drv->pdo.statusWord, &(drv->status_word), sizeof(drv->status_word));\n");
        printf("drv->pdo.statusWord.index) %d\n", (drv->pdo.statusWord.index));
        return HAL_DRIVER_ERROR;
    }
    /* -----------------------------------------------------
     * 6064 Position Actual Value（从总线读取）
     * ----------------------------------------------------- */

    ret = GM_TxPdoEntry_Read(drv->pdo.act_pos, &drv->actual_pos, sizeof(drv->actual_pos));

    if (ret < 0){
        printf("ret = GM_TxPdoEntry_Read(drv->pdo.act_pos, &drv->actual_pos, sizeof(drv->actual_pos));\n");
        printf("drv->pdo.act_pos.index) %d\n", (drv->pdo.act_pos.index));
        return HAL_DRIVER_ERROR;
    }
    /* -----------------------------------------------------
     * 606C Velocity Actual Value（从总线读取）
     * ----------------------------------------------------- */

    ret = GM_TxPdoEntry_Read(drv->pdo.act_speed, &drv->actual_vel, sizeof(drv->actual_vel));

    if (ret < 0){
        printf("ret = GM_TxPdoEntry_Read(drv->pdo.act_speed, &drv->actual_vel, sizeof(drv->actual_vel));\n");
        printf("drv->pdo.act_speed.index) %d\n", (drv->pdo.act_speed.index));
        return HAL_DRIVER_ERROR;
    }
    /* -----------------------------------------------------
     * Actual Torque
     * ----------------------------------------------------- */

    // ret = GM_TxPdoEntry_Read(drv->pdo.act_torque, &drv->actual_torque, sizeof(drv->actual_torque));

    // if (ret < 0){
    //     printf("ret = GM_TxPdoEntry_Read(drv->pdo.act_torque, &drv->actual_torque, sizeof(drv->actual_torque));\n");
    //     printf("drv->pdo.act_torque.index) %d\n", (drv->pdo.act_torque.index));
    //     return HAL_DRIVER_ERROR;
    // }

    /* -----------------------------------------------------
     * Error Code（从总线读取）
     * ----------------------------------------------------- */

    // todo 测试平台1需清除以下注释内容
    ret = GM_TxPdoEntry_Read(drv->pdo.error_code, &drv->error_code, sizeof(drv->error_code));

    if (ret < 0){
        printf("ret = GM_TxPdoEntry_Read(drv->pdo.error_code, &drv->error_code, sizeof(drv->error_code));\n");
        printf("drv->pdo.error_code.index) %d\n", (drv->pdo.error_code.index));
        return HAL_DRIVER_ERROR;
    }

    /* -----------------------------------------------------
     * 6061 Modes of Operation Display（从总线读取）
     * ----------------------------------------------------- */

    ret = Ethercat_Driver_Read_Actual_Mode(drv);

    if (ret != HAL_DRIVER_SUCCESS)
    {
        printf("ret != HAL_DRIVER_SUCCESS 68\n");
        return ret;
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 只读取状态字
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Read_Status(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int ret = GM_TxPdoEntry_Read(drv->pdo.statusWord, &drv->status_word, sizeof(drv->status_word));

    if (ret < 0)
    {
        return HAL_DRIVER_ERROR;
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 写控制字 6040
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Write_Controlword(hal_driver_t *drv, uint16_t control_word)
{
    if (drv == NULL)
    {
        printf("Ethercat_Driver_Write_Controlword if (drv == NULL) err\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int ret = GM_RxPdoEntry_Write(drv->pdo.Control_word, control_word);

    if (ret < 0)
    {
        printf("Ethercat_Driver_Write_Controlword GM_RxPdoEntry_Write(drv->pdo.Control_word, (uint32_t)control_word); err\n");
        return HAL_DRIVER_ERROR;
    }

    drv->control_word = control_word;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 写工作模式 6060
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Write_Mode(hal_driver_t *drv, drv_mode_t mode)
{
    if (drv == NULL)
    {
        printf("Ethercat_Driver_Write_Mode if (drv == NULL) err\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int16_t cia_mode;

    switch (mode)
    {
    case DRV_MODE_PP:
        cia_mode = 0x01;
        break;

    case DRV_MODE_PV:
        cia_mode = 0x03;
        break;

    case DRV_MODE_PT:
        cia_mode = 0x04;
        break;

    case DRV_MODE_HOMING:
        cia_mode = 0x06;
        break;

    case DRV_MODE_CSP:
        cia_mode = 0x08;
        break;

    case DRV_MODE_CSV:
        cia_mode = 0x09;
        break;

    case DRV_MODE_CST:
        cia_mode = 0x0A;
        break;

    default:
        printf("驱动器写模式失败 模式传入参数有误\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int ret = GM_RxPdoEntry_Write(drv->pdo.Modes_of_operation, (int16_t)cia_mode);

    if (ret < 0)
    {
        printf("驱动器写模式失败\n");
        return HAL_DRIVER_ERROR;
    }

    drv->req_mode = mode;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 读取实际模式 6061 （从总线读取）
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Read_Actual_Mode(hal_driver_t *drv)
{
    if (drv == NULL)
    {
        printf("mode drv == NULL\n");
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    int16_t mode;

    int ret = GM_TxPdoEntry_Read(drv->pdo.act_mode, &mode, sizeof(mode));

    if (ret < 0)
    {
        printf("drv->pdo.act_mode.index) %d,%d\n", (drv->pdo.act_mode.index),(drv->pdo.act_mode.bit_length));
        printf("mode drv GM_TxPdoEntry_Read %d\n",ret);
        return HAL_DRIVER_ERROR;
    }

    switch (mode)
    {
    case 0x01:
        drv->act_mode = DRV_MODE_PP;
        break;

    case 0x03:
        drv->act_mode = DRV_MODE_PV;
        break;

    case 0x04:
        drv->act_mode = DRV_MODE_PT;
        break;

    case 0x06:
        drv->act_mode = DRV_MODE_HOMING;
        break;

    case 0x08:
        drv->act_mode = DRV_MODE_CSP;
        break;

    case 0x09:
        drv->act_mode = DRV_MODE_CSV;
        break;

    case 0x0A:
        drv->act_mode = DRV_MODE_CST;
        break;

    default:
        drv->act_mode = DRV_MODE_NONE;
        break;
    }

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 写目标位置 607A
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Write_Target_Position(hal_driver_t *drv, int32_t position)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    // todo wcf测试
    // int ret = GM_RxPdoEntry_Write(drv->pdo.target_pos, (uint32_t)position);
    int ret = GM_RxPdoEntry_Write(drv->pdo.target_pos, position);

    if (ret < 0)
    {
        return HAL_DRIVER_ERROR;
    }

    drv->target_pos = position;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 写目标速度 60FF
 *=========================================================*/
hal_retcode_t Ethercat_Driver_Write_Target_Velocity(hal_driver_t *drv, int32_t velocity)
{
    if (drv == NULL)
    {
        return HAL_DRIVER_ERR_INVALID_PARAM;
    }

    // todo wcf测试
    // int ret = GM_RxPdoEntry_Write(drv->pdo.target_speed, (uint32_t)velocity);
    int ret = GM_RxPdoEntry_Write(drv->pdo.target_speed, velocity);

    if (ret < 0)
    {
        return HAL_DRIVER_ERROR;
    }

    drv->target_vel = velocity;

    return HAL_DRIVER_SUCCESS;
}

/*=========================================================
 * 写目标转矩
 *=========================================================*/
// hal_retcode_t Ethercat_Driver_Write_Target_Torque(hal_driver_t *drv, int16_t torque)
// {
//     if (drv == NULL)
//     {
//         return HAL_DRIVER_ERR_INVALID_PARAM;
//     }

//     int ret = GM_RxPdoEntry_Write(drv->pdo.target_torque, (uint32_t)(uint16_t)torque);

//     if (ret < 0)
//     {
//         return HAL_DRIVER_ERROR;
//     }

//     drv->target_torque = torque;

//     return HAL_DRIVER_SUCCESS;
// }