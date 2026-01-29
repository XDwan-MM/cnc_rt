/*****************************************************************
 * Copyright @ GreeElectric Co.,Ltd.2025-2035.All right reserved
 * File name :     cyclescanning.h
 * Description:    1.简单叙述本文的内容，程序完成的主要功能；
 *                 2.与其他模块或函数的接口；
 *                 3.其他内容的说明
 *
 * ***************************************************************
 *作者：     黄松清
 *日期：     2025.9.18
 *版本：     V2.0
 * ***************************************************************/

#ifndef PLC_DATATYPE_H
#define PLC_DATATYPE_H

#include"laddata.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef unsigned long  uint64;
typedef char           int8;
typedef short          int16;
typedef int            int32;
typedef long           int64;

// #define REG_X_NUM 512
// #define REG_Y_NUM 512
// #define REG_I_NUM 128
// #define REG_Q_NUM 128
// #define REG_A_NUM 128
// #define REG_F_NUM 512
// #define REG_G_NUM 512
// #define REG_R_NUM 2024
// #define REG_W_NUM 256
// #define REG_D_NUM 256
#define REG_P_NUM 5000
// #define REG_T_NUM 512
// #define REG_C_NUM 512
// #define REG_B_NUM 512
// #define REG_L_NUM 5000
// #define REG_S_NUM 5000


extern int16 WW[REG_W_NUM];
extern int32 PP[REG_P_NUM],BB[REG_B_NUM],DD[REG_D_NUM];
extern uint8  XX[REG_X_NUM],YY[REG_Y_NUM],II[REG_I_NUM],QQ[REG_Q_NUM],
             RR[REG_R_NUM],FF[REG_F_NUM],GG[REG_G_NUM]/*,AA[REG_A_NUM]*/;

//****************操作数处理******************
#define _gnc_get_x_num() (REG_X_NUM)
#define _gnc_get_y_num() (REG_Y_NUM)
#define _gnc_get_f_num() (REG_F_NUM)
#define _gnc_get_g_num() (REG_G_NUM)
#define _gnc_get_r_num() (REG_R_NUM)
#define _gnc_get_w_num() (REG_W_NUM)
#define _gnc_get_d_num() (REG_D_NUM)
#define _gnc_get_p_num() (REG_P_NUM)
#define _gnc_get_t_num() (REG_T_NUM)
#define _gnc_get_c_num() (REG_C_NUM)
#define _gnc_get_b_num() (REG_B_NUM)
//#define _gnc_get_l_num() (REG_L_NUM)
#define _gnc_get_s_num() (REG_S_NUM)


#define _gnc_get_x_bit(i,b) (XX[i] & (1<<b))
#define _gnc_set_x_bit(i,b) (XX[i] |=(1<<b))
#define _gnc_clr_x_bit(i,b) (XX[i] &=(~(1<<b)))

#define _gnc_get_y_bit(i,b) (YY[i] & (1<<b))
#define _gnc_set_y_bit(i,b) (YY[i] |=(1<<b))
#define _gnc_clr_y_bit(i,b) (YY[i] &=(~(1<<b)))

#define _gnc_get_f_bit(i,b) (FF[i] & (1<<b))
#define _gnc_set_f_bit(i,b) (FF[i] |=(1<<b))
#define _gnc_clr_f_bit(i,b) (FF[i] &=(~(1<<b)))

#define _gnc_get_g_bit(i,b) (GG[i] & (1<<b))
#define _gnc_set_g_bit(i,b) (GG[i] |=(1<<b))
#define _gnc_clr_g_bit(i,b) (GG[i] &=(~(1<<b)))

#define _gnc_get_r_bit(i,b) (RR[i] & (1<<b))
#define _gnc_set_r_bit(i,b) (RR[i] |=(1<<b))
#define _gnc_clr_r_bit(i,b) (RR[i] &=(~(1<<b)))

#define _gnc_get_w_bit(i,b) (WW[i] & (1<<b))
#define _gnc_set_w_bit(i,b) (WW[i] |=(1<<b))
#define _gnc_clr_w_bit(i,b) (WW[i] &=(~(1<<b)))

#define _gnc_get_d_bit(i,b) (DD[i] & (1<<b))
#define _gnc_set_d_bit(i,b) (DD[i] |=(1<<b))
#define _gnc_clr_d_bit(i,b) (DD[i] &=(~(1<<b)))

#define _gnc_get_p_bit(i,b) (PP[i] & (1<<b))
#define _gnc_set_p_bit(i,b) (PP[i] |=(1<<b))
#define _gnc_clr_p_bit(i,b) (PP[i] &=(~(1<<b)))

#define _gnc_get_t_bit(i,b) (TT[i] & (1<<b))
#define _gnc_set_t_bit(i,b) (TT[i] |=(1<<b))
#define _gnc_clr_t_bit(i,b) (TT[i] &=(~(1<<b)))

#define _gnc_get_c_bit(i,b) (CC[i] & (1<<b))
#define _gnc_set_c_bit(i,b) (CC[i] |=(1<<b))
#define _gnc_clr_c_bit(i,b) (CC[i] &=(~(1<<b)))

#define _gnc_get_b_bit(i,b) (BB[i] & (1<<b))
#define _gnc_set_b_bit(i,b) (BB[i] |=(1<<b))
#define _gnc_clr_b_bit(i,b) (BB[i] &=(~(1<<b)))

#define _gnc_get_l_bit(i,b) (LL[i] & (1<<b))
#define _gnc_set_l_bit(i,b) (LL[i] |=(1<<b))
#define _gnc_clr_l_bit(i,b) (LL[i] &=(~(1<<b)))

#define _gnc_get_s_bit(i,b) (SS[i] & (1<<b))
#define _gnc_set_s_bit(i,b) (SS[i] |=(1<<b))
#define _gnc_clr_s_bit(i,b) (SS[i] &=(~(1<<b)))


//***************标记操作*****************
#define _gnc_get_x_flag(i,b) (XX[i] & b)
#define _gnc_set_x_flag(i,b) (XX[i] |=b)
#define _gnc_clr_x_flag(i,b) (XX[i] &=(~b))

#define _gnc_get_y_flag(i,b) (YY[i] & b)
#define _gnc_set_y_flag(i,b) (YY[i] |=b)
#define _gnc_clr_y_flag(i,b) (YY[i] &=(~b))

#define _gnc_get_f_flag(i,b) (FF[i] & b)
#define _gnc_set_f_flag(i,b) (FF[i] |=b)
#define _gnc_clr_f_flag(i,b) (FF[i] &=(~b))

#define _gnc_get_g_flag(i,b) (GG[i] & b)
#define _gnc_set_g_flag(i,b) (GG[i] |=b)
#define _gnc_clr_g_flag(i,b) (GG[i] &=(~b))

#define _gnc_get_r_flag(i,b) (RR[i] & b)
#define _gnc_set_r_flag(i,b) (RR[i] |=b)
#define _gnc_clr_r_flag(i,b) (RR[i] &=(~b))

#define _gnc_get_w_flag(i,b) (WW[i] & b)
#define _gnc_set_w_flag(i,b) (WW[i] |=b)
#define _gnc_clr_w_flag(i,b) (WW[i] &=(~b))

#define _gnc_get_d_flag(i,b) (DD[i] & b)
#define _gnc_set_d_flag(i,b) (DD[i] |=b)
#define _gnc_clr_d_flag(i,b) (DD[i] &=(~b))

#define _gnc_get_p_flag(i,b) (PP[i] & b)
#define _gnc_set_p_flag(i,b) (PP[i] |=b)
#define _gnc_clr_p_flag(i,b) (PP[i] &=(~b))

#define _gnc_get_b_flag(i,b) (BB[i] & b)
#define _gnc_set_b_flag(i,b) (BB[i] |=b)
#define _gnc_clr_b_flag(i,b) (BB[i] &=(~b))


//**************值操作******************
#define _gnc_get_x_value(i)   (XX[i])
#define _gnc_set_x_value(i,b) (XX[i]=b)

#define _gnc_get_y_value(i)   (YY[i])
#define _gnc_set_y_value(i,b) (YY[i]=b)

#define _gnc_get_f_value(i)   (FF[i])
#define _gnc_set_f_value(i,b) (FF[i]=b)

#define _gnc_get_g_value(i)   (GG[i])
#define _gnc_set_g_value(i,b) (GG[i]=b)

#define _gnc_get_r_value(i)   (RR[i])
#define _gnc_set_r_value(i,b) (RR[i]=b)

#define _gnc_get_w_value(i)   (WW[i])
#define _gnc_set_w_value(i,b) (WW[i]=b)

#define _gnc_get_d_value(i)   (DD[i])
#define _gnc_set_d_value(i,b) (DD[i]=b)

#define _gnc_get_p_value(i)   (PP[i])
#define _gnc_set_p_value(i,b) (PP[i]=b)

#define _gnc_get_b_value(i)   (BB[i])
#define _gnc_set_b_value(i,b) (BB[i]=b)

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif

#endif // PLC_DATATYPE_H
