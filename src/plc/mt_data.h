/*****************************************************************
 * Copyright @ GreeElectric Co.,Ltd.2025-2035.All right reserved
 * File name :     cyclescanning.h
 * Description:    1.简单叙述本文的内容，程序完成的主要功能；
 *                 2.与其他模块或函数的接口；
 *                 3.其他内容的说明
 *
 * ***************************************************************
 *作者：     黄松清
 *日期：     2025.9.28
 *版本：     V2.0
 * ***************************************************************/
#ifndef MT_DATA_H
#define MT_DATA_H

#include "plcdatatype.h"

//****************机床侧输入数据**************
/* 操作面板上面的按键原始数据 连接现场总线 物理上对应*/
struct kb_in1_origin
{
     uint16 X00 : 1;
     uint16 X01 : 1;
     uint16 X02 : 1;
     uint16 X03 : 1;
     uint16 X04 : 1;
     uint16 X05 : 1;
     uint16 X06 : 1;
     uint16 X07 : 1;
     uint16 X10 : 1;
     uint16 X11 : 1;
     uint16 X12 : 1;
     uint16 X13 : 1;
     uint16 X14 : 1;
     uint16 X15 : 1;
     uint16 X16 : 1;
     uint16 X17 : 1;
};

/* 操作面板上面的按键原始数据 连接现场总线 物理上对应*/
struct kb_in2_origin
{
    uint16 X20 : 1;
    uint16 X21 : 1;
    uint16 X22 : 1;
    uint16 X23 : 1;
    uint16 X24 : 1;
    uint16 X25 : 1;
    uint16 X26 : 1;
    uint16 X27 : 1;
    uint16 X30 : 1;
    uint16 X31 : 1;
    uint16 X32 : 1;
    uint16 X33 : 1;
    uint16 X34 : 1;
    uint16 X35 : 1;
    uint16 X36 : 1;
    uint16 X37 : 1;
};

/* 操作面板上面的按键原始数据 连接现场总线 物理上对应*/
struct kb_in3_origin
{
    uint16 X40 : 1;
    uint16 X41 : 1;
    uint16 X42 : 1;
    uint16 X43 : 1;
    uint16 X44 : 1;
    uint16 X45 : 1;
    uint16 X46 : 1;
    uint16 X47 : 1;
    uint16 X50 : 1;
    uint16 X51 : 1;
    uint16 X52 : 1;
    uint16 X53 : 1;
    uint16 X54 : 1;
    uint16 X55 : 1;
    uint16 X56 : 1;
    uint16 X57 : 1;
};

/* 操作面板上面的按键原始数据 连接现场总线 物理上对应*/
struct kb_in4_origin
{
    uint16 X60 : 1;
    uint16 X61 : 1;
    uint16 X62 : 1;
    uint16 X63 : 1;
    uint16 X64 : 1;
    uint16 X65 : 1;
    uint16 X66 : 1;
    uint16 X67 : 1;
    uint16 X70 : 1;
    uint16 X71 : 1;
    uint16 X72 : 1;
    uint16 X73 : 1;
    uint16 X74 : 1;
    uint16 X75 : 1;
    uint16 X76 : 1;
    uint16 X77 : 1;
};

struct kb_in5_origin
{
    uint16 X80 : 1;
    uint16 X81 : 1;
    uint16 X82 : 1;
    uint16 X83 : 1;
    uint16 X84 : 1;
    uint16 X85 : 1;
    uint16 X86 : 1;
    uint16 X87 : 1;
    uint16 X90 : 1;
    uint16 X91 : 1;
    uint16 X92 : 1;
    uint16 X93 : 1;
    uint16 X94 : 1;
    uint16 X95 : 1;
    uint16 X96 : 1;
    uint16 X97 : 1;
};

struct kb_in6_origin
{
    uint16 X100 : 1;
    uint16 X101 : 1;
    uint16 X102 : 1;
    uint16 X103 : 1;
    uint16 X104 : 1;
    uint16 X105 : 1;
    uint16 X106 : 1;
    uint16 X107 : 1;
    uint16 X110 : 1;
    uint16 X111 : 1;
    uint16 X112 : 1;
    uint16 X113 : 1;
    uint16 X114 : 1;
    uint16 X115 : 1;
    uint16 X116 : 1;
    uint16 X117 : 1;
};

struct kb_in7_origin
{
    uint16 X120 : 1;
    uint16 X121 : 1;
    uint16 X122 : 1;
    uint16 X123 : 1;
    uint16 X124 : 1;
    uint16 X125 : 1;
    uint16 X126 : 1;
    uint16 X127 : 1;
    uint16 X130 : 1;
    uint16 X131 : 1;
    uint16 X132 : 1;
    uint16 X133 : 1;
    uint16 X134 : 1;
    uint16 X135 : 1;
    uint16 X136 : 1;
    uint16 X137 : 1;
};

struct kb_in8_origin
{
    uint16 X140 : 1;
    uint16 X141 : 1;
    uint16 X142 : 1;
    uint16 X143 : 1;
    uint16 X144 : 1;
    uint16 X145 : 1;
    uint16 X146 : 1;
    uint16 X147 : 1;
    uint16 X150 : 1;
    uint16 X151 : 1;
    uint16 X152 : 1;
    uint16 X153 : 1;
    uint16 X154 : 1;
    uint16 X155 : 1;
    uint16 X156 : 1;
    uint16 X157 : 1;
};

struct kb_in9_origin
{
    uint16 X160 : 1;
    uint16 X161 : 1;
    uint16 X162 : 1;
    uint16 X163 : 1;
    uint16 X164 : 1;
    uint16 X165 : 1;
    uint16 X166 : 1;
    uint16 X167 : 1;
    uint16 X170 : 1;
    uint16 X171 : 1;
    uint16 X172 : 1;
    uint16 X173 : 1;
    uint16 X174 : 1;
    uint16 X175 : 1;
    uint16 X176 : 1;
    uint16 X177 : 1;
};


/* 操作面板上面的按键原始数据 手轮 连接现场总线*/
struct kb_handwheel_origin
{
    uint16 rate_1:1;       //档位1
    uint16 rate_10:1;      //档位10
    uint16 rate_100:1;     //档位100
    uint16 axis_x:1;       //选择轴1
    uint16 axis_y:1;       //选择轴2
    uint16 axis_z:1;       //选择轴3
    uint16 axis_4:1;       //选择轴4
    uint16 axis_5:1;       //选择轴5
    uint16 remain9:1;
    uint16 remain10:1;
    uint16 remain11:1;
    uint16 remain12:1;
    uint16 remain13:1;
    uint16 remain14:1;
    uint16 remain15:1;
    uint16 remain16:1;
};

union kb_in1_oreg
{
    uint16 all;
    struct kb_in1_origin bit;
};

union kb_in2_oreg
{
    uint16 all;
    struct kb_in2_origin bit;
};

union kb_in3_oreg
{
    uint16 all;
    struct kb_in3_origin bit;
};

union kb_in4_oreg
{
    uint16 all;
    struct kb_in4_origin bit;
};

union kb_in5_oreg
{
    uint16 all;
    struct kb_in5_origin bit;
};

union kb_in6_oreg
{
    uint16 all;
    struct kb_in6_origin bit;
};

union kb_in7_oreg
{
    uint16 all;
    struct kb_in7_origin bit;
};

union kb_in8_oreg
{
    uint16 all;
    struct kb_in8_origin bit;
};

union kb_inb_oreg
{
    uint16 all;
    struct kb_handwheel_origin bit;
};

union kb_in9_oreg
{
    uint16 all;
    struct kb_in9_origin bit;
};



/* 操作面板上面的软PLC信号给到运动控制 按键*/
struct kb_in_plcout
{
    uint32 plus_x : 1;           //X轴正向  为1时运动
    uint32 sub_x : 1;            //X轴负向
    uint32 plus_y : 1;           //Y轴正向
    uint32 sub_y : 1;            //Y轴负向
    uint32 plus_z : 1;           //Z轴正向
    uint32 sub_z : 1;            //Z轴负向
    uint32 plus_4 : 1;           //4轴正向
    uint32 sub_4 : 1;            //4轴负向
    uint32 plus_5 : 1;           //5轴正向
    uint32 sub_5 : 1;            //5轴负向
    uint32 Single_run : 1;       //单段运行
    uint32 Select_Stop : 1;      //选择停止  与M01配合使用
    uint32 empty_run : 1;        //程序空跑  不加工，快速运行
    uint32 single_ignore : 1;    //单节忽略  与释码器配合使用
    uint32 auto_poweroff : 1;    //程序执行完后，自动断电
    uint32 handwheel_back : 1;   //手轮回退
    uint32 handwheel_stop : 1;   //手轮中断，停下来
    uint32 sp_cw : 1;            //主轴正转
    uint32 sp_ccw : 1;           //主轴反转
    uint32 sp_stop : 1;          //主轴停止
    uint32 sp_orient : 1;        //主轴定向
    uint32 Tap_rollback : 1;     //攻丝回退
    uint32 cycle_start : 1;      //程序启动
    uint32 cycle_stop : 1;       //程序启动
    uint32 remain : 8;           //预留
};

typedef struct
{
    union kb_in1_oreg keyboardIn1;
    union kb_in2_oreg keyboardIn2;
    union kb_in3_oreg keyboardIn3;
    union kb_in4_oreg keyboardIn4;
    union kb_in5_oreg keyboardIn5;
    union kb_in6_oreg keyboardIn6;
    union kb_in7_oreg keyboardIn7;
    union kb_in8_oreg keyboardIn8;
    union kb_inb_oreg keyboardInb;
    union kb_in9_oreg keyboardIn9;
    uint16 handcnt_l;                 //手轮计数器低位
    uint16 handcnt_h;                 //手轮计数器高位
    uint16 mode;                      //操作模式选择
    uint16 feedrate;                  //进给挡位
    uint16 rapid;                     //快速挡位
    uint16 spindle;                   //主轴转速挡位
    uint8  OnOFF;                     //确定是否有接手轮
    uint8  AxisOverride;              //手轮选择的轴号旋钮
}pd_PlcData;

/* 通用输入IO口， 连接现场总线*/
struct IO_IN1_BITS    // bits   description
{
     uint32 bit0  : 1;
     uint32 bit1  : 1;
     uint32 bit2  : 1;
     uint32 bit3  : 1;
     uint32 bit4  : 1;
     uint32 bit5  : 1;
     uint32 bit6  : 1;
     uint32 bit7  : 1;
     uint32 bit8  : 1;
     uint32 bit9  : 1;
     uint32 bit10 : 1;
     uint32 bit11 : 1;
     uint32 bit12 : 1;
     uint32 bit13 : 1;
     uint32 bit14 : 1;
     uint32 bit15 : 1;
     uint32 bit16 : 1;
     uint32 bit17 : 1;
     uint32 bit18 : 1;
     uint32 bit19 : 1;
     uint32 bit20 : 1;
     uint32 bit21 : 1;
     uint32 bit22 : 1;
     uint32 bit23 : 1;
     uint32 bit24 : 1;
     uint32 bit25 : 1;
     uint32 bit26 : 1;
     uint32 bit27 : 1;
     uint32 bit28 : 1;
     uint32 bit29 : 1;
     uint32 bit30 : 1;
     uint32 bit31 : 1;
};

union  IO_IN1_REG
{
    uint32             all;
    struct IO_IN1_BITS bit;
};

/* 操作面板上面的软PLC信号给到运动控制  手轮*/
struct kb_handwheel_plcout
{
    uint16 rate_1:1;
    uint16 rate_10:1;
    uint16 rate_100:1;
    uint16 axis_x:1;
    uint16 axis_y:1;
    uint16 axis_z:1;
    uint16 axis_4:1;
    uint16 axis_5:1;
    uint16 remain9:1;
    uint16 remain10:1;
    uint16 remain11:1;
    uint16 remain12:1;
    uint16 remain13:1;
    uint16 remain14:1;
    uint16 remain15:1;
    uint16 remain16:1;
};

/*操作面板  软PLC到 运动控制*/
union kb_plc_Motion_ctrl
{
    uint32 all;
    struct kb_in_plcout bit;
};

/*操作面板  软PLC到 运动控制 手轮*/
union kb_plcout_handwheel
{
    uint16 all;
    struct kb_handwheel_plcout bit;
};

typedef struct
{
    union kb_plc_Motion_ctrl  keyboardOut;          //操作面板的按键信号  软PLC反馈给运动控制
    union kb_plcout_handwheel keyboardOutb;         //操作面板的手轮信号  软PLC反馈给运动控制
    uint16 handcnt_l;
    uint16 handcnt_h;
    uint16 mode;
    uint16 feedrate;
    uint16 rapid;
    uint16 spindle;
    uint8  OnOFF;                       //确定是否有接手轮
    uint8  AxisOverride;                //手轮选择的轴号旋钮
}pd_MotionData;

struct kb_out1_led
{
    uint16 X00 : 1;   //bit X0.0
    uint16 X01 : 1;   //bit X0.1
    uint16 X02 : 1;   //X0.2
    uint16 X03 : 1;   //X0.3
    uint16 X04 : 1;   //X0.4
    uint16 X05 : 1;   //X0.5
    uint16 X06 : 1;   //X0.6
    uint16 X07 : 1;   //X0.7
    uint16 X10 : 1;   //X1.0
    uint16 X11 : 1;   //X1.1
    uint16 X12 : 1;   //X1.2
    uint16 X13 : 1;   //X1.3
    uint16 X14 : 1;   //X1.4
    uint16 X15 : 1;   //X1.5
    uint16 X16 : 1;   //X1.6
    uint16 X17 : 1;   //X1.7
};

/*****操作面板上面的led灯输出数据****/
struct kb_out2_led
{
    uint16 X20 : 1;   //bit X2.0
    uint16 X21 : 1;   //bit X2.1
    uint16 X22 : 1;   //X2.2
    uint16 X23 : 1;   //X2.3
    uint16 X24 : 1;   //X2.4
    uint16 X25 : 1;   //X2.5
    uint16 X26 : 1;   //X2.6
    uint16 X27 : 1;   //X2.7
    uint16 X30 : 1;   //X3.0
    uint16 X31 : 1;   //X3.1
    uint16 X32 : 1;   //X3.2
    uint16 X33 : 1;   //X3.3
    uint16 X34 : 1;   //X3.4
    uint16 X35 : 1;   //X3.5
    uint16 X36 : 1;   //X3.6
    uint16 X37 : 1;   //X3.7
};

/* 操作面板上面的led灯输出数据*/
struct kb_out3_led
{
    uint16 X40 : 1;   //bit X4.0
    uint16 X41 : 1;   //bit X4.1
    uint16 X42 : 1;   //X4.2
    uint16 X43 : 1;   //X4.3
    uint16 X44 : 1;
    uint16 X45 : 1;
    uint16 X46 : 1;
    uint16 X47 : 1;
    uint16 X50 : 1;
    uint16 X51 : 1;
    uint16 X52 : 1;
    uint16 X53 : 1;
    uint16 X54 : 1;
    uint16 X55 : 1;
    uint16 X56 : 1;
    uint16 X57 : 1;
};

/* 操作面板上面的led灯输出数据*/
struct kb_out4_led
{
    uint16 X60 : 1;
    uint16 X61 : 1;
    uint16 X62 : 1;
    uint16 X63 : 1;
    uint16 X64 : 1;
    uint16 X65 : 1;
    uint16 X66 : 1;
    uint16 X67 : 1;
    uint16 X70 : 1;
    uint16 X71 : 1;
    uint16 X72 : 1;
    uint16 X73 : 1;
    uint16 X74 : 1;
    uint16 X75 : 1;
    uint16 X76 : 1;
    uint16 X77 : 1;
};


struct kb_out5_led
{
    uint16 X80 : 1;
    uint16 X81 : 1;
    uint16 X82 : 1;
    uint16 X83 : 1;
    uint16 X84 : 1;
    uint16 X85 : 1;
    uint16 X86 : 1;
    uint16 X87 : 1;
    uint16 X90 : 1;
    uint16 X91 : 1;
    uint16 X92 : 1;
    uint16 X93 : 1;
    uint16 X94 : 1;
    uint16 X95 : 1;
    uint16 X96 : 1;
    uint16 X97 : 1;
};

struct kb_out6_led
{
    uint16 X100 : 1;
    uint16 X101 : 1;
    uint16 X102 : 1;
    uint16 X103 : 1;
    uint16 X104 : 1;
    uint16 X105 : 1;
    uint16 X106 : 1;
    uint16 X107 : 1;
    uint16 X110 : 1;
    uint16 X111 : 1;
    uint16 X112 : 1;
    uint16 X113 : 1;
    uint16 X114 : 1;
    uint16 X115 : 1;
    uint16 X116 : 1;
    uint16 X117 : 1;
};

struct kb_out7_led
{
    uint16 X120 : 1;
    uint16 X121 : 1;
    uint16 X122 : 1;
    uint16 X123 : 1;
    uint16 X124 : 1;
    uint16 X125 : 1;
    uint16 X126 : 1;
    uint16 X127 : 1;
    uint16 X130 : 1;
    uint16 X131 : 1;
    uint16 X132 : 1;
    uint16 X133 : 1;
    uint16 X134 : 1;
    uint16 X135 : 1;
    uint16 X136 : 1;
    uint16 X137 : 1;
};

struct kb_out8_led
{
    uint16 X140 : 1;
    uint16 X141 : 1;
    uint16 X142 : 1;
    uint16 X143 : 1;
    uint16 X144 : 1;
    uint16 X145 : 1;
    uint16 X146 : 1;
    uint16 X147 : 1;
    uint16 X150 : 1;
    uint16 X151 : 1;
    uint16 X152 : 1;
    uint16 X153 : 1;
    uint16 X154 : 1;
    uint16 X155 : 1;
    uint16 X156 : 1;
    uint16 X157 : 1;
};

union  kb_led1_reg
{
    uint16 all;
    struct kb_out1_led bit;
};

union  kb_led2_reg
{
    uint16 all;
    struct kb_out2_led bit;
};

union  kb_led3_reg
{
    uint16 all;
    struct kb_out3_led bit;
};

union  kb_led4_reg
{
    uint16 all;
    struct kb_out4_led bit;
};

union  kb_led5_reg
{
    uint16 all;
    struct kb_out5_led bit;
};

union  kb_led6_reg
{
    uint16 all;
    struct kb_out6_led bit;
};

union  kb_led7_reg
{
    uint16 all;
    struct kb_out7_led bit;
};

union  kb_led8_reg
{
    uint16 all;
    struct kb_out8_led bit;
};


typedef struct
{
    union kb_led1_reg keyboardOut1;
    union kb_led2_reg keyboardOut2;
    union kb_led3_reg keyboardOut3;
    union kb_led4_reg keyboardOut4;
    union kb_led5_reg keyboardOut5;
    union kb_led6_reg keyboardOut6;
    union kb_led7_reg keyboardOut7;
    union kb_led8_reg keyboardOut8;

    uint8 tube1;
    uint8 tube2;
    uint8 tube3;
    uint8 tube4;
    uint8 tube5;
    uint8 tube6;
}pd_output;

/******通用输出IO口， 连接现场总线*******/
struct IO_OUT1_BITS   // bits   description
{
    uint32 bit0  : 1;
    uint32 bit1  : 1;
    uint32 bit2  : 1;
    uint32 bit3  : 1;
    uint32 bit4  : 1;
    uint32 bit5  : 1;
    uint32 bit6  : 1;
    uint32 bit7  : 1;
    uint32 bit8  : 1;
    uint32 bit9  : 1;
    uint32 bit10 : 1;
    uint32 bit11 : 1;
    uint32 bit12 : 1;
    uint32 bit13 : 1;
    uint32 bit14 : 1;
    uint32 bit15 : 1;
    uint32 bit16 : 1;
    uint32 bit17 : 1;
    uint32 bit18 : 1;
    uint32 bit19 : 1;
    uint32 bit20 : 1;
    uint32 bit21 : 1;
    uint32 bit22 : 1;
    uint32 bit23 : 1;
    uint32 bit24 : 1;
    uint32 bit25 : 1;
    uint32 bit26 : 1;
    uint32 bit27 : 1;
    uint32 bit28 : 1;
    uint32 bit29 : 1;
    uint32 bit30 : 1;
    uint32 bit31 : 1;
};

union IO_OUT1_REG
{
    uint32              all;
    struct IO_OUT1_BITS bit;
};

typedef struct
{
    pd_PlcData   pd_data;               //面板状态给软PLC
    union IO_IN1_REG   IO_In1Reg;             //IO板1输入信号
//    union IO_IN2_REG   IO_In2Reg;             //IO板2输入信号
} MT_PLC_IN;


typedef struct
{
    pd_output panel_show;                     //软PLC 控制操作面板上指示灯 、数码管
    union IO_OUT1_REG IO_Out1Reg;             //IO板1输出信号
//    union IO_OUT2_REG IO_Out2Reg;             //IO板2输出信号
} PLC_MT_OUT;

extern MT_PLC_IN  plc_in;
extern PLC_MT_OUT plc_out;

void Get_PlcToMtData(PLC_MT_OUT*);
void Set_PlcToMtData(PLC_MT_OUT*);
void Clr_PlcToMtData(PLC_MT_OUT*);

void Get_MtToPlcData(MT_PLC_IN*);
void Set_MtToPlcData(MT_PLC_IN*);
void Clr_MtToPlcData(MT_PLC_IN*);

#endif // MT_DATA_H
