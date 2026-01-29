#ifndef PARSERSTATE_H
#define PARSERSTATE_H
#include "coordinate.h"
#include <QHash>
/********************
 * 译码状态数据结构
 *******************/

//=========================00非模态===============================
enum class Unmodality
{
    None,
    Suspend,            //暂停G04
    SplineCurve,        //样条曲线G05.1
    ExactStop,          //精确停止G09
    RefCheck,           //参考点返回检查G27
    BackToRef,          //返回参考点G28
    MoveFromRef,        //从参考点移动G29
    BackToRef_X,        //返回指定参考点G30
    STOPSKIP,           //跳转G31

    Screw,              //螺纹插补G33
    Tap,                //攻丝G33.1
    TapReturn,          //攻丝回退G33.2

    ToolMeasure,        //刀具长度自动测定G37
    ToolComp_Circle,    //刀补圆弧插补G39
    ChildCoord_set,     //设定局部坐标系G52
    MachineCoord_set,   //设定机械坐标系G53
    DirctLocate,        //单向定位G60
    MacroUse,           //宏程序调用G65
    CoordExcursion,     //工件坐标偏置G92
    WCS_set,            //工件坐标系预设
};

//=========================01运动方式===============================
enum class AxesMove_Code
{
    Rapid_Traverse = 0,         //快速定位 G00
    Linear_Interp = 1,          //直线插补 G01
    Circle_Clkwise = 2,         //顺时针圆弧插补 G02
    Circle_Anticlkwise = 3,     //逆时针圆弧插补 G03
    Involute_Clkwise = 4,       //顺时针渐开线插补 G02.2
    Involute_Anticlkwise = 5,   //逆时针渐开线插补 G03.2
    Circle_3D = 6,              //三维空间圆弧插补 G02.4
    Circle_3D_Anti = 7,         //三维空间圆弧插补 G03.4
    Votex_Clkwise = 8,          //顺时针涡旋线插补G02.1
    Votex_Anticlkwise = 9,      //逆时针涡旋线插补G03.1
};

//=========================02坐标平面选择=============================
enum class Current_Plane
{
    X_Y_Plane = 0, //选择XY平面 G17
    Z_X_Plane = 1, //选择ZX平面 G18
    Y_Z_Plane = 2, //选择YZ平面 G19
};

//==========================03坐标编程方式==========================
enum class Absolute_Additive
{
    Absolute = 0,    //G90 绝对坐标编程
    Additive = 1,    //G91 相对坐标编程
};

//==========================05进给速度换算==========================
enum class FeedSpeedConvertion
{
    FeedPerMin,     //G94 每分钟进给
    FeedPerCircle,   //G95 每转进给
};

//==========================06计量单位选择==========================
enum class Inch_Metric
{
    Inch = 0,       //使用英制单位 G20(G70)
    Metric = 1,     //使用公制单位 G21(G71)
};

//=========================07刀补选择/取消=======================
enum class Tool_Comp
{
    NONE = 0,     //( 不执行刀补/结束刀补 G40 )
    Left = 1,     //( 左刀补 G41 )
    Right = 2,	  //( 右刀补 G42 )
};

//========================08刀具长度补偿===========================
enum class Tool_LengthComp
{
    NONE,               //刀具长度补偿取消 G49/G49.1
    Positive,           //刀具长度正补偿 G43
    Negative,           //刀具长度负补偿 G44
};

//========================09固定循环===========================
enum class FixedCycle
{
    G73,
    G74,
    G74_1,
    G76,
    G80,
    G81,
    G82,
    G83,
    G84,
    G84_1,
    G84_2,
    G84_3,
    G85,
    G86,
    G87,
//    G88,    //暂不实现
    G89
};


//========================10固定循环返回点===========================
enum class FixedCycle_Back
{
    BasePlane,      //固定循环基准平面返回 G98
    RPlane,         //固定循环R点平面返回 G99
};

//========================15切削模式===========================
enum class ChopMode
{
    ExactStop,      //精准停止模式 G61
    Tapping,       //攻丝模式 G63
    Chop,           //切削模式 G64
};

//========================可编程输入模式===========================
enum class ProgramInputValid
{
    InputValid,     //可编程输入G10
    UnInputValid    //取消可编程输入G11
};

//========================坐标旋转模式===========================
enum class CoorRotation
{
    CoorRotationON,     //坐标旋转模式激活G68
    CoorRotationOFF    //坐标旋转模式关闭G69
};

//========================极坐标编程模式===========================
enum class PolarCoordinate
{
    PolarCoorON,     //极坐标编程模式激活G16
    PolarCoorOFF  //极坐标编程模式关闭G15
};

//========================坐标缩放模式===========================
enum class CoorScaling
{
    CoorScalingON,     //坐标缩放模式激活G51
    CoorScalingOFF    //坐标缩放模式关闭G50
};

enum class DecodeStatus
{
    HoldDecode,     // 保持译码
    WaitDecode,     // 等待译码
};

struct int_v
{
    int_v(bool valid = false, int value = 0)
    {
        m_valid = valid;
        m_value = value;
    }

    bool m_valid =false;
    int m_value = 0;
};

struct double_v
{
    bool m_valid = false;
    double m_value = 0;
};

struct NcKeyWord
{
    double G = 0;
    int O = 0;
    int N = 0;
    double F = 0;
    int S = 0;
    int T = 0;
    double X = 0;
    double Y = 0;
    double Z = 0;
    double A = 0;
    double B = 0;
    double C = 0;
    double I = 0;
    double J = 0;
    double K = 0;
    double U = 0;
    double V = 0;
    double W = 0;
    int D = 0;
    double D_ = 0;    //,D
    int H = 0;
    int P = 0;
    double R = 0;
    double Q = 0;
    int L = 0;
    int M = -1;
};

struct ParserState
{
    Unmodality unModalState = Unmodality::None;
    AxesMove_Code moveCode = AxesMove_Code::Rapid_Traverse;
    Current_Plane plane = Current_Plane::X_Y_Plane;
    Absolute_Additive absAdd = Absolute_Additive::Absolute;
    FeedSpeedConvertion feedSpeedConvertion = FeedSpeedConvertion::FeedPerMin;

    Inch_Metric unit = Inch_Metric::Metric;
    Tool_Comp toolComp = Tool_Comp::NONE;
    Tool_LengthComp toolLengthcomp = Tool_LengthComp::NONE;
    FixedCycle cycle = FixedCycle::G80;
    FixedCycle_Back cycleBack = FixedCycle_Back::BasePlane;
    ProgramInputValid programInputValid = ProgramInputValid::UnInputValid;//可编程输入模式激活
    bool macroModality = false; //宏模态
    int coordinate = 1; //选择工件坐标系1~6代表G54~G59
    bool toollifeManagement = false;//刀具寿命管理
    int endSpeedMode = 0; //终点速度模式 0:默认TESTOFF指令设定 1:转角平滑测试CSTEST设定
    ChopMode chopMode = ChopMode::Chop;
    CoorRotation coorRotation = CoorRotation::CoorRotationOFF;//坐标旋转
    CoorScaling coorScaling = CoorScaling::CoorScalingOFF;//坐标缩放
    PolarCoordinate polarCoordinate = PolarCoordinate::PolarCoorOFF;
    bool coorRotaStart = false;
    bool coorScalingStart = false;

    DecodeStatus decodeStatus = DecodeStatus::HoldDecode;

    double feedSpeed = 0.0;

    double_v G;
    int_v O;        //程序号O
    int_v N;        //顺序号N
    double_v F;     //进给速率(每分钟速率/每转速率)
    int_v S;        //主轴速率
    int_v T;        //刀具
    double_v X;
    double_v Y;
    double_v Z;
    double_v A;
    double_v B;
    double_v C;
    double_v I;
    double_v J;
    double_v K;
    double_v U;
    double_v V;
    double_v W;
    int_v D;
    double_v D_;    //,D
    int_v H;        //偏置号
    int_v P;        //子程序号指定/程序调用循环次数/暂停/缩放倍率*1000
    double_v R;     //多用于表示半径
    double_v Q;
    int_v L;        //程序调用循环次数
    int_v M = int_v(false, -1);

    CoordPoint g52Coord;
    CoordPoint g92Coord;
    CoordPoint g92_1Coord;
    QHash<int,double> varInputList;
};

#endif // PARSERSTATE_H
