#ifndef NCITPERRORTYPE_H
#define NCITPERRORTYPE_H

enum class NCITPErrorType
{
    Exception,                  //无错误
    //1~100词法错误
    NCWordLack,                 //NC程序字不全
    IFLineErr,                  //IF语句格式错误
    GotoNumIllegal,             //GOTO跳转数字非法
    TOOLLIFELineIllegal,        //TOOLLIFE刀具寿命管理指令非法
    ThenLineErr,                //Then语句格式错误
    DoNumErr,                   //Do数字非法
    WhileLineErr,               //While语句格式错误
    EndNumErr,                  //End数字非法
    EndNumLack,                 //End数字缺失
    AssignmentErr,              //赋值语句格式错误
    VariableNumErr,             //程序号数字非法
    ILLegalVaribleName,         //程序号非法
    NoEqualSignErr,             //等号缺失
    NCMacroCoexist,             //宏与NC同时存在
    MacroFormatErr,             //宏语句格式错误
    ILLegalChar,                //非法字符
    ProgramInIllegal,           //可编程输入数字非法
    ToolNumIllegal,             //刀具号非法
    G024UVWDataLess,            //空间圆弧插补中间点坐标数据不全
    GFunctionSyntaxOvermuch,    //G功能语法定义过多

    //101~200宏错误
    TargetBlockNotFound = 101,  //跳转程序段无法找到
    EndLineNotFound,            //End程序段无法找到
    WhileEndNotMatched,         //While-end不匹配
    SingleEndLine,              //End语句没有配对while
    ExprStackErr,               //运算堆栈错误
    DivZero,                    //除零错误
    MacroVarGradeNotEnough,     //宏变量权限不足
    MacroVarNumOverLine,        //宏变量过界

    //201后为nc语句语法错误
    FeedSpeedZero = 201,        //进给率为0
    MiddlePointNotSet,          //中间点未指定
    ILLegalRefPonit,            //参考点指定非法 *
    G27Error,                   //G27错误
    EndPointErr,                //终点坐标错误
    CannotConstituteArc,        //无法构成圆弧
    RUseErr,                    //不能在整圆指令中使用R指定半径
    G40EndWithArc,              //在退刀段不允许切圆弧
    RDiffOverMax,               //圆弧半径值超差
    G37Error,                   //G37不能动作在相对模式下 *
    G87WhileG99,                //在G99模态下指定了G87
    G39WithMove,                //在含G39的程序段中指定了位移指令
    SpiralEndPointErr,          //螺旋线终点错误 *
    IllegalFormatInG02_2G03_2,  //渐开线指令无I, J, K, R指定
    IllegalCommandInG02_2G03_2, //渐开线指令值有误
    CanNotCalToolCompensation,  //无法计算刀补（刀补指令有误或不支持此类型计算）
    IncorrectEnding,            //缺失m02/m30或者卡在子程序里
    MainFileAbsence,            //主文件缺失
    NextLineError,              //进入下一行失败
    M06FileAbsence,             //M06文件缺失 *
    CompExceedRadius,           //刀补半径补偿长度超出圆弧半径
    CompNoIntersection,         //刀补无交点
    CompLineCheckFail,          //刀补直线方向检查未通过
    CompCircleCheckFail,        //刀补圆弧方向检查未通过
    CompBufferHinder,           //刀补缓冲区内发生干涉
    G65FileAbsence,             //G65子程序未找到 *
    M98FileAbsence,             //M99子程序未找到 *
    G68ModeExistOrderConflict,  //G68模态存在指令冲突
    G68RotaionPlanErr,          //同一段G68模态内不能改变旋转平面
    G68RotaionAbsCoordinateErr, //同一段G68模态内不能切换工件坐标系
    G16CircleNeedR,             //G16极坐标模式下，圆弧指令需要指定R
    FixCycle_ZReferencePoint,   //目标位置与基准点定义错误
    G51ModeExistOrderConflict,  //G51模态存在指令冲突 *
    G51ScalingFunctionNotEnabled, //G51比例缩放功能未启用
    G51ScalingRatioZero,         //G51比例缩放倍率为0
    G51SpecialCurveScalingIllegal,//G51特殊曲线缩放非法,只能三轴倍率相同
    InvoluteTarPosDiffOverMax,   //渐开线终点误差超过最大值
    G73QValueLessThanDValue,   //G73进给量小于等于退刀量

    //3000
    MacroAlarm = 3000,          //宏报警
};

#endif // NCITPERRORTYPE_H
