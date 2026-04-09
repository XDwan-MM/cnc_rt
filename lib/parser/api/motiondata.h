#ifndef MOTIONDATA_H
#define MOTIONDATA_H
#ifdef __cplusplus
extern "C"{
#endif

typedef enum pmc_mode_t
{
    PMC_MODE_CHOP,          //切削模式
    PMC_MODE_EXACT_STOP,    //精准停止模式
    PMC_MODE_TAP,           //攻丝模式
    PMC_MODE_PAUSE,         //暂停
    PMC_MODE_MEASURE_TOOL,  //自动测量刀长
    PMC_MODE_TAP_RETURE,    //攻丝回退
    PMC_MODE_M_CMD,         //M代码
    PMC_MODE_TOOL_CHANGE,   //换刀
    PMC_MODE_ARRIVE_CHECK,  //到位检测
    PMC_MODE_STOPSKIP,      //跳转
}pmc_mode;

typedef enum pmc_interp_t
{
    PMC_G00 = 0,            //快速定位 G00
    PMC_G01 = 1,            //直线插补 G01
    PMC_G02 = 2,            //顺时针圆弧插补 G02
    PMC_G03 = 3,            //逆时针圆弧插补 G03
    PMC_G02_2 = 202,        //顺时针渐开线插补 G02.2
    PMC_G03_2 = 302,        //逆时针渐开线插补 G03.2
    PMC_G02_4 = 204,        //三维空间圆弧插补 G02.4
    PMC_G03_4 = 304,        //三维空间圆弧插补 G03.4
    PMC_G33 = 33,           //螺纹插补 G33
    PMC_G33_1 = 331,        //攻丝 G33.1
    PMC_G33_2 = 332,        //攻丝回退 G33.2
    PMC_G37 = 37,           //刀具测量 G37
    PMC_G02_1 = 201,        //顺时针涡旋/圆锥插补 G02.1
    PMC_G03_1 = 301,        //逆时针涡旋/圆锥插补 G03.1
    PMC_G02_5 = 205,        //顺时针螺旋线G02
    PMC_G03_5 = 305,        //逆时针螺旋线G03
//    PMC_G02_3 = 203,        //顺时针指数插补 G02.3
//    PMC_G03_3 = 303,        //逆时针指数插补 G03.3
//    PMC_G02_5 = 205,        //顺时针空间圆弧 G02.5
//    PMC_G03_5 = 305,        //逆时针空间圆弧 G03.5
//    PMC_G601 = 601,         //样条插补
//    PMC_G602 = 602,         //NURBS插补
//    PMC_G603 = 603,         //多项式插补
//    PMC_G604 = 604,         //sin 插补
}pmc_interp;

typedef enum pmc_plane_t
{
    PMC_PLANE_XY = 0, //选择XY平面 G17
    PMC_PLANE_XZ = 1, //选择XZ平面 G18
    PMC_PLANE_YZ = 2, //选择YZ平面 G19
}pmc_plane;

typedef struct pmc_pos_t
{
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;
}pmc_pos;

typedef struct pmc_circle_center_t
{
    double x_coc;    //圆心centre of circle
    double y_coc;    //圆心centre of circle
    double z_coc;    //圆心centre of circle
}pmc_circle_center;

typedef struct pmc_feed_t
{
    enum
    {
        PMC_FEED_PER_MIN,
        PMC_FEED_PER_CIRCLE
    }mode;
    double value;   //进给速率
}pmc_feed;

typedef struct pmc_spindle_t
{
    int speed;  //速率
    int reverse;    //反向 0正 1反
    double angle;  //角度
}pmc_spindle;

typedef enum pmc_mcode
{
    M00 = 0,
    M01,
    M02,
    M03,
    M04,
    M05,
    M06,
    M07,
    M08,
    M09,
    M10,
    M11,
    M12,
    M13,
    M14,
    M15,
    M16,
    M17,
    M18,
    M19,
    M20,
    M21,
    M22,
    M23,
    M24,
    M25,
    M26,
    M27,
    M28,
    M29,
    M30,
    M31,
    M32,
    M33
}pmc_mcode;

typedef struct pmc_data_t
{
    pmc_mode mode;
    pmc_interp interp;
    pmc_plane plane;
    pmc_pos position;
    pmc_pos auxiliary_pos;      //辅助点，用于确定轨迹的第三点，目前用到的指令：G02.4
    pmc_circle_center circle_center;
    double screw_offset;        //螺旋插补的螺距(向量)
    int screw_number;           //螺距数（涡旋数）
    pmc_feed feed;
    pmc_spindle spindle;        //主轴
    int tool_number;            //刀具号
    int tips;                   //跳过指令
    int pause_time;             //暂停时间（ms）
    int m_code;

    int splineCurveMode;        //样条曲线模式
    double baseCirRadius;       //基圆半径

    int endSpeedMode;           //终点速度模式 0：默认 1：转角平滑测试
    int passWay;                // 通道 0 1 2 3
}pmc_data;

#ifdef __cplusplus
}
#endif
#endif // MOTIONDATA_H
