/********************************************************************
 * @file        datatype.h
 * @brief       模拟运动控制外其他数据
 * @version     V1.01
 * @date        2026/01/05
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 ********************************************************************/

#ifndef __DATATYPE_H__
#define __DATATYPE_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 使用项目根目录宏构建日志文件路径
#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

#define CHECK_RES(res) \
    if (!res) break;

#define SYS_EXTERNAL_DECEL_FUNCTION_COUNT 5
#define SYS_EXTERNAL_DECEL_DIRECTION_COUNT 2

// 定义数据类型
typedef int int32_t;

// 返回值类型
typedef enum result {
    // NC
    GNC_OPERATOR_SUCCESS = 0,   // 操作成功
    GNC_OPERATOR_ERROR = 1,     // 操作失败
    GNC_OPERATOR_WARNING = 2,   // 操作警告
    GNC_CAL_SUCCESS = 100,      // 计算成功
    GNC_CAL_ERROR = 101,        // 计算失败
    GNC_PARAM_UNVALID = 102,    // 输入参数无效
    GNC_DATA_EMPTY = 200,       // 数据为空
    GNC_DATA_FULL = 201,        // 数据已满
    GNC_DATA_NULL = 202,        // 数据为NULL
    GNC_PARSER_OVER = 301,      // 译码结束
    GNC_PARSER_WAIT = 302,      // 等待译码
    GNC_PARSER_OVER_AND_WAIT = 303, // 译码结束且等待
    GNC_ARRIVE_CHECK_FAIL,      // 轴定位超差
    GNC_SBL_ARRIVE,
    GNC_LENGTH_IS_ZERO,
    // 运动控制
    VP_CAL_SUCCESS = 400,
    VP_IS_NULL,
    VP_PARAM_UNVALID,
    VP_MAX_VEL_IS_ZERO,
    VP_MAX_ACC_IS_ZERO,
    VP_MAX_JERK_IS_ZERO,
    VP_LENGTH_IS_ZERO,
    VP_LENGTH_UNVALID,
    VP_FEED_RATE_IS_ZERO,
    VP_CAL_ERROR,
    VP_CAL_S_ERROR,
    VP_CAL_V_ERROR,
    VP_DELCINE_TO_END,
    VP_VS_LESS_THAN_ZERO,
    VP_VE_LESS_THAN_ZERO,
    VP_VMAX_LESS_THAN_VS,
    VP_VMAX_LESS_THAN_VE,
    VP_VP_LESS_THAN_VS,
    VP_VP_LESS_THAN_VE,
} GNC_RES;

// 轴类型
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_SP
} AXIS_TYPE;

// 位置
typedef struct pos {
    double x;
    double y;
    double z;
} POS;

// 选择平面
typedef enum plane {
    PLANE_XY = 0,   //选择XY平面 G17
    PLANE_XZ = 1,   //选择XZ平面 G18
    PLANE_YZ = 2,   //选择YZ平面 G19
} PLANE;

// 切矢
typedef POS VECTOR;

// 加减速类型
typedef enum velplan_type {
    T_SHAPE = 0,                // T型
    S_SHAPE_DICHOTOMY = 1,      // S型二分法
    SIN_SHAPE_DICHOTOMY = 2,    // SIN型二分法
    S_SHAPE_EQUATION = 3,       // S型解析法
    SIN_SHAPE_EQUATION = 4,     // SIN型解析法
} VELPLAN_TYPE;

// 回零方式
typedef enum {
    ABSOLUTE_AXIS_HOMING = 0,
    RELATIVE_AXIS_HOMING,
} HOMING_TYPE;

// 插补类型
typedef enum {
    UNKNOWN = 0,
    RAPID,          // 快速进给 G00
    LINE,           // 直线 G01
    CIRCLE_CW,      // 顺时针圆弧 G02
    CIRCLE_CCW,     // 逆时针圆弧 G03
    HELIX_CW,       // 顺时针螺旋 G02
    HELIX_CCW,      // 逆时针螺旋 G03
    RIGID_TAP,      // 刚性攻丝 G84
    TAP_RETURN,     // 攻丝回退
    SPLINE,         // 样条
    PROBE,          // 探针
    NURBS,          
} INTERP_TYPE;

// 一致性处理类型
typedef enum {
    NONE = 0,
    UNIF_ARC,       
    UNIF_BESIER,    // 贝塞尔
    UNIF_HERMIT,    // hermit
    UNIF_NURBS,     // NURBS
    UNIF_CIRCLE,    // 圆弧过渡     
} UNIF_TYPE;

// NC 运行模式
typedef enum {
    MODE_EDIT = 0,   // 编辑
    MODE_MEM,        // MEM
    MODE_MDI,        // MDI
    MODE_DNC,        // DNC
    MODE_MPG,        // 手轮
    MODE_JOG,        // 手动
    MODE_HOMING,     // 回零
    MODE_IDLE,       // 空闲
} NC_MODE;

// NC 事件
typedef enum {
    SYS_EVENT_NONE = 0,
    // 操作面板事件
    SYS_EVENT_RERUN,                    // 循环启动
    SYS_EVENT_PAUSE,                    // 进给保持
    SYS_EVENT_FEED_OVERRIDE,            // 切换进给倍率
    SYS_EVENT_RAPID_OVERRIDE,           // 切换快移倍率
    SYS_EVENT_SPDL_OVERRIDE,            // 切换主轴倍率
    SYS_EVENT_X_POS_PRESS,              // 按下X+
    SYS_EVENT_X_POS_RELEASE,            // 松开X+
    SYS_EVENT_X_NEG_PRESS,              // 按下X-
    SYS_EVENT_X_NEG_RELEASE,            // 松开X-
    SYS_EVENT_Y_POS_PRESS,              // 按下Y+
    SYS_EVENT_Y_POS_RELEASE,            // 松开Y+
    SYS_EVENT_Y_NEG_PRESS,              // 按下Y-
    SYS_EVENT_Y_NEG_RELEASE,            // 松开Y-
    SYS_EVENT_Z_POS_PRESS,              // 按下Z+
    SYS_EVENT_Z_POS_RELEASE,            // 松开Z+
    SYS_EVENT_Z_NEG_PRESS,              // 按下Z-
    SYS_EVENT_Z_NEG_RELEASE,            // 松开Z-
    SYS_EVENT_OUTSIDE_RESET,            // 外部复位
    SYS_EVENT_SINGLE_BLOCK_ON,          // 开启单程序段
    SYS_EVENT_SINGLE_BLOCK_OFF,         
    SYS_EVENT_MAC_LOCK_ON,              // 开启机械锁住
    SYS_EVENT_MAC_LOCK_OFF,             
    SYS_EVENT_IT_ON,                    // 开启所有轴互锁
    SYS_EVENT_IT_OFF,                   
    SYS_EVENT_STLK_ON,                  // 开启启停锁停
    SYS_EVENT_STLK_OFF,                 
    SYS_EVENT_BSL_ON,                   // 开启程序段开始互锁
    SYS_EVENT_BSL_OFF,  
    SYS_EVENT_CSL_ON,                   // 开启切削程序段开始互锁
    SYS_EVENT_CSL_OFF,  
    SYS_EVENT_IT1_ON,                   // 开启轴1互锁
    SYS_EVENT_IT1_OFF,  
    SYS_EVENT_IT2_ON,                   // 开启轴2互锁
    SYS_EVENT_IT2_OFF,  
    SYS_EVENT_IT3_ON,                   // 开启轴3互锁
    SYS_EVENT_IT3_OFF,  
    SYS_EVENT_IT4_ON,                   // 开启轴4互锁
    SYS_EVENT_IT4_OFF,  
    SYS_EVENT_IT5_ON,                   // 开启轴5互锁
    SYS_EVENT_IT5_OFF,  
    SYS_EVENT_PMIT1_ON,                 // 开启轴1正向互锁
    SYS_EVENT_PMIT1_OFF,    
    SYS_EVENT_PMIT2_ON,                 // 开启轴2正向互锁
    SYS_EVENT_PMIT2_OFF,    
    SYS_EVENT_PMIT3_ON,                 // 开启轴3正向互锁
    SYS_EVENT_PMIT3_OFF,    
    SYS_EVENT_PMIT4_ON,                 // 开启轴4正向互锁
    SYS_EVENT_PMIT4_OFF,    
    SYS_EVENT_PMIT5_ON,                 // 开启轴5正向互锁
    SYS_EVENT_PMIT5_OFF,    
    SYS_EVENT_NMIT1_ON,                 // 开启轴1负向互锁
    SYS_EVENT_NMIT1_OFF,    
    SYS_EVENT_NMIT2_ON,                 // 开启轴2负向互锁
    SYS_EVENT_NMIT2_OFF,    
    SYS_EVENT_NMIT3_ON,                 // 开启轴3负向互锁
    SYS_EVENT_NMIT3_OFF,    
    SYS_EVENT_NMIT4_ON,                 // 开启轴4负向互锁
    SYS_EVENT_NMIT4_OFF,    
    SYS_EVENT_NMIT5_ON,                 // 开启轴5负向互锁
    SYS_EVENT_NMIT5_OFF,    
    SYS_EVENT_DRY_RUN_ON,               // 开启空运行
    SYS_EVENT_DRY_RUN_OFF,  
    SYS_EVENT_PROGRAM_RESTART_ON,       // 开启程序段重启
    SYS_EVENT_PROGRAM_RESTART_OFF,  
    SYS_EVENT_BLOCK_SKIP_ON,            // 开启跳段
    SYS_EVENT_BLOCK_SKIP_OFF,   
    SYS_EVENT_STOP_SKIP_ON,             // 开启跳过
    SYS_EVENT_STOP_SKIP_OFF,    
    SYS_EVENT_RIGID_RETURN_ON,          // 开启攻丝回退
    SYS_EVENT_RIGID_RETURN_OFF,
    SYS_EVENT_EXTERNAL_DECEL_POS_ON,    /* 外部减速正向生效 */
    SYS_EVENT_EXTERNAL_DECEL_POS_OFF,   /* 外部减速正向解除 */
    SYS_EVENT_EXTERNAL_DECEL_NEG_ON,    /* 外部减速负向生效 */
    SYS_EVENT_EXTERNAL_DECEL_NEG_OFF,   /* 外部减速负向解除 */
    // MDI事件
    SYS_EVENT_HMI_RESET,                // 按下reset
    SYS_EVENT_COMP_PARAM_CHANGED,       /* 误差补偿参数变更（需 sync + on_params_synced） */
    SYS_EVENT_COMP_TABLE_CHANGED,       /* 数据表参数变更（需重建表 + on_params_synced） */
} SYS_EVENT;

double POS_Distance(POS* p1, POS* p2);
POS POS_VECTOR(POS* p1, POS* p2);
double POS_Norm(POS p);
double POS_Dot(POS p1, POS p2);
VECTOR VECTOR_Unit(POS* p1, POS* p2);
void POS_COPY(POS* dest, POS* src);
double CrossVector(POS* AB,POS* BC );

#ifdef __cplusplus
}
#endif

#endif
