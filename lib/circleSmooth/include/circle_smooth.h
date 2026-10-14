#ifndef CIRCLE_SMOOTH_H
#define CIRCLE_SMOOTH_H

#include <stddef.h>
#include <stdbool.h>
#include "plan_queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CORNER_SMOOTH_PI
#define CORNER_SMOOTH_PI 3.14159265358979323846264338327950288
#endif

// 控制器枚举适配；当前映射必须与 pathplan.c 的平面和方向约定一致。
typedef struct
{
    int plane_xy;                  // 控制器 XY 平面枚举值
    int plane_xz;                  // 控制器 XZ 平面枚举值
    int plane_yz;                  // 控制器 YZ 平面枚举值
    int dir_cw;                    // 控制器顺时针方向枚举值
    int dir_ccw;                   // 控制器逆时针方向枚举值
    int reverse_xy_direction;      // 兼容字段，当前必须为零
    int reverse_xz_direction;      // 兼容字段，当前必须为零
    int reverse_yz_direction;      // 兼容字段，当前必须为零
} CORNER_SMOOTH_CONTROLLER_MAP;

// 几何配置；所有长度参数采用控制器当前长度单位，角度采用弧度。
typedef struct
{
    double min_radius;             // 最小允许圆角半径
    double max_radius;             // 最大允许圆角半径
    double min_turn_angle_rad;     // 最小可平滑转角
    double max_turn_angle_rad;     // 最大可平滑转角，必须小于圆周率
    double geometry_eps;           // 几何有效性和数值精度检查阈值
    double connection_tolerance;   // 连接间隙初筛阈值，不启用端点吸附
    double path_tolerance;         // 顶点到圆弧的最大允许几何偏差
    double segment_use_ratio;      // 单次切除比例，必须在 (0, 0.5] 内

    // 以下字段仅保留结构兼容，当前算法不读取；逐轴约束由控制器负责。
    unsigned int max_constraint_iter;
    double axis_max_vel[3];
    double axis_max_acc[3];
    double axis_max_jerk[3];
    int enable_curvature_jerk_limit;
    double max_chord_error;
} CORNER_SMOOTH_CONFIG;

// 单拐角几何结果，不包含队列状态。
typedef struct
{
    POS tangent_in;                // 前段与圆弧的入切点
    POS tangent_out;               // 圆弧与后段的出切点
    POS center;                    // 圆心位置
    double radius;                 // 圆角半径
    double sweep_angle;            // 正值圆弧扫角
    bool is_ccw;                   // 当前控制器约定下是否为逆时针
    int plane_type;                // 内部平面分类，不可与控制器 PLANE 混用
} SimpleSmoothCorner;

// 算法内部平面编号，通过映射转换为控制器平面枚举。
typedef enum
{
    CIRCLE_PLANE_XY = 0,
    CIRCLE_PLANE_XZ,
    CIRCLE_PLANE_YZ,
    CIRCLE_PLANE_NONE
} PlaneType;

// 初始化默认控制器枚举映射；空指针时直接返回。
void circle_smooth_default_map(CORNER_SMOOTH_CONTROLLER_MAP *map);

// 初始化默认几何约束；空指针时直接返回。
void circle_smooth_default_config(CORNER_SMOOTH_CONFIG *cfg);

// 计算圆角几何，不修改输入；返回 0 成功、-1 不满足条件、1 参数错误。
int circle_smooth_calc_corner(const PLAN_BLOCK *prev_block,
    const PLAN_BLOCK *curr_block, const CORNER_SMOOTH_CONFIG *cfg,
    const CORNER_SMOOTH_CONTROLLER_MAP *ctrl_map, SimpleSmoothCorner *out_corner);

// 根据已验证的几何结果生成圆弧和后段；输出不得与输入或彼此重叠。
// 此接口不输出截短前段，完整三段处理应调用 circle_smooth_pair。
void circle_smooth_generate_blocks(const PLAN_BLOCK *prev_block,
    const PLAN_BLOCK *curr_block, const SimpleSmoothCorner *corner,
    const CORNER_SMOOTH_CONTROLLER_MAP *ctrl_map,
    PLAN_BLOCK *arc_block, PLAN_BLOCK *modified_curr);

// 兼容原单步接口；返回值同几何计算接口，仅输出圆弧和截短后段。
int process_single_smooth_step(const PLAN_BLOCK *last_processed,
    const PLAN_BLOCK *current, const CORNER_SMOOTH_CONFIG *cfg,
    const CORNER_SMOOTH_CONTROLLER_MAP *ctrl_map,
    PLAN_BLOCK *out_arc, PLAN_BLOCK *out_modified_curr);

// 输出截短前段、过渡圆弧、截短后段，不操作队列。
// 返回 0 成功、-1 不满足条件、1 参数错误；失败时不得使用输出结果。
// 三个输出对象必须相互独立，并且不得与任一输入对象重叠。
int circle_smooth_pair(const PLAN_BLOCK *prev, const PLAN_BLOCK *curr,
    const CORNER_SMOOTH_CONFIG *cfg, const CORNER_SMOOTH_CONTROLLER_MAP *map,
    PLAN_BLOCK *out_prev, PLAN_BLOCK *out_arc, PLAN_BLOCK *out_curr);

// 只负责几何平滑及队列顺序处理，不计算运动动态约束，不额外管理线程锁。
// 调用期间必须为输入的唯一消费者、输出的唯一生产者，且不允许并行复位。
// flush 表示输入已结束；否则保留最后一个可平滑直线块等待后继。
// GNC_DATA_EMPTY 同时表示输入为空或需要等待一段前瞻数据。
// 输出容量不足返回 GNC_DATA_FULL，此时两队列数据均保持不变。
GNC_RES circle_smooth_queue_step(PLAN_QUEUE *input, PLAN_QUEUE *output, bool flush,
    const CORNER_SMOOTH_CONFIG *cfg, const CORNER_SMOOTH_CONTROLLER_MAP *map);

#ifdef __cplusplus
}
#endif

#endif // CIRCLE_SMOOTH_H
