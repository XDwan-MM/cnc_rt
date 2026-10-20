#include "mpg.h"
#include "mpg_planner.h"
#include "global.h"
#include <GreeMasterAPI.h>
#include <math.h>
#include <string.h>

#define MPG_FEED_AXIS_COUNT 3

/*
 * MPG 控制层负责把面板选择和手轮计数转换成单轴目标，再将规划器结果写回
 * 实时轴/通道数据。上下文在周期调用之间保留；急停入口会清空整个上下文。
 */
typedef struct
{
    mpg_planner_t planner_by_axis[MPG_FEED_AXIS_COUNT]; /* X/Y/Z 各自独立的轨迹状态 */
    int selected_axis;                                  /* 当前面板选中的轴数组下标；-1 表示无效 */
    int selected_multiplier;                            /* 当前倍率数值：1、10 或 100 */
    bool enabled;                                       /* 最近一次采样是否为有效的轴和倍率组合 */
    bool initialized;                                   /* 是否已经建立过有效选择及计数基准 */
    int last_count_direction[MPG_FEED_AXIS_COUNT];      /* 最近采样计数差的符号：正 1、负 -1、无增量 0 */
} mpg_context_t;

static mpg_context_t mpg_context = {
    .selected_axis = -1,
    .selected_multiplier = -1};
static uint32_t mpg_switch_word = 0;    /* 面板轴/倍率开关的原始位域 */
static uint32_t mpg_hardware_count = 0; /* 最近一次从手轮硬件读取的 32 位计数 */

/* 内部函数声明 */
static int mpg_decode_multiplier(int switch_value);
static int mpg_decode_axis(int switch_value);
static bool mpg_axis_is_valid(int axis);
static bool mpg_selection_is_valid(int axis, int multiplier);
static void mpg_update_axis_output(int axis);
static void mpg_stop_axis_chasing(int axis);
static int mpg_get_motion_direction(const mpg_planner_t *planner);
static double mpg_get_external_decel_velocity(int axis, int direction);
static double mpg_get_velocity_limit(int axis, const mpg_planner_t *planner);
static void mpg_update_active_axis(void);

// ljw
static bool handwheel_is_mac_lock_switching(void);
static void handwheel_stop_chasing(mpg_planner_t *tp);
static bool handwheel_motion_is_stopped(void);
static void handwheel_update_mac_lock(void);

/* 外部函数定义 */

/*
 * 急停入口：清除所有轴的 MPG 轨迹状态和面板选择，并将 X/Y/Z 及通道 0 的
 * 指令速度置零。轴命令位置不在这里改写；急停后下次有效选择会从当前命令位置
 * 重新建立规划基准。
 * 调用时机：急停处理流程要求 MPG 丢弃待执行位置和速度时。
 */
void handwheel_handle_estop(void)
{
    memset(&mpg_context, 0, sizeof(mpg_context));
    mpg_context.selected_axis = -1;
    mpg_context.selected_multiplier = -1;
    if (rt_data != NULL)
    {
        for (int axis_index = 0; axis_index < MPG_FEED_AXIS_COUNT; axis_index++)
            rt_data->axis_data[axis_index].cmd_vel = 0.0;
        rt_data->chan_data[0].cmd_vel = 0.0;
    }
}

/*
 * 每个运动周期调用一次，完成以下处理：读取手轮计数和面板开关、校验轴/倍率、
 * 更新目标位置、选择基础或外部减速速度上限、推进加速度斜坡，最后写回轴/通道。
 *
 * 无效选择会停止接收新的计数，并把当前轴的追踪目标收至当前位置，让规划器按
 * 加速度限制减速；轴或倍率变化则先用本周期计数重置计数基准，避免把切换前后的
 * 计数差误当成新的手轮输入。急停状态由急停入口统一清理，本周期在此直接返回。
 */
void handle_mpg_cmd(void)
{
    bool valid_selection;
    bool selection_changed;
    int selected_axis;
    int selected_multiplier;
    int previous_axis;
    mpg_planner_t *planner;

    /* 实时数据尚未就绪或系统处于急停时，不读取/推进 MPG；急停清理由专用入口完成。 */
    if (rt_data == NULL || rt_data->sys_state[SYS_IS_ESTOP_ACTIVE])
        return;

    /* 轴选择位于 bits 3..7，倍率选择位于 bits 0..2；解码后再做轴槽位校验。 */
    HW_Get_Count(&mpg_hardware_count);
    HW_Get_Axis_And_Rate(&mpg_switch_word);
    selected_multiplier = mpg_decode_multiplier((int)(mpg_switch_word & 0x7U));
    selected_axis = mpg_decode_axis((int)((mpg_switch_word >> 3) & 0x1fU));
    valid_selection = mpg_selection_is_valid(selected_axis, selected_multiplier);

    if (!valid_selection)
    {
        /* OFF 或非法编码会禁止新输入；同步计数可避免无效期间的转动延迟到恢复后执行。 */
        mpg_context.enabled = false;
        if (mpg_context.initialized && mpg_axis_is_valid(mpg_context.selected_axis))
        {
            planner = &mpg_context.planner_by_axis[mpg_context.selected_axis];
            planner->count = mpg_hardware_count;
            planner->previous_count = mpg_hardware_count;
            /* 目标收至当前位置，保留当前速度并由规划器按加速度上限减速。 */
            mpg_stop_axis_chasing(mpg_context.selected_axis);
        }
        /* 保留当前有效轴的速度/位置输出，或在没有有效轴时清零通道速度。 */
        mpg_update_active_axis();
        return;
    }

    /* 失效后重新启用也视为切换，使被忽略期间的计数不进入目标位置。 */
    selection_changed = (!mpg_context.initialized || !mpg_context.enabled ||
                         selected_axis != mpg_context.selected_axis ||
                         selected_multiplier != mpg_context.selected_multiplier);
    if (selection_changed && mpg_context.initialized &&
        mpg_axis_is_valid(mpg_context.selected_axis) &&
        mpg_context.selected_axis != selected_axis)
    {
        previous_axis = mpg_context.selected_axis;
        /* 切换到另一根轴时，旧轴不再接收脉冲，但仍沿规划加速度斜坡减速。 */
        mpg_stop_axis_chasing(previous_axis);
        mpg_update_axis_output(previous_axis);
    }

    planner = &mpg_context.planner_by_axis[selected_axis];
    if (selection_changed)
    {
        /*
         * 切换轴或倍率时，用当前计数建立新基准，并从实时命令位置重建规划器。
         * first_cycle 让规划器消费本周期计数但不产生位置增量，避免选择变化时跳动。
         */
        planner->count = mpg_hardware_count;
        planner->previous_count = mpg_hardware_count;
        planner->first_cycle = true;
        planner->axis = (AXIS_TYPE)selected_axis;
        planner->target_position = rt_data->axis_data[selected_axis].wcs_cmd_pos_cur;
        planner->reference_position = rt_data->axis_data[selected_axis].wcs_cmd_pos_cur;
        planner->current_position = rt_data->axis_data[selected_axis].wcs_cmd_pos_cur;
        planner->current_velocity = 0.0;
        mpg_context.last_count_direction[selected_axis] = 0;
    }

    mpg_context.selected_axis = selected_axis;
    mpg_context.selected_multiplier = selected_multiplier;
    mpg_context.enabled = true;
    mpg_context.initialized = true;

    planner->count = mpg_hardware_count;
    planner->multiplier = (uint32_t)selected_multiplier;
    planner->axis = (AXIS_TYPE)selected_axis;
    planner->max_velocity = rt_data->axis_data[selected_axis].mpg_spd;
    planner->max_acceleration = rt_data->axis_data[selected_axis].max_feed_acc;
    planner->reference_position = rt_data->axis_data[selected_axis].wcs_cmd_pos_cur;

    /*
     * 记录本周期新计数差的方向。无新计数时置零；减速方向随后会回退到未完成的
     * 目标误差，若误差已为零则使用当前速度方向，因此手轮停转时仍能限制减速。
     */
    if (planner->first_cycle)
        mpg_context.last_count_direction[selected_axis] = 0;
    else
    {
        int32_t count_delta;

        count_delta = (int32_t)(mpg_hardware_count - planner->previous_count);
        if (count_delta > 0)
            mpg_context.last_count_direction[selected_axis] = 1;
        else if (count_delta < 0)
            mpg_context.last_count_direction[selected_axis] = -1;
        else
            mpg_context.last_count_direction[selected_axis] = 0;
    }

    /*
     * 先用基础 mpg_spd 计算目标允许超前量和停止距离，保持原有手轮目标跟随行为；
     * 再计算当前方向的最终速度上限并推进规划器。外部减速只影响速度规划，信号
     * 解除后恢复基础速度也通过同一加速度斜坡完成，不会瞬间改变速度。
     */
    mpg_planner_update_target(planner);
    planner->max_velocity = mpg_get_velocity_limit(selected_axis, planner);
    mpg_planner_step(planner, rt_data->ncu_data.cycle);
    planner->reference_position = planner->current_position;
    mpg_update_axis_output(selected_axis);
}

void handwheel_resync_position(void)
{
    /*
     * 清掉轴选/倍率的历史状态，使下一次 handle_mpg_cmd() 走“选择变化”分支：
     * 以实时数据中的 wcs_cmd_pos_cur 重建目标位置和规划位置、速度清零，
     * 并把 first_pass 置 1，从而丢弃未选择期间的累计计数。
     * 由 emcmot 在重新进入手轮模式时调用：离开手轮后其他模式可能已经移动过机床，
     * 不能沿用上一次手轮会话的规划位置（否则回到手轮的第一个周期会产生位置跳变）。
     */

    // valid_axis = -1;
    // valid_rate = -1;
    // mpg_enable = false;
    // mpg_initialized = false;
}

/* 内部函数定义 */

/*
 * 将面板倍率编码转换为规划器使用的倍率数值。
 * 参数 switch_value 是倍率开关的低 3 位编码；返回值为 1、10、100，非法编码返回 -1。
 * 该函数只负责解码，不修改 MPG 上下文。
 */
static int mpg_decode_multiplier(int switch_value)
{
    /* 面板开关编码与倍率数值不同；其他编码统一判为无效。 */
    switch (switch_value)
    {
    case HANDWHEEL_RATE_X1:
        return 1;
    case HANDWHEEL_RATE_X10:
        return 10;
    case HANDWHEEL_RATE_X100:
        return 100;
    default:
        return -1;
    }
}

/*
 * 将面板轴选择编码转换为实时轴数组下标。
 * 目前 MPG 只接受 X/Y/Z 三个进给轴；4、5 轴及 OFF 保留编码均返回 -1，
 * 以避免把面板编码直接当数组下标造成主轴访问或越界。
 */
static int mpg_decode_axis(int switch_value)
{
    /* 只映射当前实现有对应实时数组槽位的 X/Y/Z，其他保留编码均拒绝。 */
    switch (switch_value)
    {
    case HANDWHEEL_AXIS_X:
        return AXIS_X;
    case HANDWHEEL_AXIS_Y:
        return AXIS_Y;
    case HANDWHEEL_AXIS_Z:
        return AXIS_Z;
    case HANDWHEEL_AXIS_OFF:
    case HANDWHEEL_AXIS_4:
    case HANDWHEEL_AXIS_5:
    default:
        return -1;
    }
}

/*
 * 检查轴下标是否属于 MPG 支持的连续进给轴范围。
 * 返回值仅表示数组访问和轨迹状态访问是否安全，不表示该轴当前是否被选中。
 */
static bool mpg_axis_is_valid(int axis)
{
    /* 进给轴槽位连续排列为 X、Y、Z；避免把主轴或保留轴编码当数组下标使用。 */
    if (axis >= AXIS_X && axis <= AXIS_Z)
        return true;
    return false;
}

/*
 * 检查轴和倍率是否构成一次可执行的面板选择。
 * 解码失败时轴为 -1 或倍率为负数，调用方会进入受控停止路径并同步计数基准。
 */
static bool mpg_selection_is_valid(int axis, int multiplier)
{
    /* 有效面板选择必须同时包含支持的进给轴和已解码的正倍率。 */
    return mpg_axis_is_valid(axis) && multiplier > 0;
}

/*
 * 将某一规划器的当前位置和速度写回实时数据。
 * 位置使用规划器积分后的当前位置；轴速度和通道速度保持历史约定，写入速度绝对值，
 * 运动方向由位置变化体现。无效轴或实时数据未初始化时函数不产生任何副作用。
 */
static void mpg_update_axis_output(int axis)
{
    mpg_planner_t *planner;

    if (rt_data == NULL || !mpg_axis_is_valid(axis))
        return;
    planner = &mpg_context.planner_by_axis[axis];
    /* 轴和通道速度沿用既有绝对值语义；运动正负方向由位置及计数方向表示。 */
    rt_data->axis_data[axis].wcs_cmd_pos_cur = planner->current_position;
    rt_data->axis_data[axis].cmd_vel = fabs(planner->current_velocity);
    rt_data->chan_data[0].cmd_vel = fabs(planner->current_velocity);
}

/*
 * 撤销指定轴尚未完成的目标位置，并推进一个周期的减速过程。
 * 这里不直接清零 current_velocity，目的是让停止仍受最大加速度约束，避免指令速度阶跃。
 * 调用方负责保证实时数据已就绪；无效轴输入会被安全忽略。
 */
static void mpg_stop_axis_chasing(int axis)
{
    mpg_planner_t *planner;

    if (!mpg_axis_is_valid(axis))
        return;
    planner = &mpg_context.planner_by_axis[axis];
    /* 停止追踪只撤销未完成的位置请求，不直接清零当前速度，避免速度阶跃。 */
    planner->target_position = planner->current_position;
    mpg_context.last_count_direction[axis] = 0;
    planner->max_velocity = mpg_get_velocity_limit(axis, planner);
    mpg_planner_step(planner, rt_data->ncu_data.cycle);
    planner->reference_position = planner->current_position;
    mpg_update_axis_output(axis);
}

/*
 * 推断当前规划运动方向，供方向相关的外部减速信号使用。
 * 优先级为：本周期计数增量方向、目标位置误差方向、当前速度方向；三者都不存在时返回 0。
 * 返回值为 1（正方向）、-1（负方向）或 0（方向未知），不修改规划器状态。
 */
static int mpg_get_motion_direction(const mpg_planner_t *planner)
{
    double position_error;
    int axis;

    if (planner == NULL)
        return 0;
    axis = (int)planner->axis;
    /* 优先采用本周期手轮输入方向；没有新输入时按剩余目标误差判定。 */
    if (mpg_axis_is_valid(axis) && mpg_context.last_count_direction[axis] != 0)
        return mpg_context.last_count_direction[axis];
    position_error = planner->target_position - planner->current_position;
    if (position_error > 1.0e-8)
        return 1;
    if (position_error < -1.0e-8)
        return -1;
    if (planner->current_velocity > 1.0e-9)
        return 1;
    if (planner->current_velocity < -1.0e-9)
        return -1;
    /* 静止且没有剩余误差时方向不确定，不匹配任一方向减速信号。 */
    return 0;
}

/*
 * 汇总当前轴、当前方向上生效的外部减速速度上限。
 * 功能 1 使用总开关，功能 2~5 还要满足各自启用开关；同一时刻多个信号生效时取最小值。
 * 未触发减速、方向未知或输入无效时返回 HUGE_VAL，表示不额外限制基础 MPG 速度。
 */
static double mpg_get_external_decel_velocity(int axis, int direction)
{
    bool function_enabled[SYS_EXTERNAL_DECEL_FUNCTION_COUNT];
    double configured_velocity[SYS_EXTERNAL_DECEL_FUNCTION_COUNT];
    double limit;
    int direction_index;

    /* 总开关关闭、轴无效或方向不明确时，不施加外部速度限制。 */
    if (rt_data == NULL || !mpg_axis_is_valid(axis) ||
        !rt_data->chan_data[0].ex_dec_enable || (direction != 1 && direction != -1))
        return HUGE_VAL;

    /*
     * 状态矩阵第二维 0/1 对应正/负方向。功能 1 由总开关控制；功能 2..5 还必须
     * 分别启用。只有轴、功能和运动方向都匹配的有效信号才参与速度上限比较。
     */
    direction_index = (direction > 0) ? 0 : 1;
    function_enabled[0] = true;
    function_enabled[1] = rt_data->chan_data[0].ex_dec2_enable;
    function_enabled[2] = rt_data->chan_data[0].ex_dec3_enable;
    function_enabled[3] = rt_data->chan_data[0].ex_dec4_enable;
    function_enabled[4] = rt_data->chan_data[0].ex_dec5_enable;
    configured_velocity[0] = rt_data->axis_data[axis].mpg_ex_dec1_vel;
    configured_velocity[1] = rt_data->axis_data[axis].mpg_ex_dec2_vel;
    configured_velocity[2] = rt_data->axis_data[axis].mpg_ex_dec3_vel;
    configured_velocity[3] = rt_data->axis_data[axis].mpg_ex_dec4_vel;
    configured_velocity[4] = rt_data->axis_data[axis].mpg_ex_dec5_vel;

    limit = HUGE_VAL;
    for (int function = 0; function < SYS_EXTERNAL_DECEL_FUNCTION_COUNT; function++)
    {
        if (!function_enabled[function] ||
            !rt_data->axis_data[axis].status.external_decel_active[function][direction_index])
            continue;
        /* 非有限或负速度参数按 0 处理，采用停机限速而不是忽略已触发的减速信号。 */
        if (!isfinite(configured_velocity[function]) || configured_velocity[function] < 0.0)
            configured_velocity[function] = 0.0;
        /* 多个减速输入同时有效时，最小的非负速度值是共同上限。 */
        if (configured_velocity[function] < limit)
            limit = configured_velocity[function];
    }
    return limit;
}

/*
 * 计算本周期规划器允许使用的最终速度上限。
 * 先清洗轴参数中的基础 mpg_spd，再叠加当前方向的外部减速上限；无效速度按 0 处理。
 * 该函数只读取实时数据和规划器状态，实际的加速度斜坡由 mpg_planner_step 完成。
 */
static double mpg_get_velocity_limit(int axis, const mpg_planner_t *planner)
{
    double base_velocity;
    double external_velocity;
    int direction;

    if (rt_data == NULL || planner == NULL || !mpg_axis_is_valid(axis))
        return 0.0;
    /* 基础 MPG 速度无效时按零速处理，防止非有限值进入轨迹规划器。 */
    base_velocity = rt_data->axis_data[axis].mpg_spd;
    if (!isfinite(base_velocity) || base_velocity < 0.0)
        base_velocity = 0.0;
    direction = mpg_get_motion_direction(planner);
    external_velocity = mpg_get_external_decel_velocity(axis, direction);
    /* 未触发减速时 external_velocity 为 HUGE_VAL，结果自然退回基础手轮速度。 */
    if (external_velocity < base_velocity)
        return external_velocity;
    return base_velocity;
}

/*
 * 更新当前有效轴的输出，供 OFF/非法选择路径使用。
 * 轴选择仍有效时保留规划器正在执行的受控减速；从未建立有效选择时仅清零通道速度，
 * 不触碰其他轴的位置和速度数据。
 */
static void mpg_update_active_axis(void)
{
    /* 无效选择时继续输出已选轴的受控减速过程；从未选中过有效轴则清通道速度。 */
    if (mpg_axis_is_valid(mpg_context.selected_axis))
        mpg_update_axis_output(mpg_context.selected_axis);
    else if (rt_data != NULL)
        rt_data->chan_data[0].cmd_vel = 0.0;
}

/*
 * 机械锁住请求与实际生效状态不一致：正在锁住或正在解除。
 * 此时 handle_mpg_cmd() 不再接收新的手轮位移、并把目标追平当前位置让规划器停稳，
 * 停稳后 handwheel_update_mac_lock() 才让 READY 跟随 ON。
 */
static bool handwheel_is_mac_lock_switching(void)
{
    return rt_data->sys_state[SYS_IS_MAC_LOCK] != rt_data->sys_state[SYS_IS_MAC_LOCK_READY];
}

/* 锁住状态切换期间停止追赶已有目标：目标位置对齐当前规划位置，由规划器按斜坡停稳 */
static void handwheel_stop_chasing(mpg_planner_t *tp)
{
    // tp->pos_cmd = tp->curr_pos;
}

/* 当前轴选的规划器是否已停稳（锁住状态切换的判定条件）；无有效轴选视为已停稳 */
static bool handwheel_motion_is_stopped(void)
{
    // if (!mpg_initialized || valid_axis < 0 || valid_axis >= 5)
    //     return true;
    // return fabs(simple_tp[valid_axis].curr_vel) < JOG_SPEED_THRESHOLD;
}

/*
 * 机械锁住：处理开关事件，并维护“请求(ON)”与“生效(READY)”两个状态。
 * 与自动模式一致，两者分开：
 *   ON=0,READY=0 未锁住 / ON=1,READY=0 已请求、先让手轮运动停稳
 *   ON=1,READY=1 锁住生效（机床指令位置不再累加，手轮照常规划账面）
 *   松开时对称：ON=0,READY=1 先停稳 → ON=0,READY=0 解除
 * 停稳后 READY 才跟随 ON；手轮的计数与规划状态全程保留，所以锁住期间继续摇
 * 只走账面（虚拟位移），解除后机床从冻结值继续累加、差值保留。
 */
static void handwheel_update_mac_lock(void)
{
    bool lock_request;
    bool lock_effective;

    /*
     * 先按“上一周期结束时的请求/生效状态”判断能不能切换，再处理本周期的开关事件
     * （与自动模式一致），这样 ON=1,READY=0 / ON=0,READY=1 过渡态至少存在一个周期。
     */
    lock_request = rt_data->sys_state[SYS_IS_MAC_LOCK];
    lock_effective = rt_data->sys_state[SYS_IS_MAC_LOCK_READY];

    /* 请求与实际不一致（正在锁住或正在解除）：先等手轮运动停稳，再切换生效状态 */
    if (lock_request != lock_effective && handwheel_motion_is_stopped())
    {
        rt_data->sys_state[SYS_IS_MAC_LOCK_READY] = lock_request;
        LOG(LOG_LEVEL_INFO, "手动模式机械锁住%s", lock_request ? "生效" : "解除");
    }

    /* 本周期开关事件：只改“请求” */
    switch (rt_data->nc_event)
    {
    case SYS_EVENT_MAC_LOCK_ON:
        LOG(LOG_LEVEL_INFO, "手动模式处理开启机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK] = true;
        break;
    case SYS_EVENT_MAC_LOCK_OFF:
        LOG(LOG_LEVEL_INFO, "手动模式处理关闭机械锁住");
        rt_data->sys_state[SYS_IS_MAC_LOCK] = false;
        break;
    default:
        break;
    }
}