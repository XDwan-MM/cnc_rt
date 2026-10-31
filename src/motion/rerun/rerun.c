#include "rerun.h"
#include "global.h"
#include <hmisysapi.h>
#include "alarmapi.h"
#include "cnc_math.h"
#include "bufstop.h"
#include "ncitpstopdef.h"


#define RERUN_SNAPSHOT_TICKS (1) //运行时存储数据刷新频率

// 停止是否记断点标志（急停、报警、切换模式停止）需要置1
// 复位、程序跑完不置位
static bool rerun_interrupted =false;


static void rerun_persist_force(void);

///
static int rerun_block_m_code(const PLAN_BLOCK* block)
{
    return (block->mode == PMC_MODE_M_CMD) ? block->mst_data.m_code : -1;
}

GNC_RES rerun_data_init(){
    rerun_data = (RERUN_DATA*)calloc(sizeof(RERUN_DATA), 1);
    if(rerun_data == NULL) return GNC_DATA_NULL;
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    rerun_data->cur_row_line = -1;
    return GNC_OPERATOR_SUCCESS;
}

/// @brief 换程序运行：清除再启动断点信息（含断点记录）
void rerun_data_reset(){
    memset(&rerun_data->break_block, 0, sizeof(rerun_data->break_block));
    memset(&rerun_data->breakpoint, 0, sizeof(rerun_data->breakpoint));
    memset(&rerun_data->break_end, 0, sizeof(rerun_data->break_end));
    memset(&rerun_data->program_start, 0, sizeof(rerun_data->program_start));
    rerun_data->program_start_valid = false;
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    rerun_data->cur_row_line = -1;
    rerun_interrupted =false;
    rerun_mst_reset();
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    delAlarm(ALARM_CH, ALARM_ERROR, 4);
    rerun_persist_force();
}
/// @brief 复位：只清运行状态，保留断点记录
/// @note is_rerun /is_mem_rerun 关掉，要拿记录跳行，后续有页面把rerun_row设成
///       break_blocj.line_number和置is_rerun为1；
void rerun_data_reset_keep_breakpoint(){
    if (rerun_data == NULL) return;
    const bool warning = rerun_data->is_rerun && !rerun_data->is_first_time_click_rerun;
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    rerun_interrupted =false;
    rerun_mst_reset();
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    delAlarm(ALARM_CH, ALARM_ERROR, 4);
    if(warning && rt_data->current_mode == MODE_MEM)
    {
        setVarChanInt(0, VAR_CHAN_RUN_ROW, 0, 0);
    }
    rerun_persist_force();
}

/// @brief 模态分组分类表
/// @note  暂时用于转速 S 挂在哪一条 M 上。
static MST_MODAL_GROUP rerun_mst_group(int m_code)
{
    switch (m_code)
    {
    case M03:
    case M04:
    case M05:
        return MST_GROUP_SPINDLE;
    case M08:
    case M09:
        return MST_GROUP_COOLANT;
    default:
        return MST_GROUP_NONE;
    }
}

void rerun_mst_reset(void)
{
    if (rerun_data == NULL) return;
    memset(&rerun_data->mst, 0, sizeof(rerun_data->mst));
}

void rerun_mst_collect(const PLAN_BLOCK* block)
{
    if (rerun_data == NULL || block == NULL) return;
    MST_RESTORE* r = &rerun_data->mst;

    if(block->mode != PMC_MODE_M_CMD) return;

    if (block->mst_data.spindle.speed > 0 &&
        block->mst_data.spindle.speed != r->s[0]) {
        r->s[1] = r->s[0];
        r->s[0] = block->mst_data.spindle.speed;
    }

  
    const int m = block->mst_data.m_code;
    if (m < 0) return;

    // 程序控制/文件调用/阻止缓冲的 M 码不参与重演
    if (m == M00 || m == M01 || m == M02 || m == M30
        || m == ITP_BUFEXEC_M65 || m == ITP_BUFEXEC_M66
        || m == ITP_BUFEXEC_M98 || m == ITP_BUFEXEC_M99
        || parser_isBufStopMCode(m))
        return;

    // 在 M06 上取刀号
    if (m == M06 && block->mst_data.tool_number >= 0) {
        r->t = block->mst_data.tool_number;
    }

    if (r->count >= MST_REPLAY_MAX) {
        memmove(&r->m_code[0], &r->m_code[1],
                sizeof(r->m_code[0]) * (MST_REPLAY_MAX - 1));
        r->m_code[MST_REPLAY_MAX - 1] = m;
        return;
    }
    r->m_code[r->count++] = m;
}

void rerun_make_mst_block(const MST_RESTORE* r, int index,
                          const PLAN_BLOCK* target, PLAN_BLOCK* out)
{
    if (r == NULL || out == NULL || target == NULL) return;
    if (index < 0 || index >= r->count) return;

    memset(out, 0, sizeof(*out));
    out->mode = PMC_MODE_M_CMD;
    out->state = target->state;
    // 统一显示在再启动行：恢复期间 HMI 的运行行不来回跳
    out->line_number = rerun_data->rerun_row;
    out->is_breakpoint = true;
    out->mst_data.m_code = r->m_code[index];
    out->mst_data.tool_number = -1;
    out->mst_data.spindle.speed = 0;
    out->mst_data.size = 1;

    // M03、04、05带上最新S号
    if (rerun_mst_group(r->m_code[index]) == MST_GROUP_SPINDLE) {
        out->mst_data.spindle.speed = r->s[0];
    }
    // M06带上刀号T
    if (r->m_code[index] == M06) {
        out->mst_data.tool_number = r->t;
    }
}

/// @brief 清掉断点快照（恢复运行 / 从头跑时调用）

void rerun_data_clear_breakpoint(void)
{
    if (rerun_data == NULL) return;
    if (rerun_data->is_rerun) return; //跳行依据
    memset(&rerun_data->break_block, 0, sizeof(rerun_data->break_block));
    memset(&rerun_data->breakpoint, 0, sizeof(rerun_data->breakpoint));
    memset(&rerun_data->break_end, 0, sizeof(rerun_data->break_end));
    rerun_data->is_rerun = 0;
    rerun_data->is_mem_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data->is_first_time_click_rerun = true;
    rerun_interrupted =false;
    rerun_data->cur_row_line =-1;
    memset(&rerun_data->cur_row_first_end, 0, sizeof(rerun_data->cur_row_first_end));
    rerun_mst_reset();
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    delAlarm(ALARM_CH, ALARM_ERROR, 4);
    rerun_persist_force();
}

/// @brief 程序段换行时把这一行的段信息写进断电块
static void rerun_snapshot_row(const PLAN_BLOCK* block)
{
    rerun_data->break_block.line_number = block->line_number;
    rerun_data->break_block.mode        = block->mode;
    rerun_data->break_block.type        = block->type;
    rerun_data->break_block.m_code      = rerun_block_m_code(block);   
    // valid = 这份段信息有效（有行号、有坐标可参考）
    // 但是不等于可以自动再启动—— 能不能自动跳行看 is_mem_rerun，那个只由打断路径置位
    rerun_data->break_block.valid       = true;
    memcpy(&rerun_data->break_end, &rerun_data->cur_row_first_end, sizeof(POS));
    rerun_persist();
}

/// @brief 运行中持续刷新断电块
/// @note  掉电时界面是先存盘、再给实时发信号触发急停，实时那条记录赶不上存盘，
///        所以运行中就把行号/坐标持续写进共享内存。
///        断电记录的位置与真正急停触发的位置差一点。
///        只更新数据，不置 is_mem_rerun —— 跑到这和被打断过、能自动再启动是两回事。
void rerun_snapshot_position(const PLAN_BLOCK* block)
{
    static int ticks = 0;

    if (rerun_data == NULL || block == NULL) return;
    if (rt_data->current_mode != MODE_MEM) return;
    if (cycle_tp == NULL || cycle_tp->exec_state != EXEC_RUNNING) return;
    if (block->is_breakpoint) return;     // 回断点定位段不计
    if (rerun_data->is_rerun) return;     // 回断点途中不刷，否则会把回断点目标盖掉

    rerun_data->breakpoint.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    rerun_data->breakpoint.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    rerun_data->breakpoint.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;

    if (++ticks < RERUN_SNAPSHOT_TICKS) return;
    ticks = 0;
    rerun_persist();
}

/// @brief 维护"当前行"的指纹（每段流过时调，只在换行时更新,启动时能判断"这一行还是不是原来那一行）
void rerun_track_current_row(const PLAN_BLOCK* block)
{
    if (rerun_data == NULL || block == NULL) return;
    if (rt_data->current_mode != MODE_MEM) return;
    if (block->is_breakpoint) return;
    if (rerun_data->is_rerun) return;
    if (block->line_number == rerun_data->cur_row_line) return;   // 同一行的后续包，不动

    rerun_data->cur_row_line = block->line_number;
    memcpy(&rerun_data->cur_row_first_end, &block->end_pos, sizeof(POS));
    rerun_snapshot_row(block);
}

/// @brief 被打断时记一份断点快照
void rerun_store_breakpoint(const PLAN_BLOCK* block){
    if (rerun_data == NULL || block == NULL) return;
    if (rt_data->current_mode != MODE_MEM) return;
    if (block->is_breakpoint) return;

    rerun_data->break_block.valid       = true;
    rerun_data->break_block.line_number = block->line_number;
    rerun_data->break_block.mode        = block->mode;
    rerun_data->break_block.type        = block->type;
    rerun_data->break_block.m_code      = rerun_block_m_code(block);
    memcpy(&rerun_data->break_end, &rerun_data->cur_row_first_end, sizeof(POS));

    rerun_data->breakpoint.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    rerun_data->breakpoint.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    rerun_data->breakpoint.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;

    rerun_data->is_mem_rerun = 1;   // 这一轮记忆运行被打断过（切模式武装的判据）
    rerun_persist();
}

/// @brief 标记这次停止是被打断的（急停 / 报警停 / 切模式停调用）
void rerun_mark_interrupted(void)
{
    rerun_interrupted = true;
}

/// @brief rerun_interrupted被标记过就记一份断点快照
void rerun_record_if_interrupted(const PLAN_BLOCK* block)
{
    if (!rerun_interrupted) return;
    rerun_interrupted = false;
    rerun_store_breakpoint(block);
}
/// @brief 断点页"载入"的武装请求：页面把 RERUN_SAVE.rerun_row 设成断点记录行
static bool rerun_take_ui_arm_request(void)
{
    if (rerun_data == NULL || rerun_data->is_rerun)
        return false;

    RERUN_SAVE save = {0};
    if (getRerunSave(&save) != 0)
        return false;
    if (!save.break_block.valid)
        return false;
    if (save.rerun_row <= 0 || save.rerun_row != save.break_block.line_number)
        return false;

    char path[NCFILEPATH_CHAR_LEN] = {0};
    getNcFilePath(path);
    if (path[0] == '\0'
        || strncmp(path, save.parser_file_path, NCFILEPATH_CHAR_LEN) != 0)
        return false;

    rerun_data->is_rerun = 1;
    rerun_data->is_mem_rerun = 1;
    rerun_data->rerun_row = save.rerun_row;
    rerun_data->is_first_time_click_rerun = true;
    LOG(LOG_LEVEL_INFO, "断点页载入：武装再启动，目标行 %d", save.rerun_row);
    return true;
}

/// @brief 切换模式
void rerun_update_when_switching(){
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    rerun_mst_reset();  //本次再启动重新收集辅助码
    /* 先收武装请求：切出 MEM 时 RAM 里的 rerun_row 还是 0，直接持久化会覆盖掉请求。 */
    (void)rerun_take_ui_arm_request();
    switch (rt_data->target_mode)
    {
    case MODE_MDI:
        rerun_data->is_rerun = 0;
        rerun_data->rerun_row = 0;
        break;
    case MODE_MEM:
        /* 用记录行重写 rerun_row；take 的值与它相等，若以后允许页面指定不同行需保留 take 结果。 */
        rerun_data->is_rerun = rerun_data->is_mem_rerun && rerun_data->break_block.valid;
        rerun_data->rerun_row = rerun_data->is_rerun ? rerun_data->break_block.line_number : 0;
        rerun_data->is_first_time_click_rerun = true;
        break;
    default:
        break;
    }
    rerun_persist_force();    // 锁存结果变了 → 落一次盘
}

/// @brief 同步当前程序路径到 rerun 记录：切换程序时重置断点
/// @note  从指定行启动时 auto 侧也会调用本函数，保证后续断点记录的路径是最新的
void rerun_sync_program_path(){
    if (rerun_data == NULL) return;
    char parser_file_path[NCFILEPATH_CHAR_LEN] = {0};
    getNcFilePath(parser_file_path);
    parser_file_path[NCFILEPATH_CHAR_LEN - 1] = '\0';
    if(parser_file_path[0] != '\0' && strcmp(parser_file_path, rerun_data->parser_file_path)){
        rerun_data_reset();
        strncpy(rerun_data->parser_file_path, parser_file_path,
                sizeof(rerun_data->parser_file_path) - 1);
        rerun_data->parser_file_path[sizeof(rerun_data->parser_file_path) - 1] = '\0';
    }
}

/// @brief 判断是否程序再启动
bool check_is_program_rerun(){
    rerun_sync_program_path();
    /* 断点模式下程序页不发起始行请求，启动会落到这里；手动选行时起始行请求优先，武装由 rerun_reset_is_rerun() 清掉。 */
    (void)rerun_take_ui_arm_request();
    // 程序再启动时提示用户风险
    if (rerun_data->is_rerun){
        if (rerun_data->is_first_time_click_rerun) {
            LOG(LOG_LEVEL_WARN, "从程序中间开始，请确保刀具安全后按开始继续");
            /* 报警号 3 与"从指定行启动"的首按确认共用，两条路径互斥。 */
            setAlarm(ALARM_CH, ALARM_WARNING, 3);

            setVarChanInt(0, VAR_CHAN_RUN_ROW, rerun_data->rerun_row, 0);
            rerun_data->is_first_time_click_rerun = false;
            return true;
        }
    }
    delAlarm(ALARM_CH, ALARM_WARNING, 3);
    rerun_data->is_first_time_click_rerun = true;
    return false;
}

/// @brief 切换程序
void rerun_reset_is_rerun(){
    rerun_data->is_rerun = 0;
    rerun_data->rerun_row = 0;
    rerun_data_clear_breakpoint();

}

/// @brief 记下程序起点
void rerun_store_program_start(void)
{
    if(rerun_data == NULL) return;
    if( rerun_data->is_rerun || rt_data->current_mode != MODE_MEM)
    {
        return;
    }
    rerun_data->program_start.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    rerun_data->program_start.y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    rerun_data->program_start.z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
    rerun_data->program_start_valid = true;
    LOG(LOG_LEVEL_DEBUG, "程序起点 %f %f %f",
        rerun_data->program_start.x, rerun_data->program_start.y, rerun_data->program_start.z);
    rerun_persist();
}

/// @brief 显式构造构建回断点的定位段
void create_rerun_block(const PLAN_BLOCK* target, PLAN_BLOCK* rerun_block)
{
    memset(rerun_block, 0, sizeof(*rerun_block));
    rerun_block->mode       = PMC_MODE_CHOP;
    rerun_block->type       = RAPID;
    rerun_block->line_number = rerun_data->rerun_row;
    // 目标点 = 记录的断点位置
    // 断点块不是阻止缓冲的停止块，清掉可能继承的调度 ID 
    PLAN_BLOCK_BUF_STOP_ID(rerun_block) = 0;
    memcpy(&rerun_block->end_pos, &rerun_data->breakpoint, sizeof(POS));
    // 以空运行速度移动到断点位置
    rerun_block->feed_vel   = rt_data->chan_data[0].dryrun_spd;
    rerun_block->is_breakpoint = true;
    if (target != NULL) rerun_block->state = target->state;

    LOG(LOG_LEVEL_DEBUG, "断点程序 %s, 行号 %d", rerun_data->parser_file_path, rerun_block->line_number);
    LOG(LOG_LEVEL_DEBUG, "刀具位置 %f %f %f", rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur);
    LOG(LOG_LEVEL_DEBUG, "断点位置 %f %f %f", rerun_block->end_pos.x, rerun_block->end_pos.y, rerun_block->end_pos.z);
}

/// @brief 检查断点是否可再启动。
bool check_rerun_row_is_valid(const PLAN_BLOCK* target){
    int res = 0;
    if (target == NULL) {
        res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
        if(res != -8) LOG(LOG_LEVEL_ERR, "目标段为空，不可以执行程序再启动");
        return false;
    }
    switch (target->mode)
    {
    // 在下列方式在的程序段不可以执行程序再启动
    case PMC_MODE_TAP:
    case PMC_MODE_TAP_RETURE:
        res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
        if(res != -8) LOG(LOG_LEVEL_ERR, "攻丝不可以执行程序再启动");
        return false;
    case PMC_MODE_CHOP:
    case PMC_MODE_EXACT_STOP:
    case PMC_MODE_MEASURE_TOOL:
    case PMC_MODE_ARRIVE_CHECK:
        switch (target->type)
        {
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
            if(res != -8) LOG(LOG_LEVEL_ERR, "圆弧螺旋不可以执行程序再启动");
            return false;
        case RIGID_TAP:
        case TAP_RETURN:
            res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
            if(res != -8) LOG(LOG_LEVEL_ERR, "攻丝不可以执行程序再启动");
            return false;
        case SPLINE:
            res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
            if(res != -8) LOG(LOG_LEVEL_ERR, "样条不可以执行程序再启动");
            return false;
        case NURBS:
            res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
            if(res != -8) LOG(LOG_LEVEL_ERR, "NURBS样条不可以执行程序再启动");
            return false;
        default:
            break;
        }
        break;
    case PMC_MODE_M_CMD:
        switch (target->mst_data.m_code)
        {
        case M06:
        case M19:
            res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
            if(res != -8) LOG(LOG_LEVEL_ERR, "主轴定向/刀具管理不可以执行程序再启动");
            return false;
        default:
            break;
        }
        break;
    case PMC_MODE_STOPSKIP:
        res = setAlarm(ALARM_CH, ALARM_ERROR, 4);
        if(res != -8) LOG(LOG_LEVEL_ERR, "跳过不可以执行程序再启动");
        return false;
    default:
        break;
    }
    // LOG(LOG_LEVEL_DEBUG, "在该程序段可以执行程序再启动");
    delAlarm(ALARM_CH, ALARM_ERROR, 4);
    return true;
}

/// @brief 检出同一份程序是否被改过
bool rerun_program_changed(const PLAN_BLOCK* target)
{
    if (rerun_data == NULL || target == NULL) return false;

    if(rerun_data->cur_row_line <0) return false; 

    const bool same = IsEqualTo(target->end_pos.x, rerun_data->break_end.x)
                   && IsEqualTo(target->end_pos.y, rerun_data->break_end.y)
                   && IsEqualTo(target->end_pos.z, rerun_data->break_end.z);
    if (same) {
        return false;
    }
    LOG(LOG_LEVEL_WARN,
        "程序已被修改：断点行 %d 的终点 记录(%.3f, %.3f, %.3f) ≠ 本次扫到(%.3f, %.3f, %.3f) "
        "—— 本次仍按断点再启动，请确认后继续",
        rerun_data->rerun_row,
        rerun_data->break_end.x, rerun_data->break_end.y, rerun_data->break_end.z,
        target->end_pos.x, target->end_pos.y, target->end_pos.z);
    
    return true;
}

/* ======================= 断电保存 ======================= */

void rerun_save_pack(RERUN_SAVE* out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (rerun_data == NULL) return;

    memcpy(out->parser_file_path, rerun_data->parser_file_path, sizeof(out->parser_file_path));
    out->program_start_valid = rerun_data->program_start_valid ? 1 : 0;
    memcpy(&out->program_start, &rerun_data->program_start, sizeof(POS));
    memcpy(&out->breakpoint,    &rerun_data->breakpoint,    sizeof(POS));
    out->rerun_row           = rerun_data->rerun_row;
    out->break_block.valid       = rerun_data->break_block.valid ? 1 : 0;
    out->break_block.line_number = rerun_data->break_block.line_number;
    out->break_block.mode        = (int)rerun_data->break_block.mode;
    out->break_block.type        = (int)rerun_data->break_block.type;
    out->break_block.m_code      = rerun_data->break_block.m_code;
    memcpy(&out->break_end,     &rerun_data->break_end,     sizeof(POS));
    out->is_mem_rerun        = rerun_data->is_mem_rerun ? 1 : 0;
}

/// 坐标是否是有限值
static bool rerun_pos_is_finite(const RERUN_POS* p)
{
    return isfinite(p->x) && isfinite(p->y) && isfinite(p->z);
}


static bool rerun_save_is_sane(const RERUN_SAVE* in)
{
    if (!rerun_pos_is_finite(&in->program_start) ||
        !rerun_pos_is_finite(&in->breakpoint) ||
        !rerun_pos_is_finite(&in->break_end)) return false;
    if (in->rerun_row < 0) return false;
    if (in->break_block.line_number < 0) return false;
    if (in->break_block.m_code < -1) return false;
    if (in->break_block.mode < 0 || in->break_block.mode > (int)PMC_MODE_NULL) return false;
    /* type 只用于留档/显示；NURBS 是快照可能出现的真实类型，不能按 PROBE 判废整条记录。 */
    if (in->break_block.type < 0 || in->break_block.type > (int)NURBS) return false;
    return true;
}

bool rerun_save_unpack(const RERUN_SAVE* in)
{
    if (in == NULL || rerun_data == NULL) return false;
    // 自检放在**动 RAM 之前** —— 不认的记录一个字段都不落地（用例钉住这一点）
    if (!rerun_save_is_sane(in)) {
        LOG(LOG_LEVEL_WARN, "断电块里的记录不可用（字段越界或坐标非有限），按无记录启动");
        return false;
    }
    // "一件事都没有"就当没内容（既没被打断过、也没记过程序起点）
    if (!in->is_mem_rerun && !in->program_start_valid) return false;

    memcpy(rerun_data->parser_file_path, in->parser_file_path, sizeof(rerun_data->parser_file_path));
    memcpy(&rerun_data->program_start, &in->program_start, sizeof(POS));

    rerun_data->program_start_valid = (in->program_start_valid != 0);
    memcpy(&rerun_data->breakpoint, &in->breakpoint, sizeof(POS));
    rerun_data->break_block.valid       = (in->break_block.valid != 0);
    rerun_data->break_block.line_number = in->break_block.line_number;
    rerun_data->break_block.mode        = (pmc_mode)in->break_block.mode;      // 还原（已自检）
    rerun_data->break_block.type        = (INTERP_TYPE)in->break_block.type;
    rerun_data->break_block.m_code      = in->break_block.m_code;
    memcpy(&rerun_data->break_end, &in->break_end, sizeof(POS));

    rerun_data->is_mem_rerun        = (in->is_mem_rerun != 0);

    rerun_data->rerun_row = 0;//is_rerun还没设置，所以行号从0，需要页面设置
    return true;
}

static RERUN_SAVE s_last_saved;
static bool s_last_saved_valid = false;

static void rerun_persist_ex(bool force)
{
    if (rerun_data == NULL) return;

    RERUN_SAVE save;
    rerun_save_pack(&save);


    if (!force && s_last_saved_valid && memcmp(&s_last_saved, &save, sizeof(save)) == 0)
        return;
    if (setRerunSave(&save) != 0)
    {
        LOG(LOG_LEVEL_WARN, "程序断电块写入失败");
        return;
    }
    s_last_saved = save;
    s_last_saved_valid = true;// 防止程序暂停每周器写一次存储块
}

void rerun_persist(void)
{
    rerun_persist_ex(false);
}

static void rerun_persist_force(void)
{
    rerun_persist_ex(true);
}

void rerun_restore(void)
{
    RERUN_SAVE saved={0};

    if (getRerunSave(&saved) != 0) 
    {
        LOG(LOG_LEVEL_WARN, "断点记录读取失败");

        return;
    }

    if (!rerun_save_unpack(&saved)) {
        LOG(LOG_LEVEL_WARN, "没有可用的断点记录");
        return;
    }

    LOG(LOG_LEVEL_INFO,
        "程序断点记录: 目标行=%d 断点=(%.3f,%.3f,%.3f) 程序起点=%s",
        rerun_data->break_block.line_number, rerun_data->breakpoint.x, rerun_data->breakpoint.y,
        rerun_data->breakpoint.z, rerun_data->program_start_valid ? "有" : "无");
}
