#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <hmisysapi.h>

/* M29 FIN 时序自查：直接驱动生产代码 auto_mst.c 的辅助功能握手，不接共享内存/总线，
 * 覆盖"双条件"里 PLC FIN 一侧的时序。LOG 抓最后一行以锁住诊断日志字段。 */
#undef LOG
static char log_line[1024];
static char motion_log_line[1024];
static void log_capture(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char *dest = strncmp(fmt, "M29运动及PLC信号", strlen("M29运动及PLC信号")) == 0
        ? motion_log_line : log_line;
    vsnprintf(dest, sizeof(log_line), fmt, ap);
    va_end(ap);
}
#define LOG(level, fmt, ...) log_capture(fmt, ##__VA_ARGS__)
#include "../src/motion/auto/trajplan/auto_mst.c"

/* ---------------- 全局桩 ---------------- */
spindle_context_t spl_context;
COORD_TP *cycle_tp;
static COORD_TP cycle_store;
static PLAN_BLOCK plan_store;
static ParamUnit par_store[1][PARAMAN_FILE_CHAN_MAX];
ParamUnit (*par_chan)[PARAMAN_FILE_CHAN_MAX];
static VarUnion var_store[1][VAR_CHAN_NUM];
VarUnion (*var_chan)[VAR_CHAN_NUM];

static bool fin_high;        /* G4.3 */
static bool mf_strobe;       /* F7.0 当前指令选通 */
static bool m29_ready;       /* 双条件的 NC 一侧 */
static unsigned complete_calls;
static unsigned alarm_calls;

bool spindle_m29_is_ready(void) { return m29_ready; }
void spindle_complete_m29(void) { ++complete_calls; }
/* 卡住诊断只读快照：本自查不校验内容 */
void spindle_m29_diag_info(spindle_m29_diag_t *out) { memset(out, 0, sizeof(*out)); }

int getRegValue8(RegType type, int idx, char *value)
{ (void)type; (void)idx; *value = 0; return 0; }

int getRegBit(RegType type, int idx, int bit, bool *value)
{
    if (type == REG_G && idx == 4 && bit == 3) { *value = fin_high; return 0; }
    if (type == REG_F && idx == 7 && bit == 0) { *value = mf_strobe; return 0; }
    *value = false;
    return 0;
}
int setRegBit(RegType type, int idx, int bit)
{ (void)type; (void)idx; (void)bit; return 0; }
int clearRegBit(RegType type, int idx, int bit)
{ (void)type; (void)idx; (void)bit; return 0; }
int setRegValue8(RegType type, int idx, char value)
{ (void)type; (void)idx; (void)value; return 0; }
int setVarChanInt(int no, int index, int value, int clientNo)
{ (void)no; (void)index; (void)value; (void)clientNo; return 0; }
int setAlarm(int type, int level, int id)
{ (void)type; (void)level; (void)id; ++alarm_calls; return 0; }
int delAlarm(int type, int level, int id)
{ (void)type; (void)level; (void)id; return 0; }

/* ---------------- 夹具 ---------------- */
static void prepare(int m_code)
{
    memset(&cycle_store, 0, sizeof(cycle_store));
    memset(&plan_store, 0, sizeof(plan_store));
    memset(&spl_context, 0, sizeof(spl_context));
    memset(par_store, 0, sizeof(par_store));
    memset(var_store, 0, sizeof(var_store));
    par_chan = par_store;
    var_chan = var_store;
    cycle_tp = &cycle_store;
    cycle_tp->plan_block = &plan_store;
    cycle_tp->plan_block->mst_data.m_code = m_code;
    /* 已过 MF 选通延时：选通脉冲属当前指令 */
    cycle_tp->mstb_cmd_data.signal_den = false;
    cycle_tp->mstb_cmd_data.signal_mf = true;
    par_chan[0][PAR_CH_TFIN_DELEY].value.val_int = 2;   /* FIN 宽度 2 拍 */
    spl_context.m29_active = true;
    spl_context.m29_phase = SPINDLE_M29_READY;
    fin_high = false;
    mf_strobe = true;
    m29_ready = true;
    complete_calls = 0;
    alarm_calls = 0;
}

static void run(unsigned cycles)
{
    for (unsigned i = 0; i < cycles; ++i) auto_execute_mstb_cmd();
}

typedef struct { bool latched; bool low_seen; int tfin; } fin_state_t;
static fin_state_t snapshot(void)
{
    fin_state_t st;
    st.latched = cycle_tp->mstb_cmd_data.m29_fin_latched;
    st.low_seen = cycle_tp->mstb_cmd_data.m29_fin_low_seen;
    st.tfin = cycle_tp->mstb_cmd_data.tfin_count;
    return st;
}

/* 1) 旧高电平 FIN（当前指令内从没见过低电平）不能锁存。 */
static void test_stale_high_level_rejected(void)
{
    prepare(M29);
    fin_high = true;             /* 上一条指令遗留的高电平 */
    mf_strobe = true;
    run(20);
    assert(!snapshot().latched);
    assert(!cycle_tp->mstb_cmd_data.finished);
    assert(complete_calls == 0);
    puts("PASS: M29 拒绝旧高电平 FIN（未见过低电平不锁存）");
}

/* 2) 选通前的高电平、未选通 MF 的高电平都要被拒。 */
static void test_pre_strobe_high_level_rejected(void)
{
    /* 2a) F7.0 未选通 */
    prepare(M29);
    fin_high = false; run(1);            /* 先见低电平 */
    fin_high = true; mf_strobe = false;  /* F7.0 未选通当前指令 */
    run(20);
    assert(!snapshot().latched);
    assert(!snapshot().low_seen);

    /* 2b) FIN 在 MF 选通之前就拉高（signal_mf=false） */
    prepare(M29);
    fin_high = false; run(1);
    fin_high = true;
    cycle_tp->mstb_cmd_data.signal_mf = false;
    run(20);
    assert(!snapshot().latched);
    assert(!snapshot().low_seen);
    assert(!cycle_tp->mstb_cmd_data.finished);
    puts("PASS: M29 拒绝选通前的高电平 FIN");
}

/* 3) 宽度不足的短脉冲不算；低电平 + 选通 + 宽度达标才锁存。 */
static void test_width_required(void)
{
    prepare(M29);
    fin_high = false; run(1);            /* 见低电平 */
    fin_high = true;                     /* 脉冲开始 */
    run(2);                              /* 2 拍还不够 */
    assert(!snapshot().latched);
    run(1);                              /* 第 3 拍宽度达标 */
    assert(snapshot().latched);
    puts("PASS: M29 FIN 宽度门槛（短脉冲不锁存）");
}

/* 4) 双条件：PLC FIN 已锁存但 NC 未就绪 → 不推进；就绪后才推进一次。 */
static void test_dual_condition(void)
{
    prepare(M29);
    m29_ready = false;
    fin_high = false; run(1);
    fin_high = true;
    run(5);
    assert(snapshot().latched);          /* FIN 侧满足 */
    assert(!cycle_tp->mstb_cmd_data.finished);
    assert(complete_calls == 0);

    m29_ready = true;
    run(1);
    assert(cycle_tp->mstb_cmd_data.finished);
    assert(complete_calls == 1);
    puts("PASS: M29 双条件（FIN 锁存 + NC 就绪）才推进");
}

/* 5) 锁存属当前指令：M29 结束/出错要清，指令复位也要清。 */
static void test_latch_is_per_command(void)
{
    prepare(M29);
    m29_ready = false;                   /* 只锁存不完成，便于验证清除 */
    fin_high = false; run(1);
    fin_high = true;
    run(5);
    assert(snapshot().latched);
    assert(!cycle_tp->mstb_cmd_data.finished);

    /* 5a) M29 结束 */
    spl_context.m29_active = false;
    run(1);
    assert(!snapshot().latched && !snapshot().low_seen);
    assert(snapshot().tfin == 0);
    assert(!cycle_tp->mstb_cmd_data.finished);

    /* 5b) M29 出错 */
    prepare(M29);
    m29_ready = false;
    fin_high = false; run(1);
    fin_high = true;
    run(5);
    assert(snapshot().latched);
    spl_context.m29_phase = SPINDLE_M29_ERROR;
    run(1);
    assert(!snapshot().latched);

    /* 5c) 指令切换复位三个字段 */
    cycle_tp->mstb_cmd_data.m29_fin_latched = true;
    cycle_tp->mstb_cmd_data.m29_fin_low_seen = true;
    cycle_tp->mstb_cmd_data.tfin_count = 7;
    cycle_tp_mstb_cmd_data_reset();
    assert(!cycle_tp->mstb_cmd_data.m29_fin_latched);
    assert(!cycle_tp->mstb_cmd_data.m29_fin_low_seen);
    assert(cycle_tp->mstb_cmd_data.tfin_count == 0);
    puts("PASS: M29 FIN 锁存属当前指令（结束/出错/换指令都清）");
}

/* 7) 卡住诊断与超时报警：只观察，不影响握手判定；每 1000 拍一行、30s 报一次且不重复刷。 */
static void test_stuck_diag_and_timeout(void)
{
    prepare(M29);
    m29_ready = false;                   /* NC 侧未就绪 */
    fin_high = false; run(1);            /* 见低电平 */
    fin_high = true;
    assert(cycle_tp->mstb_cmd_data.m29_wait_cycles == 1);

    /* 到诊断周期：打第 1 行 */
    log_line[0] = '\0';
    run(M29_DIAG_PERIOD_CYCLES - 1);
    assert(cycle_tp->mstb_cmd_data.m29_wait_cycles == M29_DIAG_PERIOD_CYCLES);
    assert(strstr(log_line, "M29等待握手 1s") != NULL);
    assert(strstr(log_line, "FIN锁存=") != NULL);
    assert(strstr(log_line, "低电平见过=") != NULL);
    assert(strstr(log_line, "宽度=") != NULL);
    assert(strstr(log_line, "G4.3=") != NULL);
    assert(strstr(log_line, "F7.0=") != NULL);
    assert(strstr(log_line, "phase=") != NULL);
    assert(strstr(log_line, "ready=") != NULL);
    assert(strstr(log_line, "模式切换=") != NULL);
    assert(strstr(log_line, "实际模式=") != NULL);
    assert(strstr(log_line, "使能=") != NULL);
    assert(strstr(log_line, "状态字=0x") != NULL);
    assert(strstr(log_line, "F45.7=") != NULL);
    assert(strstr(log_line, "F76.3=") != NULL);
    printf("  诊断样例: %s\n", log_line);
    assert(strstr(motion_log_line, "target_rpm=") != NULL);
    assert(strstr(motion_log_line, "actual_rpm=") != NULL);
    assert(strstr(motion_log_line, "target_deg=") != NULL);
    assert(strstr(motion_log_line, "F45.1=") != NULL);
    assert(strstr(motion_log_line, "F45.3=") != NULL);
    assert(strstr(motion_log_line, "F10=") != NULL);
    assert(strstr(motion_log_line, "G70=") != NULL);
    assert(strstr(motion_log_line, "G29=") != NULL);
    printf("  运动及信号样例: %s\n", motion_log_line);

    /* 到 30s 才报一次 */
    run(M29_DIAG_TIMEOUT_CYCLES - M29_DIAG_PERIOD_CYCLES - 1);
    assert(cycle_tp->mstb_cmd_data.m29_wait_cycles == M29_DIAG_TIMEOUT_CYCLES - 1);
    assert(alarm_calls == 0);
    assert(!cycle_tp->mstb_cmd_data.finished);

    run(1);
    assert(alarm_calls == 1);
    assert(!cycle_tp->mstb_cmd_data.finished);   /* 只提示，不改控制流程 */

    run(2000);
    assert(alarm_calls == 1);                    /* 不重复刷 */

    m29_ready = true;                            /* NC 就绪 */
    run(1);
    assert(cycle_tp->mstb_cmd_data.finished);
    assert(cycle_tp->mstb_cmd_data.m29_wait_cycles == 0);
    assert(complete_calls == 1);
    puts("PASS: M29 等待诊断与超时报警（不影响握手判定）");
}

/* 6) 通用 FIN 路径不受影响：非 M29 指令只看 FIN 宽度即可推进。 */
static void test_generic_path_unchanged(void)
{
    prepare(M03);
    m29_ready = false;                   /* 与普通 M 指令无关 */
    fin_high = true;
    run(5);
    assert(cycle_tp->mstb_cmd_data.finished);
    assert(complete_calls == 0);         /* 不触发 M29 完成 */
    puts("PASS: 非 M29 指令仍走通用 FIN 宽度路径");
}

int main(void)
{
    test_stale_high_level_rejected();
    test_pre_strobe_high_level_rejected();
    test_width_required();
    test_dual_condition();
    test_latch_is_per_command();
    test_generic_path_unchanged();
    test_stuck_diag_and_timeout();
    puts("PASS: M29 FIN 时序全部通过");
    return 0;
}
