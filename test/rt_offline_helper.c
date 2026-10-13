/* 离线验证的辅助端：替代 Qt 界面，把一个完整启动流程需要的东西给上。
 *
 * 干什么：
 *   1. shmInit() —— 连上共享内存（必须 root，共享内存库自己的策略）
 *   2. 可选地往参数域写一组台架参数（--params）—— 这是 "参数域 → rt_hal_config →
 *      HAL 配置" 那条链的输入端，离线没有 HMI 时它是空的，RT 会正确地退回占位值
 *   3. 置 SYS_START_RT，保持 N 秒，再置 SYS_CLOSE_RT
 *
 * 注意它不是产品的一部分，只是离线冒烟用的脚手架。
 *
 * 用法：  rt_offline_helper <保持秒数> [--params] [--reset] [--spindle] [--estop-mid-reset]
 *                                          [--drive-err] [--drive-err-feed]
 *
 * --spindle 跑一段主轴回归：M3 -> M5 -> M19 -> 释放 -> 再 M19。
 * --cold-orient: simulation-only M19 from rest, without a preceding M3.
 * 走的是真路径：往共享内存的 G 寄存器写边沿，PLC 扫描器翻译成主轴事件，
 * 再从 F 寄存器读回主轴自己的判断（F45.1 转速、F45.3 到速、F45.7 定向完成）。
 * 不接界面、不接硬件，纯粹验"移植后的主轴逻辑 + HAL"这条链。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shmapi.h"
#include "sysapi.h"
#include "parapi.h"
#include "pardata.h"
#include "gncdatadef.h"
#include "operatesignalapi.h"
#include "operatesignaldata.h"
#include "regapi.h"
#include "regdata.h"
#include "varapi.h"
#include <stdbool.h>

/* RT 的参数域索引：X=0, Y=1, Z=2, SP=3（src/datadef/datatype.h 的 AXIS_TYPE）。
 * 这里不引 RT 内部头，直接写数并注明来源。 */
#define RT_PAR_AXIS_X   0
#define RT_PAR_AXIS_Y   1
#define RT_PAR_AXIS_Z   2
#define RT_PAR_AXIS_SP  3

/* 台架参数（**自洽**是这里唯一的讲究）。
 *
 * 进给：每转 10mm、每转 10000 个计数 → 当量 = 10/10000 = 0.001 mm/count，
 *       反过来 10000 × 0.001 = 10mm ✓
 * 主轴：每转 360°、每转 100000 个计数 → 当量 = 360/100000 = 0.0036 deg/count，
 *       反过来 100000 × 0.0036 = 360° ✓
 *
 * 为什么强调自洽：示例配置里那组 100000 count/rev × 0.16 deg/count = 16000°，
 * 而一转应当是 360° —— 那组数是自相矛盾的（见计划书 §4.3）。这里取一组乘起来
 * 正好等于一转的量，免得把同样的毛病带进验证。
 *
 * pm_munit 的单位是 um，装入 rt_data 时由 CONV_UM_TO_MM 除以 1000 变成 mm。 */
static void set_axis(ParData* pd, int axis, int munit_um, int pulse, double enc_off, int mt_ppr) {
    pd->par[0].par_axis[axis][PAR_AX_PM_MUNIT].value.val_int  = munit_um;
    pd->par[0].par_axis[axis][PAR_AX_PM_PULSE].value.val_int  = pulse;
    pd->par[0].par_axis[axis][PAR_AX_ENC_OFF].value.val_real  = enc_off;
    /* 加减速时间常数（ms）。**不写就是 0**，CONV_T_MS_TO_ACCEL(0) = 0，
     * 主轴速度规划器的 diff 也是 0 —— 于是 M3 收下了、状态也变了，就是转速永远不动。
     * 这个坑和"参数域为空"表现一样，靠日志根本看不出来。给 10ms 让它快速到位。 */
    pd->par[0].par_axis[axis][PAR_AX_FEED_ACC].value.val_real  = 10.0;
    pd->par[0].par_axis[axis][PAR_AX_RAPID_ACC].value.val_real = 10.0;
    /* 软限位。**不写就是 0**，而坐标是 -enc_off = -150，于是存储行程检查一路报警；
     * 报警置 SYS_IS_ALARMING 后，spindle_stop_is_required() 每拍把主轴事件改写成
     * STOP —— M3/M19 看着"发了但没反应"，其实是被报警吃掉了。给一组够宽的量程。 */
    pd->par[0].par_axis[axis][PAR_AX_PLMT].value.val_real    = 10000.0;
    pd->par[0].par_axis[axis][PAR_AX_NLMT].value.val_real    = -10000.0;
    pd->par[0].par_axis[axis][PAR_AX_PLMT2].value.val_real   = 10000.0;
    pd->par[0].par_axis[axis][PAR_AX_NLMT2].value.val_real   = -10000.0;
    /* 电机每转脉冲数（HAL 做模循环展开用的模）。**故意取与 pm_pulse 不同的值**，
     * 这样跑出来一眼就能看出用的是哪个——两个相等的话，"优先级对不对"就验不出来。 */
    pd->par[0].par_axis[axis][PAR_AX_MT_PPR].value.val_int    = mt_ppr;
}

static void write_bench_params(void) {
    ParData* pd = getParDataPtr();
    if (!pd) { printf("参数域指针为空，跳过写参数\n"); return; }

    set_axis(pd, RT_PAR_AXIS_X,  10000, 10000, 150.0, 20000);
    set_axis(pd, RT_PAR_AXIS_Y,  10000, 10000, 150.0, 20000);
    set_axis(pd, RT_PAR_AXIS_Z,  10000, 10000, 150.0, 20000);
    set_axis(pd, RT_PAR_AXIS_SP, 360000, 100000, 0.0, 120000);
    printf("已写入台架参数：进给 每转10mm/10000指令计数（当量 0.001 mm/count），"
           "主轴 每转360°/100000指令计数（当量 0.0036 deg/count）\n");
    printf("  电机每转脉冲数 MT_PPR 故意与指令计数不同：进给 20000、主轴 120000\n");
}

/* ---- 主轴回归用的参数：缺省 S 与最大转速 ----
 * 这两个不写，M3 会算出 0 目标速度（S=0 等价 M5），跑起来看着像"没反应"，
 * 其实是参数域是空的。 */
#define SPL_TEST_SPEED   500.0
#define SPL_TEST_MAX     3000.0

/* 缺省 S 可被 --s <rpm> 覆盖：用来验"参数域里的真实上限真的接到 HAL 了"——
 * 上限曾被写死 1000 rpm，编 S1500 会被 HAL 拒收、主轴不转且拿不到 FIN。 */
static double spl_test_speed = SPL_TEST_SPEED;

static void write_spindle_params(void) {
    ParData* pd = getParDataPtr();
    if (!pd) { printf("参数域指针为空，跳过写主轴参数\n"); return; }
    pd->par[0].par_axis[RT_PAR_AXIS_SP][PAR_AX_DEFAULT_S].value.val_real   = spl_test_speed;
    pd->par[0].par_axis[RT_PAR_AXIS_SP][PAR_SPDL_MAX_SPEED].value.val_real = SPL_TEST_MAX;
    printf("已写主轴参数：缺省 S=%.0f，最大转速=%.0f\n", spl_test_speed, SPL_TEST_MAX);
}

/* ---- 主轴回归 ---- */

static int spl_failed;

static int read_f(int idx, int bit);

static int read_f(int idx, int bit) {
    bool v = false;
    if (getRegBit(REG_F, idx, bit, &v) != 0) return -1;
    return v ? 1 : 0;
}

static void expect(const char* what, int got, int want) {
    const int ok = (got == want);
    printf("[SPL] %-34s got=%d want=%d  %s\n", what, got, want, ok ? "PASS" : "FAIL");
    if (!ok) ++spl_failed;
}

/* 主轴修调必须给 100%：CNC 速度源下 target = S * override，override 默认 0 会让
 * M3 目标速度恒为 0——这个坑和"参数域为空"表现一模一样。 */
static void spl_set_override(void) {
    double back = -1.0;
    setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, 100.0, 0);
    getVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, &back, 0);
    printf("已把主轴修调置 100%%，读回 %.1f\n", back);
}

/* 倍率和 S 码都读回来打出来：M3 不动的时候，先分清是"目标速度算成 0"还是
 * "算出来了但没下发"。 */
static void spl_probe(const char* when) {
    double ov = -1.0;
    int s_code = -1;
    getVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, &ov, 0);
    getVarChanInt(0, VAR_CHAN_S_CODE, &s_code, 0);
    printf("[SPL] %s: 倍率=%.1f%% S码=%d F45.1=%d F45.3=%d F45.7=%d\n",
           when, ov, s_code, read_f(45, 1), read_f(45, 3), read_f(45, 7));
}

static int run_spindle_sequence(void) {

    printf("-> 释放急停（G8.4 0->1，应触发自动复位）\n"); fflush(stdout);
    setRegBit(REG_G, 8, 4);
    sleep(3);

    spl_probe("M3 前");
    printf("-> M3（G70.5 上升沿）\n"); fflush(stdout);
    setRegBit(REG_G, 70, 5);
    sleep(4);
    spl_probe("M3 后");
    expect("M3 后 F45.3（到速）", read_f(45, 3), 1);
    /* F45.1 的语义是"转速 < 5"（置位=已停），不是"在转"，别读反。 */
    expect("M3 后 F45.1（转速<5，应为0）", read_f(45, 1), 0);

    printf("-> M5（G29.6 置 1 再清 0，制造下降沿）\n"); fflush(stdout);
    setRegBit(REG_G, 29, 6);
    sleep(1);
    clearRegBit(REG_G, 29, 6);
    sleep(4);
    expect("M5 后 F45.1（转速<5，应为1）", read_f(45, 1), 1);
    /* 停止完成（F45.3）必须由主轴控制层在确认零速后置位：PLC 发布层已改成
     * 只读查询，这条断言端到端保证“删掉发布层反向写状态”之后停止完成照样发布。 */
    expect("M5 后 F45.3（停止完成）", read_f(45, 3), 1);

    printf("-> M19（G70.6 上升沿）\n"); fflush(stdout);
    setRegBit(REG_G, 70, 6);
    sleep(12);   /* 超过定向超时 10s，受理失败时能打出超时日志 */
    expect("M19 后 F45.7（定向完成）", read_f(45, 7), 1);

    printf("-> 释放定向（G70.6 下降沿）\n"); fflush(stdout);
    clearRegBit(REG_G, 70, 6);
    sleep(4);
    expect("释放后 F45.7 落下去", read_f(45, 7), 0);

    printf("-> 再来一次 M19（重复执行）\n"); fflush(stdout);
    setRegBit(REG_G, 70, 6);
    sleep(12);
    expect("重复 M19 后 F45.7", read_f(45, 7), 1);
    clearRegBit(REG_G, 70, 6);
    sleep(2);

    printf("[SPL] 汇总：失败 %d 项\n", spl_failed); fflush(stdout);
    return spl_failed;
}

/* Simulation only: cold M19, release, then M19 again without an M3. */
static int run_cold_orient_sequence(void) {
    setRegBit(REG_G, 8, 4);
    sleep(3);
    expect("cold: stopped", read_f(45, 1), 1);
    expect("cold: no orientation", read_f(45, 7), 0);
    for (int i = 0; i < 2; ++i) {
        printf("[SPL] cold M19 attempt %d\n", i + 1);
        fflush(stdout);
        setRegBit(REG_G, 70, 6);
        sleep(12);
        expect("cold M19 completed", read_f(45, 7), 1);
        clearRegBit(REG_G, 70, 6);
        sleep(2);
        expect("cold M19 released", read_f(45, 7), 0);
    }
    return spl_failed;
}

int main(int argc, char** argv) {
    int hold = 5;
    int with_params = 0, with_reset = 0, with_spindle = 0, with_estop_mid_reset = 0;
    int with_drive_err = 0, with_drive_err_feed = 0;
    int with_cold_orient = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--params")) with_params = 1;
        else if (!strcmp(argv[i], "--reset")) with_reset = 1;
        else if (!strcmp(argv[i], "--spindle")) { with_spindle = 1; with_params = 1; }
        else if (!strcmp(argv[i], "--cold-orient")) { with_cold_orient = 1; with_params = 1; }
        else if (!strcmp(argv[i], "--estop-mid-reset")) with_estop_mid_reset = 1;
        else if (!strcmp(argv[i], "--s") && i + 1 < argc) spl_test_speed = atof(argv[++i]);
        else if (!strcmp(argv[i], "--drive-err")) { with_drive_err = 1; with_params = 1; }
        else if (!strcmp(argv[i], "--drive-err-feed")) { with_drive_err_feed = 1; with_params = 1; }
        else hold = atoi(argv[i]);
    }

    if (!shmInit()) { printf("shmInit 失败\n"); return 1; }
    printf("shmInit ok\n"); fflush(stdout);

    if (with_params) write_bench_params();
    if (with_spindle || with_cold_orient) write_spindle_params();

    printf("-> 置 SYS_START_RT\n"); fflush(stdout);
    setSysCommInt(SYS_START_RT, 1, 0);

    if (with_spindle || with_cold_orient) {
        sleep(3);              /* 等 RT 起完 HAL 与各线程 */
        spl_set_override();
        if (with_cold_orient) run_cold_orient_sequence();
        else run_spindle_sequence();
        printf("-> 置 SYS_CLOSE_RT\n"); fflush(stdout);
        setSysCommInt(SYS_CLOSE_RT, 1, 0);
        sleep(3);
        return spl_failed == 0 ? 0 : 1;
    }

    if (with_drive_err || with_drive_err_feed) {
        const int feed_only = with_drive_err_feed;
        /* 驱动器故障码（0x603F）→ is_error → 这两个 F 位。
         * 走的是真路径：仿真桩报出 0x603F → alarm_poll_drive_errors() 置 is_error
         * → plc_f_signal_handler 输出 F45.0 / F0.6，再从共享内存读回来。 */
        sleep(3);
        printf("-> 读驱动器报警信号（F45.0 主轴报警 / F0.6 轴无报警）\n"); fflush(stdout);
        if (feed_only) {
            /* 反向对照：只坏 X 轴时，主轴报警位必须是 0 —— 否则说明 F45.0 是
             * "有报警就置起"，而不是真按轴映射的。 */
            expect("F45.0 主轴报警（坏的只是 X 轴，应为 0）", read_f(45, 0), 0);
            expect("F0.6 轴无报警（有轴报错应落下）", read_f(0, 6), 0);
        } else {
            expect("F45.0 主轴报警（主轴注入了 0x603F）", read_f(45, 0), 1);
            expect("F0.6 轴无报警（有轴报错应落下）", read_f(0, 6), 0);
        }
        printf("[SPL] 汇总：失败 %d 项\n", spl_failed); fflush(stdout);
        printf("-> 置 SYS_CLOSE_RT\n"); fflush(stdout);
        setSysCommInt(SYS_CLOSE_RT, 1, 0);
        sleep(3);
        return spl_failed == 0 ? 0 : 1;
    }

    if (with_reset) {
        /* 释放急停。
         *
         * 走的是**真正活着的那条路径**：G8.4 由梯形图产出，plc_g_signal_scanner 检测
         * 它的 0→1 上升沿后会自动"解除急停 + 推一个复位事件"
         * （plc_g_signal_scanner.c 的 SIGNAL_RISING 分支 → plc_dispatch_event(PLC_EV_MSG_RESET)），
         * 那个事件经 task_handle_reset_event 变成 SYS_EVENT_HMI_RESET → handle_alarm_cmd。
         *
         * 注意别走 X8.4：plc_x_signal_scan_cycle() 在运动线程里是注释掉的，那条路当前
         * 根本没被调用。一开始我改的就是 X 侧，白改。
         *
         * 起点 G8.4 = 0 表示急停按下（寄存器初值就是 0），所以置 1 就构成上升沿。 */
        sleep(3);
        if (setRegBit(REG_G, 8, 4) != 0)
            printf("置 G8.4 失败\n");
        else
            printf("-> 已释放急停（G8.4 0→1，应触发自动复位）\n");
        fflush(stdout);

        if (with_estop_mid_reset) {
            /* 复位还在飞时把急停按回去：G8.4 = 0 就是"拍下"
             * （plc_g_signal_scanner.c:84-89 把它翻成 SYS_ESTOP_INPUT）。
             * 需要配 CNC_RT_SIM_FAULT_STUCK=1，否则复位几拍就收口、按不住窗口。 */
            sleep(2);
            if (clearRegBit(REG_G, 8, 4) != 0)
                printf("清 G8.4 失败\n");
            else
                printf("-> 复位在飞时按下急停（G8.4 1→0），并保持按住\n");
            fflush(stdout);
        }
    }

    sleep(hold);

    printf("-> 置 SYS_CLOSE_RT\n"); fflush(stdout);
    setSysCommInt(SYS_CLOSE_RT, 1, 0);
    sleep(3);
    return 0;
}
