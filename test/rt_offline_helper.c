/* 离线验证辅助端（非产品，替代 Qt 界面）：shmInit 连共享内存（需 root），可选写台架参数
 * （--params = 参数域→rt_hal_config→HAL 配置链的输入端），置 SYS_START_RT 保持 N 秒后置
 * SYS_CLOSE_RT。--spindle/--cold-orient 走真路径：写 G 寄存器边沿→PLC 扫描器→读 F45.x 判定。 */

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

/* RT 参数域索引（src/datadef/datatype.h AXIS_TYPE）：X=0,Y=1,Z=2,SP=3；不引 RT 内部头 */
#define RT_PAR_AXIS_X   0
#define RT_PAR_AXIS_Y   1
#define RT_PAR_AXIS_Z   2
#define RT_PAR_AXIS_SP  3

/* 台架参数（唯一讲究：自洽 —— 每转位移 × 每转计数必须正好等于一转）。
 * 进给 10mm/转 ÷ 10000 计数 = 0.001 mm/count；主轴 360°/转 ÷ 100000 计数 = 0.0036 deg/count。
 * pm_munit 单位 um，装入 rt_data 时由 CONV_UM_TO_MM 除以 1000 变 mm。 */
static void set_axis(ParData* pd, int axis, int munit_um, int pulse, double enc_off, int mt_ppr) {
    pd->par[0].par_axis[axis][PAR_AX_PM_MUNIT].value.val_int  = munit_um;
    pd->par[0].par_axis[axis][PAR_AX_PM_PULSE].value.val_int  = pulse;
    pd->par[0].par_axis[axis][PAR_AX_ENC_OFF].value.val_real  = enc_off;
    /* 加减速时间常数（ms）：不写即 0，CONV_T_MS_TO_ACCEL(0)=0 会让转速永远不动；给 10ms。 */
    pd->par[0].par_axis[axis][PAR_AX_FEED_ACC].value.val_real  = 10.0;
    pd->par[0].par_axis[axis][PAR_AX_RAPID_ACC].value.val_real = 10.0;
    /* 软限位：不写即 0，而坐标 -enc_off=-150 会一路报行程超限（报警会吃掉 M3/M19）；给 ±10000。 */
    pd->par[0].par_axis[axis][PAR_AX_PLMT].value.val_real    = 10000.0;
    pd->par[0].par_axis[axis][PAR_AX_NLMT].value.val_real    = -10000.0;
    pd->par[0].par_axis[axis][PAR_AX_PLMT2].value.val_real   = 10000.0;
    pd->par[0].par_axis[axis][PAR_AX_NLMT2].value.val_real   = -10000.0;
    /* 电机每转脉冲数（HAL 做模循环展开用的模）：故意取与 pm_pulse 不同，便于区分优先级。 */
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

/* ---- 主轴回归参数：缺省 S 与最大转速（不写则 M3 目标速度为 0，等价 M5）---- */
#define SPL_TEST_SPEED   500.0
#define SPL_TEST_MAX     3000.0

/* 缺省 S 可被 --s <rpm> 覆盖：验参数域上限真的接到 HAL；超上限会被拒收、拿不到 FIN。 */
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

/* 主轴修调必须给 100%：target = S * override，override 默认 0 会让目标速度恒为 0。 */
static void spl_set_override(void) {
    double back = -1.0;
    setVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, 100.0, 0);
    getVarChanDouble(0, VAR_CHAN_SPDL_OVERRIDE, &back, 0);
    printf("已把主轴修调置 100%%，读回 %.1f\n", back);
}

/* 读回并打印倍率/S 码与三个 F 位：定位 M3 不动的原因。 */
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
    /* F45.1 语义 = 转速 < 5（置位=已停），不是"在转"，别读反。 */
    expect("M3 后 F45.1（转速<5，应为0）", read_f(45, 1), 0);

    printf("-> M5（G29.6 置 1 再清 0，制造下降沿）\n"); fflush(stdout);
    setRegBit(REG_G, 29, 6);
    sleep(1);
    clearRegBit(REG_G, 29, 6);
    sleep(4);
    expect("M5 后 F45.1（转速<5，应为1）", read_f(45, 1), 1);
    /* F45.3 停止完成必须由主轴控制层在确认零速后置位：PLC 发布层只做只读查询。 */
    expect("M5 后 F45.3（停止完成）", read_f(45, 3), 1);

    printf("-> M19（G70.6 上升沿）\n"); fflush(stdout);
    setRegBit(REG_G, 70, 6);
    sleep(12);   /* 超过定向超时 10s */
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
        /* 真路径：仿真桩报 0x603F → alarm_poll_drive_errors() 置 is_error
         * → plc_f_signal_handler 输出 F45.0 / F0.6。 */
        sleep(3);
        printf("-> 读驱动器报警信号（F45.0 主轴报警 / F0.6 轴无报警）\n"); fflush(stdout);
        if (feed_only) {
            /* 反向对照：只坏 X 轴时 F45.0 必须为 0（证明按轴映射，而非有报警就置起）。 */
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
        /* 释放急停走真路径：G8.4（梯形图产出）0→1 上升沿由 plc_g_signal_scanner 的
         * SIGNAL_RISING 分支转成 PLC_EV_MSG_RESET → task_handle_reset_event →
         * SYS_EVENT_HMI_RESET。不要走 X8.4：plc_x_signal_scan_cycle() 在运动线程里没被调用。
         * G8.4 初值 0 = 急停按下，置 1 即构成上升沿。 */
        sleep(3);
        if (setRegBit(REG_G, 8, 4) != 0)
            printf("置 G8.4 失败\n");
        else
            printf("-> 已释放急停（G8.4 0→1，应触发自动复位）\n");
        fflush(stdout);

        if (with_estop_mid_reset) {
            /* 复位在飞时按住急停：G8.4=0 被翻成 SYS_ESTOP_INPUT；需配
             * CNC_RT_SIM_FAULT_STUCK=1，否则复位几拍就收口、按不住窗口。 */
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
