#include <QtTest>
#include "test_registry.h"

extern "C" {
#include "datatype.h"
#include "global.h"
#include "homing.h"
#include <shmapi.h>
#include <varapi.h>
#include <parapi.h>
}

/********************************************************************
 * @file        tst_homing.cpp
 * @brief       src/motion/homing 回零模块单元测试
 *
 * 回零模块重度依赖共享内存（par_axis / var_chan / rt_data）与全局
 * 轨迹规划指针 homing_tp，核心逻辑多为 static。
 *   - homing_tp_init()      按轴参数分配并初始化 homing_tp
 *   - homing_tp_reset()     将各轴状态归零
 *   - homing_handle_estop() 急停：复位并清通道回零/移动标志
 *
 * 依赖共享内存初始化（shmInit + rt_global_init）。若初始化失败
 * （例如未以 sudo 运行、共享内存不可用），相关用例 QSKIP 跳过而非崩溃。
********************************************************************/

class TestHoming : public QObject
{
    Q_OBJECT

private:
    bool shm_ready = false;

    // 通过共享内存接口写入一组轴参数，供 homing_tp_init 读取
    void set_axis_param(int axis, int index, double real_value)
    {
        ParamUnit p = {};
        p.value.val_real = real_value;
        setParData(PARAMAN_FILE_AXIS, axis, index, p);
    }
    void set_axis_param_uint(int axis, int index, unsigned int uint_value)
    {
        ParamUnit p = {};
        p.value.val_uint = uint_value;
        setParData(PARAMAN_FILE_AXIS, axis, index, p);
    }

private slots:
    void initTestCase();
    void test_tp_init_allocates();
    void test_tp_init_reads_axis_params();
    void test_tp_reset_clears_state();
    void test_handle_estop_clears_flags();
};

void TestHoming::initTestCase()
{
    // 一次性初始化共享内存与全局实时数据
    if (!shmInit()) {
        qWarning("共享内存初始化失败，跳过回零测试（请确认以 sudo 运行）");
        return;
    }
    GNC_RES res = rt_global_init();
    if (res != GNC_OPERATOR_SUCCESS) {
        qWarning("rt_global_init 失败(%d)，跳过回零测试", res);
        return;
    }
    shm_ready = true;
}

// homing_tp_init 应为每个进给轴分配 homing_tp，并设置默认状态
void TestHoming::test_tp_init_allocates()
{
    if (!shm_ready) QSKIP("共享内存未就绪");

    homing_tp_init();
    QVERIFY(homing_tp != nullptr);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        QVERIFY(homing_tp[i].homing_block != nullptr);
        QVERIFY(homing_tp[i].homing_profile != nullptr);
        QVERIFY(homing_tp[i].traj_data != nullptr);
        // 默认空闲、正向、直线块
        QCOMPARE(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        QCOMPARE(homing_tp[i].homing_direction, AXIS_HOMING_POSITIVE);
        QCOMPARE(homing_tp[i].homing_block->type, LINE);
    }
}

// homing_tp_init 应从 par_axis 读取快移速度/加速度/捷度写入 homing_block
void TestHoming::test_tp_init_reads_axis_params()
{
    if (!shm_ready) QSKIP("共享内存未就绪");

    const double rapid_spd = 1000.0;
    const double rapid_acc = 5000.0;
    const double rapid_jk  = 20000.0;
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        set_axis_param(i, PAR_AX_RAPID_SPD, rapid_spd);
        set_axis_param(i, PAR_AX_RAPID_ACC, rapid_acc);
        set_axis_param(i, PAR_AX_RAPID_JK,  rapid_jk);
    }

    homing_tp_init();

    for (int i = 0; i < SYS_AXES_NUM; i++) {
        QVERIFY(IsEqualTo(homing_tp[i].homing_block->max_vel,  rapid_spd));
        QVERIFY(IsEqualTo(homing_tp[i].homing_block->feed_vel, rapid_spd));
        QVERIFY(IsEqualTo(homing_tp[i].homing_block->max_acc,  rapid_acc));
        QVERIFY(IsEqualTo(homing_tp[i].homing_block->max_jerk, rapid_jk));
        QVERIFY(IsEqualTo(homing_tp[i].homing_block->end_vel,  0.0));
    }
}

// homing_tp_reset 应将状态置为 IDLE、清回零请求、清空 traj_data
void TestHoming::test_tp_reset_clears_state()
{
    if (!shm_ready) QSKIP("共享内存未就绪");

    homing_tp_init();
    // 人为制造一个“回零进行中”的脏状态
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        homing_tp[i].homing_state = ABS_EXEC_HOMING;
        homing_tp[i].is_ref_on = true;
        homing_tp[i].traj_data->v = 123.0;
        homing_tp[i].traj_data->a = 456.0;
    }

    homing_tp_reset();

    for (int i = 0; i < SYS_AXES_NUM; i++) {
        QCOMPARE(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        QVERIFY(!homing_tp[i].is_ref_on);
        QVERIFY(IsEqualTo(homing_tp[i].traj_data->v, 0.0));
        QVERIFY(IsEqualTo(homing_tp[i].traj_data->a, 0.0));
    }
}

// homing_handle_estop 应复位 homing_tp 并清除通道“回零中/移动中”标志
void TestHoming::test_handle_estop_clears_flags()
{
    if (!shm_ready) QSKIP("共享内存未就绪");

    homing_tp_init();
    // 预置“回零中/移动中”标志
    setVarChanInt(0, VAR_CHAN_IS_HOMING, 1, 0);
    setVarChanInt(0, VAR_CHAN_IS_MOVING, 1, 0);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        homing_tp[i].homing_state = ABS_EXEC_HOMING;
        homing_tp[i].is_ref_on = true;
    }

    homing_handle_estop();

    int is_homing = -1, is_moving = -1;
    getVarChanInt(0, VAR_CHAN_IS_HOMING, &is_homing, 0);
    getVarChanInt(0, VAR_CHAN_IS_MOVING, &is_moving, 0);
    QCOMPARE(is_homing, 0);
    QCOMPARE(is_moving, 0);
    for (int i = 0; i < SYS_AXES_NUM; i++) {
        QCOMPARE(homing_tp[i].homing_state, ABS_EXEC_IDLE);
        QVERIFY(!homing_tp[i].is_ref_on);
    }
}

REGISTER_TEST(TestHoming)

#include "tst_homing.moc"
