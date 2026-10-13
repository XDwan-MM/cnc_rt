#include "master_sim.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include "hal_driver.h"
#include "hal_io.h"
#include "rt_data.h"
#include "global.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#ifdef DEBUG_MODE_ENABLED

extern RT_DATA *rt_data;

static int sim_cycle_count = 0;
static double sim_spindle_angle_deg = 0.0;

double get_simulated_spindle_angle_deg(void) {
    return sim_spindle_angle_deg;
}

void reset_simulated_spindle_angle(void) {
    sim_spindle_angle_deg = 0.0;
}

static void update_simulated_spindle_angle(void) {
    double interpolation_cycle = DEBUG_CYCLE_TIME_US * 1.0e-6;
    if (rt_data != NULL && rt_data->ncu_data.cycle > 0.0) {
        interpolation_cycle = rt_data->ncu_data.cycle;
    }

    if (rt_data != NULL && cycle_tp != NULL && cycle_tp->plan_block != NULL &&
        rt_data->sys_state[SYS_IS_RIGID] &&
        (cycle_tp->plan_block->type == RIGID_TAP ||
         cycle_tp->plan_block->type == TAP_RETURN)) {
        const int speed = abs(cycle_tp->plan_block->rigid_data.speed);
        const int sign = cycle_tp->plan_block->rigid_data.reverse ? -1 : 1;
        if (speed > 0) spindle.target_vel = sign * speed;
    }

    // The simulator models the spindle as an ideal velocity source. The
    // rigid tapping block above supplies its signed RPM from G84/G74.
    double angle_delta = (double)spindle.target_vel * 6.0 * interpolation_cycle;
    sim_spindle_angle_deg += angle_delta;
    sim_spindle_angle_deg = fmod(sim_spindle_angle_deg, 360.0);
    if (sim_spindle_angle_deg < 0.0) {
        sim_spindle_angle_deg += 360.0;
    }
}

void simulate_hal_data(void) {
    x_axis_driver.actual_pos = x_axis_driver.target_pos;
    y_axis_driver.actual_pos = y_axis_driver.target_pos;
    z_axis_driver.actual_pos = z_axis_driver.target_pos;
    update_simulated_spindle_angle();
    spindle.actual_vel = spindle.target_vel;


    // 模拟状态字（已使能状态）
    x_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    y_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    z_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
}

void Master_Run_Simulated(void) {
    sim_cycle_count = 0;
    reset_simulated_spindle_angle();

    printf("\n========== 模拟主站启动 ==========\n");
    printf("周期时间: %d us\n", DEBUG_CYCLE_TIME_US);
    printf("提示: 此模式下可进行断点调试\n");
    printf("==================================\n\n");

    // 设置 op_flag 表示已进入操作状态
    op_flag = 1;

    while (!thread_exit_flag) {
        sim_cycle_count++;

        // 模拟周期等待（使用 usleep 而非硬件中断）
        usleep(DEBUG_CYCLE_TIME_US);

        // 模拟 HAL 数据更新
        simulate_hal_data();

        // 触发软PLC信号量
        sem_post(&plc_sem);

        // 触发运动控制器和路径规划信号量
        sem_post(&motion_sem);
        sem_post(&path_plan_sem);

        // 等待运动执行完成
        sem_wait(&moton_sem_exec_done);

        // 每1000个周期打印一次状态（约1秒）
        if (sim_cycle_count % 1000 == 0) {
            // DEBUG_PRINT("模拟主站运行中... 周期数: %d", sim_cycle_count);
        }
    }

    printf("\n========== 模拟主站停止 ==========\n");
    printf("总运行周期数: %d\n", sim_cycle_count);
}

#endif // DEBUG_MODE_ENABLED
