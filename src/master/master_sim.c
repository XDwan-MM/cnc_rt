#include "master_sim.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include "driver_hal.h"
#include "hal_io.h"
#include "rt_data.h"
#include "global.h"
#include "spl_control.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#ifdef DEBUG_MODE_ENABLED

bool reset_sem = false;
static int sim_cycle_count = 0;
static double sim_spindle_angle_deg = 0.0;

void simulate_hal_data(void) {
    x_axis_driver.actual_pos = x_axis_driver.target_pos;
    y_axis_driver.actual_pos = y_axis_driver.target_pos;
    z_axis_driver.actual_pos = z_axis_driver.target_pos;
    /* 仿真驱动按照驱动器目标速度返回实际速度反馈。 */
    spindle_driver.actual_vel = spindle_driver.target_vel;


    // 模拟状态字（已使能状态）
    x_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    y_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    z_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    /* 主轴也必须处于运行使能状态，否则运控线程不会执行速度规划。 */
    spindle_driver.state = DRV_STATE_OPERATION_ENABLED;
}

void Master_Run_Simulated(void) {
    sim_cycle_count = 0;

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
