#include "master_sim.h"
#include "debug_mode.h"
#include "rt_sem.h"
#include "hal_driver.h"
#include "hal_io.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#ifdef DEBUG_MODE_ENABLED

static int sim_cycle_count = 0;

void simulate_hal_data(void) {
    // 模拟驱动器数据更新
    // 这里可以根据需要添加模拟逻辑

    // 模拟位置递增（简单示例）
    static int32_t sim_pos[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        sim_pos[i] += 100;  // 每周期位置增加
    }

    // 更新 HAL 驱动器数据
    x_axis_driver.actual_pos = sim_pos[1];
    y_axis_driver.actual_pos = sim_pos[2];
    z_axis_driver.actual_pos = sim_pos[3];

    // 模拟状态字（已使能状态）
    x_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    y_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
    z_axis_driver.state = DRV_STATE_OPERATION_ENABLED;
}

void Master_Run_Simulated(void) {
    printf("\n========== 模拟主站启动 ==========\n");
    printf("周期时间: %d us\n", DEBUG_CYCLE_TIME_US);
    printf("提示: 此模式下可进行断点调试\n");
    printf("==================================\n\n");

    // 模拟初始化延时
    usleep(100000);  // 100ms

    // 设置 op_flag 表示已进入操作状态
    op_flag = 1;

    while (!exit_flag) {
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
            DEBUG_PRINT("模拟主站运行中... 周期数: %d", sim_cycle_count);
        }
    }

    printf("\n========== 模拟主站停止 ==========\n");
    printf("总运行周期数: %d\n", sim_cycle_count);
}

#endif // DEBUG_MODE_ENABLED
