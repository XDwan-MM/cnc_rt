#ifndef MASTER_SIM_H
#define MASTER_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file master_sim.h
 * @brief 模拟主站头文件
 *
 * 在调试模式下替代真实 EtherCAT 主站，提供：
 * - 模拟周期循环
 * - 模拟 HAL 数据更新
 * - 线程同步信号量触发
 */

/**
 * @brief 模拟主站运行函数
 *
 * 替代 Master_Run()，在调试模式下使用。
 * 使用 usleep 模拟周期等待，触发各线程信号量。
 */
void Master_Run_Simulated(void);

/**
 * @brief 模拟 HAL 数据更新
 *
 * 模拟驱动器和 IO 模块的数据变化。
 */
void simulate_hal_data(void);

#ifdef __cplusplus
}
#endif

#endif // MASTER_SIM_H
