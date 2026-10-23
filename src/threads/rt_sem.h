#ifndef RT_SEM_H
#define RT_SEM_H


#include <semaphore.h>
#include <stdint.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile sig_atomic_t exit_flag, main_exit_flag;
/* 线程间同步量，不是信号处理场景：volatile 不构成线程同步。
 * 读写一律走 __atomic_load_n / __atomic_store_n（不引 stdatomic.h：本头会被 .cpp 包含）。 */
extern volatile sig_atomic_t thread_exit_flag;
extern volatile sig_atomic_t pdo_unexpected_exit;

extern sem_t plc_sem, motion_sem, task_sem, path_plan_sem, moton_sem_exec_done;

/* PLC 完成屏障：PLC 会直接读写面板 PDO，不设屏障就会和 begin/commit 并发。
 * PLC 干完活置周期号并 post，PDO 在 commit 前等周期号追上本拍。
 * 光看信号量计数不够：它证明不了"本拍"完成，迟到的第 N 拍会被当成第 N+1 拍。 */
extern sem_t plc_done_sem;
/* PDO 线程放行前写入本拍周期号；业务线程开工时读走、干完原样写回 *_done_cycle。
 * 用 uint32_t 而非 int：1kHz 下 int 24.9 天即溢出且有符号溢出未定义，无符号回绕
 * 到 49.7 天；不用 uint64_t 是因为 32 位目标上 8 字节原子可能非无锁。
 * 跨线程传值必须用 __atomic_* 访问（volatile 不是内存屏障）。 */
extern volatile uint32_t plc_done_cycle;
extern volatile uint32_t plc_cycle_target;
extern volatile uint32_t motion_done_cycle;
extern volatile uint32_t motion_cycle_target;

extern void init_rt_sem();

extern void post_rt_sem();

extern void destroy_rt_sem();

extern void post_rt_sem_all();

extern volatile int op_flag;

#ifdef __cplusplus
}
#endif

#endif // RT_SEM_H
