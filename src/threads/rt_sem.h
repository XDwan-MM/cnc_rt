#ifndef RT_SEM_H
#define RT_SEM_H

#include <semaphore.h>
#include <stdint.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile sig_atomic_t exit_flag, main_exit_flag, thread_exit_flag;

extern sem_t plc_sem, motion_sem, task_sem, path_plan_sem, moton_sem_exec_done;

/* PLC 完成屏障。旧路径只等运动线程，PLC 完全没屏障，而 PLC 会直接读写面板 PDO，
 * 于是 begin/commit 和 PLC 可能并发跑。PLC 干完活置周期号并 post，
 * PDO 线程 commit 前等周期号追上本拍。
 * 光看信号量计数不够：它证明不了"本拍"完成，迟到的第 N 拍会被当成第 N+1 拍。 */
extern sem_t plc_done_sem;
/* PDO 线程放行前写入本拍周期号；业务线程开工时读走、干完原样写回 *_done_cycle。
 *
 * 用 uint32_t 而不是 int：int 在 1kHz 下 24.9 天就溢出，而**有符号溢出是未定义
 * 行为**（本工程没开 -fwrapv）；无符号回绕定义良好，周期也翻倍到 49.7 天。不用
 * uint64_t 是因为 32 位目标上 8 字节原子可能不是无锁的。
 * 跨线程传值，必须用 __atomic_* 访问，裸读不行（volatile 不是内存屏障）。 */
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
