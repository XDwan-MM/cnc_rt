#include "rt_sem.h"

volatile sig_atomic_t exit_flag = 0;
volatile sig_atomic_t main_exit_flag = 0;
volatile sig_atomic_t thread_exit_flag = 0;
/* 下面这个由 PDO 线程置位、主线程读走：写用 RELEASE、读用 ACQUIRE。 */
volatile sig_atomic_t pdo_unexpected_exit = 0;
volatile int op_flag = 0;

sem_t plc_sem, motion_sem, /*task_sem,*/ path_plan_sem, moton_sem_exec_done;
sem_t plc_done_sem;
volatile uint32_t plc_done_cycle = 0;
volatile uint32_t plc_cycle_target = 0;
volatile uint32_t motion_done_cycle = 0;
volatile uint32_t motion_cycle_target = 0;

void init_rt_sem() {
    sem_init(&plc_sem, 0, 0);
    sem_init(&motion_sem, 0, 0);
    sem_init(&path_plan_sem, 0, 0);
    sem_init(&moton_sem_exec_done, 0, 0);
    sem_init(&plc_done_sem, 0, 0);
    plc_done_cycle = 0;
}

void post_rt_sem() {
    //sem_post(&task_sem);
   // sem_post(&path_plan_sem);
}

void post_rt_sem_all() {
    sem_post(&plc_sem);
    sem_post(&motion_sem);
    sem_post(&path_plan_sem);
    sem_post(&moton_sem_exec_done);
    /* 退出路径也要唤醒等 PLC 完成屏障的人，否则关机时 PDO 线程会一直卡在
     * sem_timedwait 上直到超时。置一个不可能追上的周期号没有意义，所以这里
     * 只放行信号量——等的人必须自己检查 thread_exit_flag 才能退出。 */
    sem_post(&plc_done_sem);
}

void destroy_rt_sem() {
    sem_destroy(&plc_sem);
    sem_destroy(&motion_sem);
    sem_destroy(&path_plan_sem);
    sem_destroy(&moton_sem_exec_done);
    sem_destroy(&plc_done_sem);
}
