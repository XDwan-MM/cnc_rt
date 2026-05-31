#ifndef RT_SEM_H
#define RT_SEM_H

#include <semaphore.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile sig_atomic_t exit_flag, main_exit_flag, thread_exit_flag;

extern sem_t plc_sem, motion_sem, task_sem, path_plan_sem, moton_sem_exec_done;

extern void init_rt_sem();

extern void post_rt_sem();

extern void destroy_rt_sem();

extern void post_rt_sem_all();

extern volatile int op_flag;

#ifdef __cplusplus
}
#endif

#endif // RT_SEM_H
