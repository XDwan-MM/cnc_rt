#ifndef RT_SEM_H
#define RT_SEM_H

#include <semaphore.h>
#include <signal.h>

extern volatile sig_atomic_t exit_flag;

extern sem_t plc_sem, motion_sem, task_sem, path_plan_sem;

extern void init_rt_sem();

extern void post_rt_sem();

extern void destroy_rt_sem();

#endif // RT_SEM_H
