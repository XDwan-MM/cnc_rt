#include "rt_sem.h"

volatile sig_atomic_t exit_flag = 0;

sem_t plc_sem, motion_sem, task_sem, path_plan_sem;

void init_rt_sem();
void post_rt_sem();
void destroy_rt_sem();

void init_rt_sem() {
    sem_init(&plc_sem, 0, 0);
    sem_init(&motion_sem, 0, 0);
    sem_init(&task_sem, 0, 0);
    sem_init(&path_plan_sem, 0, 0);
}

void post_rt_sem() {
    sem_post(&plc_sem);
    sem_post(&motion_sem);
    sem_post(&task_sem);
    sem_post(&path_plan_sem);
}

void destroy_rt_sem() {
    sem_destroy(&plc_sem);
    sem_destroy(&motion_sem);
    sem_destroy(&task_sem);
    sem_destroy(&path_plan_sem);
}
