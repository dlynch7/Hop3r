#ifndef __PER_THREADS__H__
#define __PER_THREADS__H__
// Header file for per_threads.c
// Implements periodic thread scheduling

// per_threads.c is a library based on timer.c from http://2net.co.uk/tutorial/periodic_threads

#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

struct periodic_info {
	int sig;
	sigset_t alarm_sig;
};

pthread_mutex_t mutex1;

int make_periodic(int unsigned period, struct periodic_info *info);

void wait_period(struct periodic_info *info);

int setup_periodic(void);

void display_sched_attr(int policy, struct sched_param *param);

#endif
