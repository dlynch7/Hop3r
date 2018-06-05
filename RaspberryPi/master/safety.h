#ifndef __SAFETY__H__
#define __SAFETY__H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <wiringPi.h>

#include "can_io.h"
#include "linux-can-utils/lib.h"

#define BOOM_ROLL_MAX 45
#define BOOM_ROLL_MIN -45
#define BOOM_PITCH_MAX 45
#define BOOM_PITCH_MIN -45
#define BOOM_YAW_MAX 45
#define BOOM_YAW_MIN -45

#define COPLEY_EN 26

uint8_t run_program;

void safety_init(void);

void trap(int signal);

int kill_motors(void);

#endif
