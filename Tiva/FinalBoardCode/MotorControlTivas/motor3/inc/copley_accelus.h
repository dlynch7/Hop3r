#ifndef __COPLEY_ACCELUS__H__
#define __COPLEY_ACCELUS__H__
// Header file for copley_accelus.c
// interfaces with Copley Accelus motor driver

// See CopleyNotes.txt (parent directory) for details

// Author: Dan Lynch
// Begun 4/12/18

// PWM interface added 4/26/18

uint8_t init_copley(void);

uint16_t set_current_mA(int16_t cur_ref_mA);
uint16_t get_current_mA(void);

#endif
