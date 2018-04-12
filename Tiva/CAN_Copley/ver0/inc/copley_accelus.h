#ifndef __COPLEY_ACCELUS__H__
#define __COPLEY_ACCELUS__H__
// Header file for copley_accelus.c
// interfaces with Copley Accelus motor driver

uint8_t set_current_mA(int16_t cur_ref_mA);
uint16_t get_current_mA(void);

#endif
