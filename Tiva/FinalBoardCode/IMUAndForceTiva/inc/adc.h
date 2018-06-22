#ifndef __ADC__H__
#define __ADC__H__
// Header file for adc.c
// implements high-level ADC functions

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

void ADCenable(void); // ADC setup function
uint32_t ADCread(void); // ADC read function

#endif
