// copley_accelus.c
// interfaces with Copley Accelus motor driver

// See CopleyNotes.txt (parent directory) for details

// Author: Dan Lynch
// Begun 4/12/18
// PWM interface added 4/26/18

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "copley_accelus.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define PWM_PERIOD 640 // f_PWM = fsys/PWM_PERIOD = 16,000,000/PWM_PERIOD
#define MAX_CUR_MA 20000 // max commandable current (mA)
// current resolution = MAX_CUR_MA/(PWM_PERIOD/2)
#define SLOPE (MAX_CUR_MA/(PWM_PERIOD>>1))
// #define SLOPE 0.015238095

//*****************************************************************************
//
// Quasi-global variables (global w.r.t. copley_accelus.c)
//
//*****************************************************************************

//*****************************************************************************
//
// Private functions (used only in copley_accelus.c):
//
//*****************************************************************************
static uint8_t init_PWM0(void) {
  //Configure PWM Clock to match system
  SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

  // Enable the peripherals used by this program.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins

  //Configure pin C5 as PWM
  GPIOPinConfigure(GPIO_PC5_M0PWM7);
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

  //Configure PWM Options
  //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  //Set the Period (expressed in clock ticks)
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_PERIOD);

  //Set PWM duty-50% (Period /2)
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,(PWM_PERIOD/2));

  // Enable the PWM generator
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);

  // Turn on the Output pins
  PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);

  return 0;
}

//*****************************************************************************
//
// Public functions (available to other files via copley_accelus.h):
//
//*****************************************************************************
uint8_t init_copley(void) {
  if(init_PWM0()) {
    UARTprintf("Failed to set up PWM0.\n");
  }
  else {
    UARTprintf("Initialized PWM0.\n");
  }

  // Initialize Copley Accelus in a safe operating mode
  set_current_mA(0);
  return 0;
}

uint16_t set_current_mA(int16_t cur_ref_mA) {
  uint16_t pulse_width = PWM_PERIOD>>1; // 50% duty cycle = 0 mA commanded
  if (abs(cur_ref_mA) < MAX_CUR_MA) {
    // pulse_width = (PWM_PERIOD>>1) - cur_ref_mA/SLOPE;
    pulse_width = (PWM_PERIOD>>1) - cur_ref_mA/SLOPE;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,pulse_width);
  }

  return pulse_width; // TO-DO: receive, process, and return response from Copley Accelus
}

uint16_t get_current_mA(void) {
  return 0;
}
