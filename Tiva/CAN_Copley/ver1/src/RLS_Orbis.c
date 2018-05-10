// RLS_orbis.c
// backend for SPI interface with RLS Orbis encoder

// Interface adapted from
// https://github.com/enginerd887/2R-Robots/blob/master/MidtermCode/Tiva/Encoder.c

#include "RLS_Orbis.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define NUM_SSI_DATA 5
uint32_t pui32DataTx[NUM_SSI_DATA];
uint32_t pui32DataRx[NUM_SSI_DATA];

void initRLS(void) {
  // Set up SPI over SSI:

  // The SSI0 peripheral must be enabled for use.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  GPIOPinConfigure(GPIO_PA3_SSI0FSS);
  GPIOPinConfigure(GPIO_PA4_SSI0RX);
  GPIOPinConfigure(GPIO_PA5_SSI0TX);

  GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

  #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8); // 8-bit mode
  #else
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8); // 8-bit mode
  #endif

  // Enable the SSI0 module.
  SSIEnable(SSI0_BASE);

  UARTprintf("SSI ->\n");
  UARTprintf("  Mode: SPI\n");
  UARTprintf("  Data: 8-bit\n");
}

uint32_t readRLS(void) {
  uint32_t angleDeg;
  uint8_t index = 0;

  uint32_t pui32DataTx[NUM_SSI_DATA];
  uint32_t pui32DataRx[NUM_SSI_DATA];

  // Initialize the data to send.
  pui32DataTx[0] = 0x74; // 't' asks for readhead temperature
  pui32DataTx[1] = 0x00;
  pui32DataTx[2] = 0x00;
  pui32DataTx[3] = 0x00;
  pui32DataTx[4] = 0x00;

  // read from the encoder via SPI
  while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0])) {;}

  for(index = 0; index < NUM_SSI_DATA; index++) {
     SSIDataPut(SSI0_BASE, pui32DataTx[index]);
  }

  while(SSIBusy(SSI0_BASE)) {;}

  SysCtlDelay(10);

  for (index = 0; index < NUM_SSI_DATA; index++) {
    pui32DataRx[index] &= 0x00FF;
    SSIDataGet(SSI0_BASE, &pui32DataRx[index]);
  }

  angleDeg = ((pui32DataRx[0]<<6) | (pui32DataRx[1]>>2));

  angleDeg = 3600*angleDeg/8192;
  // angleDeg = 0x3FF & ((pui32DataRx[0]<<8) | (pui32DataRx[1]));
  // angleDeg = 360*angleDeg/16384.0;
  //
  // UARTprintf("Sent:\n  ");
  // UARTprintf("'%c' ", pui32DataTx[0]);
  // UARTprintf("RLS: %02X %02X\n",pui32DataRx[0],pui32DataRx[1]);
  // UARTprintf("RLS: %04X = %d = %d degrees\n",angleDeg, angleDeg, (360*angleDeg/8192));
  // UARTprintf("Angle: %d\n",angleDeg);

  return angleDeg;

  // UARTprintf("Angle (degrees): %d.%01d\n", angleDeg10/10,angleDeg10%10);
}
