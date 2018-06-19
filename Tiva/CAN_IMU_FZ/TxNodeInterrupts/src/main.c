//*****************************************************************************
//
// main.c - Motor control program with CAN and UART interfaces.
// begun 4/12/18
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "adc.h"
#include "i2c_master_no_int.h"
#include "LSM6DS33.h"

#define LED_GREEN GPIO_PIN_2
#define LED_RED GPIO_PIN_3

#define PUB_FREQ 500
#define CAN_XL_ID 0x0110
#define CAN_FZ_ID 0x1010

tCANMsgObject sCANMessageXL;
tCANMsgObject sCANMessageFZ;
uint8_t pui8MsgDataXL[8];
uint8_t pui8MsgDataFZ[8];

//*****************************************************************************
//
// Global variables shared between timer interrupt and main loop
//
//*****************************************************************************
int16_t Az;
uint8_t whoiam;
uint32_t fz_adc;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;
volatile uint32_t g_ui32Msg2Count = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag1 = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIO port A which is used for UART0 pins.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0 so that we can configure the clock.
  GPIOPinConfigure(GPIO_PA0_U0RX); // pin muxing
  GPIOPinConfigure(GPIO_PA1_U0TX); // pin muxing
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Select the alternate (UART) function for these pins.
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART clock source.
  UARTStdioConfig(0, 115200, 16000000); // Initialize the UART for console I/O.
}

//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void
SensorPubIntHandler(void)
{
  static uint8_t LED_count = 0;
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.

  /****************************************************************************
  * read from sensors
  ****************************************************************************/
  Az = getzXL();
  whoiam = WhoAmI();
  fz_adc = ADCread();

  /****************************************************************************
  * write sensor data to CAN
  ****************************************************************************/
  (*(uint32_t *)pui8MsgDataXL) = Az; // get ready to send Az over CAN
  (*(uint32_t *)pui8MsgDataFZ) = fz_adc; // get ready to send fz_adc over CAN

  CANMessageSet(CAN0_BASE, 1, &sCANMessageXL, MSG_OBJ_TYPE_TX);
  CANMessageSet(CAN0_BASE, 2, &sCANMessageFZ, MSG_OBJ_TYPE_TX);

  HWREGBITW(&g_ui32Flags, 0) ^= 1; // Toggle the flag for the first timer.

  LED_count++;

  if (LED_count < PUB_FREQ/10) {
    GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, LED_GREEN);
    GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, 0);
  } else {
    GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, LED_RED);
  }
  if (LED_count>=(PUB_FREQ/5)) {
    LED_count = 0;
  }
}

//*****************************************************************************
//
// Set up timer to generate interrupts
//
//*****************************************************************************
void TimerBegin(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable the peripherals used by this example.
  IntMasterEnable(); // Enable processor interrupts.
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configure a 32-bit periodic timer.
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / PUB_FREQ);
  IntEnable(INT_TIMER0A); // Setup the interrupts for the timer timeouts.
  IntPrioritySet(INT_TIMER0A, 0x20); // set the Timer 0A interrupt priority to be "low"
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A); // Enable the timer.
  UARTprintf("Timer A0 initialized!\n");
}

//*****************************************************************************
//
// This function prints some information about the CAN message to the
// serial port for information purposes only.
//
//*****************************************************************************
void
PrintCANMessageInfo(tCANMsgObject *psCANMsg, uint32_t ui32MsgObj)
{
    unsigned int uIdx;

    if(psCANMsg->ui32Flags & MSG_OBJ_DATA_LOST) // if there is an indication that some messages were lost
    {
        UARTprintf("CAN message loss detected on message object %d\n",
                   ui32MsgObj);
    }
    // Print out the contents of the message that was received.
    UARTprintf("Msg Obj=%u ID=0x%05X len=%u data=0x", ui32MsgObj,
               psCANMsg->ui32MsgID, psCANMsg->ui32MsgLen);
    for(uIdx = 0; uIdx < psCANMsg->ui32MsgLen; uIdx++)
    {
        UARTprintf("%02X ", psCANMsg->pui8MsgData[uIdx]);
    }
    UARTprintf("\n");
}

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been received.
//
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // Read the CAN interrupt status to find the cause of the interrupt
    if(ui32Status == CAN_INT_INTID_STATUS) // If the cause is a controller status interrupt, then get the status
    {
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        g_bErrFlag = 1; // Set a flag to indicate some errors may have occurred.
    }
    else if(ui32Status == 1) // Check if the cause is message object 1.
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);
        g_ui32MsgCount++; // increment a counter to track how many messages have been received
        g_bRXFlag1 = 1; //Set flag to indicate received message is pending for this message object.
        g_bErrFlag = 0; // Since a message was received, clear any error flags.
    }
    else if(ui32Status == 2)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 2, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg2Count++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    else // Otherwise, something unexpected caused the interrupt.
    {
        // Spurious interrupt handling can go here.
    }
}


//*****************************************************************************
//
// Configure:
//    CAN
//    Timers
//
//*****************************************************************************
int
main(void)
{
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal used on your board.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC)
                                       25000000);
#else
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
#endif


    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for CAN operation.
    //
    InitConsole();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LED_GREEN|LED_RED);

    //*************************************************************************
    // Set up CAN
    //*************************************************************************
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // E for custom board, B for Launchpad

    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    GPIOPinConfigure(GPIO_PE4_CAN0RX); // E for custom board, B for Tiva
    GPIOPinConfigure(GPIO_PE5_CAN0TX);

    // Enable the alternate function on the GPIO pins.
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //Enable the CAN peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Set up the bit rate for the CAN bus.
    uint32_t canbitrate_actual;
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
        defined(TARGET_IS_TM4C129_RA1) ||                                         \
        defined(TARGET_IS_TM4C129_RA2)
        CANBitRateSet(CAN0_BASE, ui32SysClock, 500000);
    #else
        canbitrate_actual = CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
    #endif
    UARTprintf("CAN bit rate set at %d bps.\n", canbitrate_actual);

    // Enable interrupts on the CAN peripheral.
    CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Enable the CAN interrupt on the processor (NVIC).
    IntEnable(INT_CAN0);
    IntPrioritySet(INT_CAN0, 0x00); // set the CAN interrupt priority to be "high"

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    // initialize msg object 1, used for accelerometer
    sCANMessageXL.ui32MsgID = CAN_XL_ID;
    sCANMessageXL.ui32MsgIDMask = 0;
    sCANMessageXL.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageXL.ui32MsgLen = sizeof(pui8MsgDataXL);
    sCANMessageXL.pui8MsgData = pui8MsgDataXL;

    // initialize msg object 2, used for force sensor
    sCANMessageFZ.ui32MsgID = CAN_FZ_ID;
    sCANMessageFZ.ui32MsgIDMask = 0;
    sCANMessageFZ.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageFZ.ui32MsgLen = sizeof(pui8MsgDataFZ);
    sCANMessageFZ.pui8MsgData = pui8MsgDataFZ;
    //***********end of CAN setup**********************************************

    //*************************************************************************
    // Set up sensors and their peripherals
    //*************************************************************************
    LSM6DS33_init();
    UARTprintf("IMU initialized! WhoAmI = %02X\n",WhoAmI());

    ADCenable();
    UARTprintf("ADC initialized!\n");

    //*************************************************************************
    // Now that everything is set up, start the timer and its ISR
    //*************************************************************************
    TimerBegin();

    UARTprintf("XL/FZ node up!\n");

    for(;;)
    {
        UARTprintf("WhoAmI: %d, XL: %d, FZ: %4d\n",whoiam,Az,fz_adc);
    }

    //
    // Return no errors
    //
    return(0);
}
