//*****************************************************************************
//
// main.c - Boom encoder program with CAN and SPI (SSI) interfaces.
// begun 4/27/18
//
// This program uses the following peripherals and I/O signals:
// - SSI0 peripheral
// - GPIO Port A peripheral (for SSI0 pins)
// - SSI0Clk - PA2
// - SSI0Fss - PA3
// - SSI0Rx  - PA4
// - SSI0Tx  - PA5
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

// Standard C includes:
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Tivaware includes:
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
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

//#define LED_RED GPIO_PIN_1
//#define LED_BLUE GPIO_PIN_2
//#define LED_GREEN GPIO_PIN_3

// for custom board
#define LED_RED GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3

#define BOOM_READ_FREQ 10 // TODO: choose the right freq

#define ROLL 1
#define PITCH 2
#define YAW 3
#define BOOM_ID ROLL // valid IDs are 1, 2, and 3

#define NUM_SSI_DATA 3

#define PI 3.14159

//*****************************************************************************
//
// Global variables shared between main loop and motor control ISR
//
//*****************************************************************************
tCANMsgObject sCANMessage;
uint32_t ui32MsgData;
uint8_t *pui8MsgData;

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
// This function provides a 1 second delay using a simple polling method.
//
//*****************************************************************************
void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}

//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void
BoomReadIntHandler(void)
{
    int16_t angleDeg10 = 0;

    static uint32_t pui32DataTx[NUM_SSI_DATA];
    static uint32_t pui32DataRx[NUM_SSI_DATA];

    // turn on LED
    GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, LED_RED);

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.
    // read from the encoder via SPI
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0])) {;}
    //
    // Initialize the data to send.
    pui32DataTx[0] = 's';
    pui32DataTx[1] = 'p';
    pui32DataTx[2] = 'i';

    UARTprintf("Sent:\n  ");
    UARTprintf("'%c' ", pui32DataTx[0]);

    SSIDataPut(SSI0_BASE, pui32DataTx[0]);

    while(SSIBusy(SSI0_BASE)) {;}

    UARTprintf("\nReceived:\n");

    SSIDataGet(SSI0_BASE, &pui32DataRx[0]);

    pui32DataRx[0] &= 0x3FFF; // mask to 14-bit encoder output, per AEAT-6600 datasheet

    angleDeg10 = (3600*pui32DataRx[0]/16384.0f);

    UARTprintf("Angle (degrees): %d.%01d\n", angleDeg10/10,angleDeg10%10);

    (*(uint32_t *)pui8MsgData) = angleDeg10;

    // Print a message to the console showing the message count and the
    // contents of the message being sent.
    //
    UARTprintf("Sending msg: 0x%02X %02X %02X %02X\n",
               pui8MsgData[0], pui8MsgData[1], pui8MsgData[2],
               pui8MsgData[3]);

    //
    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this example).  This function will cause
    // the message to be transmitted right away.
    //

    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);

    // turn off LED
    GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, 0);

    HWREGBITW(&g_ui32Flags, 0) ^= 1; // Toggle the flag for the first timer.
    // GPIOPinWrite(GPIO_PORTF_BASE, LED_RED, g_ui32Flags << 1); // Use the flags to Toggle the LED for this timer
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
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / BOOM_READ_FREQ);
  IntEnable(INT_TIMER0A); // Setup the interrupts for the timer timeouts.
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A); // Enable the timer.
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
{ // based on CANIntHandler in simple_tx.c
    uint32_t ui32Status;
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        g_bErrFlag = 1;
    }
    else if(ui32Status == 1)
    {
        CANIntClear(CAN0_BASE, 1);
        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }
    else
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

    // tCANMsgObject sCANMessage;
    // uint32_t ui32MsgData;
    // uint8_t *pui8MsgData;

    pui8MsgData = (uint8_t *)&ui32MsgData;

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

    TimerBegin();

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for CAN operation.
    //
    InitConsole();

    // Rx: light up GREEN LED
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_GREEN);

    // for custom board
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LED_RED|LED_GREEN);

    // turn on LED
    GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, LED_RED);
    GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, LED_GREEN);

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
                           SSI_MODE_MASTER, 1000000, 16); // changed to 16-bit mode, per AEAT-6600 datasheet
    #else
        SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 1000000, 16); // changed to 16-bit mode, per AEAT-6600 datasheet
    #endif

    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);

    UARTprintf("SSI ->\n");
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 16-bit\n\n");

    //
    // For this example CAN0 is used with RX and TX pins on port B4 and B5.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // for custom board; CAN is on E4 (RX) and E5 (TX)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    //GPIOPinConfigure(GPIO_PB4_CAN0RX);
    //GPIOPinConfigure(GPIO_PB5_CAN0TX);

    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    //GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() or ui32SysClock is used to determine the
    // clock rate that is used for clocking the CAN peripheral.  This can be
    // replaced with a  fixed value if you know the value of the system clock,
    // saving the extra function call.  For some parts, the CAN peripheral is
    // clocked by a fixed 8 MHz regardless of the system clock in which case
    // the call to SysCtlClockGet() or ui32SysClock should be replaced with
    // 8000000.  Consult the data sheet for more information about CAN
    // peripheral clocking.
    //
    uint32_t canbitrate_actual;
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    CANBitRateSet(CAN0_BASE, ui32SysClock, 500000);
#else
    canbitrate_actual = CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
#endif
    UARTprintf("CAN bit rate set at %d bps.\n", canbitrate_actual);

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN0);

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);


    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0.
    //
    ui32MsgData = 0;
    sCANMessage.ui32MsgID = 0x4001;
    sCANMessage.ui32MsgIDMask = 0;
    sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessage.ui32MsgLen = sizeof(pui8MsgData);
    sCANMessage.pui8MsgData = pui8MsgData;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    UARTprintf("Boom %d node up!\n",BOOM_ID);

    // turn off LED
    //GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, 0);
    //GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, 0);

    //
    // Enter loop to process received messages.  This loop just checks a flag
    // that is set by the interrupt handler, and if set it reads out the
    // message and displays the contents.  This is not a robust method for
    // processing incoming CAN data and can only handle one messages at a time.
    // If many messages are being received close together, then some messages
    // may be dropped.  In a real application, some other method should be used
    // for queuing received messages in a way to ensure they are not lost.  You
    // can also make use of CAN FIFO mode which will allow messages to be
    // buffered before they are processed.
    //
    while(1)
    {
      ;
    }

    //
    // Return no errors
    //
    return(0);
}
