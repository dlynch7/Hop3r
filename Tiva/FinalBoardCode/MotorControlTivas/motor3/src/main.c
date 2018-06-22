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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "copley_accelus.h"
#include "RLS_Orbis.h"

#define LED_GREEN GPIO_PIN_2
#define LED_RED GPIO_PIN_3

#define POS_CTRL_FREQ 1000
#define DT 0.001
#define MOTOR_ID 3
#define MOTOR_EN_MASK (1 << (2*MOTOR_ID - 1))
#define CAN_MOTOR_ID 0x4001
#define DEADBAND 15 // in tenths of degrees

#define PI 3.14159

//*****************************************************************************
//
// Global variables shared between main loop and motor control ISR
//
//*****************************************************************************
typedef enum {CUR_CTRL, POS_CTRL, IDLE} mode; // define data structure containing modes
volatile mode MODE;
volatile int16_t CAN_REF = 2290;
volatile int16_t CUR_REF = 0;
volatile int16_t POS_REF = 2290; // squat (2290), stand (1940)
volatile uint8_t STATUS = 0;
volatile uint32_t pos_deg = 0;
volatile int32_t pos_err = 0; // position error
volatile int32_t pos_err_prev = 0; // previous position error, duh
volatile int32_t dpe_dt = 0; // d/dt of position error
volatile int32_t pos_err_int = 0; // integral of position error
volatile int32_t pos_err_int_max = 50000; // maximum, anti-windup
volatile int32_t pos_err_int_min = -50000; // minimum, anti-windup
volatile int16_t pos_cur = 0;
volatile uint16_t pulse_width = 0;
volatile float Kp = 20000;   // M1: 2000;    M2: ; M3:
volatile float Kd = 10000; // M1: 100000;  M2: ; M3:
volatile float Ki = 40;    // M1: 500;     M2: 100; M3:


tCANMsgObject sCANMessageR;
tCANMsgObject sCANMessageT;
uint8_t pui8MsgDataR[8];
uint8_t pui8MsgDataT[8];

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
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIO port A which is used for UART0 pins.
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0 so that we can configure the clock.
    // GPIOPinConfigure(GPIO_PA0_U0RX); // pin muxing
    // GPIOPinConfigure(GPIO_PA1_U0TX); // pin muxing
    // UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART clock source.
    // GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Select the alternate (UART) function for these pins.
    // UARTStdioConfig(0, 115200, 16000000); // Initialize the UART for console I/O.
}

//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void
MotorControllerIntHandler(void)
{
  static uint8_t LED_count = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the timer interrupt.

    pos_deg = readRLS();

    switch (MODE) {
      case IDLE:
      {
        CUR_REF = 0;
        pulse_width = set_current_mA(CUR_REF);
        break;
      }
      case CUR_CTRL:
      {
        CUR_REF = CAN_REF;
        pulse_width = set_current_mA(CUR_REF);
        break;
      }
      case POS_CTRL:
      {
        POS_REF = CAN_REF;

        // calculate error and stuff:
        pos_err = POS_REF - pos_deg;

        if ((pos_err < DEADBAND) && (pos_err > -DEADBAND)) {
          pos_err = 0;
        }

        dpe_dt = (pos_err - pos_err_prev);

        pos_err_int = pos_err_int + pos_err;

        // anti-windup:
        if (pos_err_int >= pos_err_int_max) { // ceiling
          pos_err_int = pos_err_int_max;
        } else if (pos_err_int <= pos_err_int_min) { // floor
          pos_err_int = pos_err_int_min;
        }

        // PID position controller:
        pos_cur = ((int16_t) Kp*((float) pos_err) + Ki*((float) pos_err_int) + Kd*((float) dpe_dt))/1000;

        pos_err_prev = pos_err; // set up for next time thru loop

        pulse_width = set_current_mA(pos_cur);
        break;
      }
      default:
      {
        CUR_REF = 0;
        pulse_width = set_current_mA(CUR_REF);
        UARTprintf("Unknown operating mode = %d\n", MODE);
        UARTprintf("MODE = %01X, set current to %d mA = %d pulse width.\n",MODE,CUR_REF,pulse_width);
        MODE = IDLE;
        break;
      }
    }

    (*(uint32_t *)pui8MsgDataT) = pos_deg;
    CANMessageSet(CAN0_BASE, 2, &sCANMessageT, MSG_OBJ_TYPE_TX); // write the angle to CAN

    HWREGBITW(&g_ui32Flags, 0) ^= 1; // Toggle the flag for the first timer.

    LED_count++;

    if (LED_count < POS_CTRL_FREQ/10) {
      GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, LED_GREEN);
      GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, 0);
    } else {
      GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, 0);
      GPIOPinWrite(GPIO_PORTD_BASE, LED_RED, LED_RED);
    }
    if (LED_count>=(POS_CTRL_FREQ/5)) {
      LED_count = 0;
    }}

//*****************************************************************************
//
// Set up timer to generate interrupts
//
//*****************************************************************************
void TimerBegin(){
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable the peripherals used by this example.
  IntMasterEnable(); // Enable processor interrupts.
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configure a 32-bit periodic timer.
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / POS_CTRL_FREQ);
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

    MODE = IDLE;

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

    initRLS();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LED_GREEN|LED_RED);

    if (init_copley()) {
      UARTprintf("Failed to initialize Copley Accelus.\n");
    }

    //
    // For this example CAN0 is used with RX and TX pins on port B4 and B5.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // E for custom board, B for Launchpad

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PE4_CAN0RX); // E for custom board, B for Tiva
    GPIOPinConfigure(GPIO_PE5_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
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
    canbitrate_actual = CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000);
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

    IntPrioritySet(INT_CAN0, 0x00); // set the CAN interrupt priority to be "high"

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);


    // Initialize a message object to receive CAN messages with ID 0x7001.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessageR.ui32MsgID = 0x001; // used for commanded current
    sCANMessageR.ui32MsgIDMask = 0xfffff;
    // sCANMessageR.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER |
    //                          MSG_OBJ_EXTENDED_ID);
    sCANMessageR.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
    sCANMessageR.ui32MsgLen = 8;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessageR, MSG_OBJ_TYPE_RX);

    sCANMessageT.ui32MsgID = CAN_MOTOR_ID;
    sCANMessageT.ui32MsgIDMask = 0;
    sCANMessageT.ui32Flags = (MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_EXTENDED_ID);
    sCANMessageT.ui32MsgLen = sizeof(pui8MsgDataT);
    sCANMessageT.pui8MsgData = pui8MsgDataT;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 2, &sCANMessageT, MSG_OBJ_TYPE_TX);

    TimerBegin();

    // set_copley_mode(1);
    // get_copley_mode();
    MODE = POS_CTRL;
    set_current_mA(0);

    UARTprintf("Motor %d node up!\r\nEnable mask is %d\n", MOTOR_ID, MOTOR_EN_MASK);
    GPIOPinWrite(GPIO_PORTD_BASE, LED_GREEN, LED_GREEN); // Use the flags to Toggle the LED for this timer

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
    for(;;)
    {
        //
        // If the flag for message object 1 is set, that means that the RX
        // interrupt occurred and there is a message ready to be read from
        // this CAN message object.
        //
        if(g_bRXFlag1)
        {
            //
            // Reuse the same message object that was used earlier to configure
            // the CAN for receiving messages.  A buffer for storing the
            // received data must also be provided, so set the buffer pointer
            // within the message object.  This same buffer is used for all
            // messages in this example, but your application could set a
            // different buffer each time a message is read in order to store
            // different messages in different buffers.
            //
            sCANMessageR.pui8MsgData = pui8MsgDataR;

            //
            // Read the message from the CAN.  Message object number 1 is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, 1, &sCANMessageR, 0);

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag1 = 0;

            //
            // Print information about the message just received.
            //
            // PrintCANMessageInfo(&sCANMessage, 1);
            STATUS = pui8MsgDataR[0];
            if (STATUS & MOTOR_EN_MASK) {
              MODE = STATUS & 0x01;
            } else {
              MODE = IDLE;
            }

            CAN_REF = (((pui8MsgDataR[2*MOTOR_ID]) << 8) | pui8MsgDataR[2*MOTOR_ID - 1]);
        }
        // UARTprintf("g_ui32Msg2Count = %d\n",g_ui32Msg2Count);
        UARTprintf("MODE: %02X, POS_REF: %d, POS_DEG: %d, POS_ERR: %d, dE/dt: %d, cur_cmd: %d mA, PW: %d\n",\
          MODE,POS_REF,pos_deg,pos_err,dpe_dt,pos_cur,pulse_width);
    }

    //
    // Return no errors
    //
    return(0);
}
