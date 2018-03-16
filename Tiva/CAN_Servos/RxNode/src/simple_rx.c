//*****************************************************************************
//
// simple_rx.c - Example demonstrating simple CAN message reception.
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

// The servo parts of this program are from the following reference:
// https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-general-purpose-timers/servo-control-in-any-gpio

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3

#define PI 3.14159

//*****************************************************************************
//
//! \addtogroup can_examples_list
//! <h1>Simple CAN RX (simple_rx)</h1>
//!
//! This example shows the basic setup of CAN in order to receive messages
//! from the CAN bus.  The CAN peripheral is configured to receive messages
//! with any CAN ID and then print the message contents to the console.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - CAN0 peripheral
//! - GPIO port B peripheral (for CAN0 pins)
//! - CAN0RX - PB4
//! - CAN0TX - PB5
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of CAN.
//! - GPIO port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - INT_CAN0 - CANIntHandler
//
//*****************************************************************************

//*****************************************************************************
//
// Global variables related to servo control
//
//*****************************************************************************
volatile uint32_t ServoCount = 0;
uint32_t ServoBase[6];
uint32_t ServoPin[6];
uint32_t ServoPos[6];
uint32_t ServoPosTemp[6];
uint32_t ServoNumber = 0;

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;

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
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
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

    //
    // Check to see if there is an indication that some messages were
    // lost.
    //
    if(psCANMsg->ui32Flags & MSG_OBJ_DATA_LOST)
    {
        UARTprintf("CAN message loss detected on message object %d\n",
                   ui32MsgObj);
    }

    //
    // Print out the contents of the message that was received.
    //
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

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgCount++;

        //
        // Set flag to indicate received message is pending for this message
        // object.
        //
        g_bRXFlag1 = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2.
    //
    else if(ui32Status == 2)
    {
        CANIntClear(CAN0_BASE, 2);
        g_ui32MsgCount++;
        g_bRXFlag2 = 1;
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

//*****************************************************************************
//
// Servo timer-based interrupt service routine
//
//*****************************************************************************
void ServoInterrupt(){

	//Remember to clear the interrupt flags
  uint32_t status=0;

  status = TimerIntStatus(TIMER5_BASE,true);
  TimerIntClear(TIMER5_BASE,status);

  /*
   * 	This increments the counter
   *	The counter works like a timer in PWM mode in count up mode.
   * 	The value 4000 sets the wave period.
   *	Then we have the "match" values that set the positive pulse width,
   * those are the values saved in the array ServoPos. The outputs start as HIGH and go to LOW
   * when the match value is reached
   */
  ServoCount++;
  uint32_t i;
  if(ServoCount > 4000)
	  ServoCount = 0;
  	  /*
  	   * This loads the next values into the ServoPos array
  	   * It needs to happen in the begining of a period
  	   */
	  for(i=0; i < ServoNumber; i++){
		  ServoPos[i] = ServoPosTemp[i];
	  }

  for(i=0; i < ServoNumber; i++){
	  if(ServoCount > ServoPos[i])
		  GPIOPinWrite(ServoBase[i],ServoPin[i], 0);
	  else
		  GPIOPinWrite(ServoBase[i],ServoPin[i], ServoPin[i]);
  }
}

//*****************************************************************************
//
// move the servo to a position
//
//*****************************************************************************
int32_t ServoWrite(uint32_t value, uint32_t pos){
  /*
   * This function changes the servopos
   * It takes values from 0 to 200 and converts them to the real values of 200-400.
   * It saves the position in a auxiliar array
   */

	//Check if the values are correct
	if(value > 200)
		return -1;

	//Increment 199 since the real values are from 200 to 400, not 0 to 200
	value = value + 199;

	//Save value in array for the interrupt
	ServoPosTemp[pos] = value;

	return 0;
}

//*****************************************************************************
//
// Initialize a servo
//
//*****************************************************************************
int32_t ServoAttach(uint32_t peripheral, uint32_t base, uint32_t pin){

	//Remember to check if you aren't controling more servos than the arrays can handle
	if(ServoNumber < 6){

		//Enable the GPIO you want to use
		SysCtlPeripheralEnable(peripheral);
		SysCtlDelay(3);
		//Set pin as output
		GPIOPinTypeGPIOOutput(base,pin);
		//Save which GPIO and pin you want to use
		ServoBase[ServoNumber] = base;
		ServoPin[ServoNumber] = pin;

		//Increment variable so we know we have 1 more servo
		ServoNumber++;
	}
	else
		return -1;

	return 0;
}

//*****************************************************************************
//
// Set up timer to generate interrupts
//
//*****************************************************************************
void TimerBegin(){
  /*
   * 200Khz - 5uS. (um servo precisa de um pulse de  1000uS a  2000uS ou 1mS a 2mS)
   * 200-400
   */
  //We set the load value so the timer interrupts each 1ms
  uint32_t Period;
  Period = 400;

  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  SysCtlDelay(3);
  /*
    Configure the timer as periodic, by omission it's in count down mode.
    It counts from the load value to 0 and then resets back to the load value.
  REMEMBER: You need to configure the timer before setting the load and match
  */
  TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER5_BASE, TIMER_A, Period -1);

  TimerIntRegister(TIMER5_BASE, TIMER_A, ServoInterrupt);

  /*
    Enable the timeout interrupt. In count down mode it's when the timer reaches
  0 and resets back to load. In count up mode it's when the timer reaches load
  and resets back to 0.
  */
  TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

  TimerEnable(TIMER5_BASE, TIMER_A);
}

//*****************************************************************************
//
// Configure the CAN and enter a loop to receive CAN messages.
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

    ServoAttach(SYSCTL_PERIPH_GPIOF,GPIO_PORTF_BASE,GPIO_PIN_1);
    ServoAttach(SYSCTL_PERIPH_GPIOF,GPIO_PORTF_BASE,GPIO_PIN_2);
    ServoAttach(SYSCTL_PERIPH_GPIOF,GPIO_PORTF_BASE,GPIO_PIN_3);
    TimerBegin();
    int i=0; // counter for servo writes

    tCANMsgObject sCANMessage;
    uint8_t pui8MsgData[8];

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

    // Rx: light up GREEN LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);

    //
    // For this example CAN0 is used with RX and TX pins on port B4 and B5.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

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
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    CANBitRateSet(CAN0_BASE, ui32SysClock, 500000);
#else
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 5000); // 5 kHz lulz
#endif

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


    // Initialize a message object to receive CAN messages with ID 0x1001.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessage.ui32MsgID = 0x1001;
    sCANMessage.ui32MsgIDMask = 0xfffff;
    sCANMessage.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER |
                             MSG_OBJ_EXTENDED_ID);
    sCANMessage.ui32MsgLen = 8;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Change the ID to 0x2001, and load into message object 2 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.
    //
    sCANMessage.ui32MsgID = 0x2001;
    CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

    UARTprintf("Rx node up!\n");

    uint8_t thetaChar = 0;
    uint8_t phiChar = 0;
    uint8_t psiChar = 0;

    int16_t thetaRad = 0;
    int16_t phiRad = 0;
    int16_t psiRad = 0;

    uint8_t thetaSrv = 0;
    uint8_t phiSrv = 0;
    uint8_t psiSrv = 0;

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
            sCANMessage.pui8MsgData = pui8MsgData;

            //
            // Read the message from the CAN.  Message object number 1 is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag1 = 0;

            //
            // Print information about the message just received.
            //
            PrintCANMessageInfo(&sCANMessage, 1);
            thetaChar = pui8MsgData[0];
            thetaRad = ((int16_t) 1000*((float) (PI/255.0)*thetaChar - PI));
            thetaSrv = thetaRad*(0.1/PI) + 300;
            phiChar = pui8MsgData[1];
            phiRad = ((int16_t) 1000*((float) (PI/255.0)*phiChar - PI));
            phiSrv = phiRad*(0.1/PI) + 300;
            psiChar = pui8MsgData[2];
            psiRad = ((int16_t) 1000*((float) (PI/255.0)*psiChar - PI));
            psiSrv = psiRad*(0.1/PI) + 300;

            ServoWrite(thetaSrv,0);
            ServoWrite(phiSrv,1);
            ServoWrite(psiSrv,2);

            UARTprintf("theta: %d.%3d\tphi: %d.%3d\tpsi: %d.%3d\n", \
            ((int) (thetaRad/1000)), abs((int) (thetaRad%1000)), \
            ((int) (phiRad/1000)), abs((int) (phiRad%1000)), \
            ((int) (psiRad/1000)), abs((int) (psiRad%1000)));
            UARTprintf("th: %d\tph: %d\tps: %d\n",thetaSrv,phiSrv,psiSrv);

            i++;
            if (i==200) {i = 0;} // rollover
        }

        //
        // Check for message received on message object 2.  If so then
        // read message and print information.
        //
        if(g_bRXFlag2)
        {
            sCANMessage.pui8MsgData = pui8MsgData;
            CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);
            g_bRXFlag2 = 0;
            PrintCANMessageInfo(&sCANMessage, 2);
        }

    }

    //
    // Return no errors
    //
    return(0);
}
