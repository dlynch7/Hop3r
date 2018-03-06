// code adapted from
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL#I2CCommunicationwiththeTITivaTM4C123GXL-ExampleofReadingdatafromaFreescaleMMA7455L3-axisAccelerometer

#include "i2c_master_no_int.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define ACCEL_SLAVE_ADDR 0x69
#define WHOAMI 0x0F
#define XOUT8 0x06
#define XOUT_L
#define XOUT_H
#define YOUT8 0x07
#define YOUT_L
#define YOUT_H
#define ZOUT8 0x08
#define ZOUT_L
#define ZOUT_H

void InitI2C0(void);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);

void InitConsole(void) // set up UART0 to use as a console
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

void SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}

// uint8_t ReadAccel(uint8_t reg)
// {
//     uint8_t accelData =  I2CReceive(ACCEL_SLAVE_ADDR, reg);
//
//     return accelData;
// }

int main(void)
{
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    // initialize the console on UART0:
    InitConsole();

    //initialize I2C module 0
    InitI2C0();

    uint8_t Ax = 0;
    uint8_t Ay = 0;
    uint8_t Az = 0;
    uint8_t whoiam = 0;
    uint8_t counter = 0;

    UARTprintf("I2C0 initialized!\n");

    while(1)
    {
      // Ax = ReadAccel(XOUT8);
      // Ay = ReadAccel(YOUT8);
      // Az = ReadAccel(ZOUT8);
      UARTprintf("Read #%d\n",counter);
      whoiam = I2CReceive(ACCEL_SLAVE_ADDR, WHOAMI);

      UARTprintf("WhoIAm: %d\tX: %d\tY: %d\tZ: %d\n",whoiam,Ax,Ay,Az);

      counter++;

      SimpleDelay();
    }

    return 0;
}
