// code adapted from
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL#I2CCommunicationwiththeTITivaTM4C123GXL-ExampleofReadingdatafromaFreescaleMMA7455L3-axisAccelerometer

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
// #include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "i2c_master_no_int.h"
#include "LSM6DS33.h"

void InitI2C0(void);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
void LSM6DS33_init(void); // initialize LSM6DS33 IMU

void InitConsole(void) // set up UART0 to use as a console
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0 so that we can configure the clock.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);


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

int main(void)
{
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);


    // initialize the console on UART0:
    InitConsole();

    UARTprintf("Hello world!\n");

    //initialize I2C module 0
    InitI2C0();
    UARTprintf("I2C0 initialized!\n");

    int16_t Ax = 0;
    int16_t Ay = 0;
    int16_t Az = 0;
    uint8_t whoiam = 0;
    uint8_t counter = 0;
    uint8_t address = 0;

    //LSM6DS33_init();
    //UARTprintf("IMU initialized!\n");

    while(1)
    {

      //
      // Turn on the LED.
      //
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);

      //Ax = getxXL();
      //Ay = getyXL();
      //Az = getzXL();
      UARTprintf("Read #%d\n",counter);
      whoiam = WhoAmI2(address);


      UARTprintf("address: %d\tWhoIAm: %d\n",address,whoiam); //X: %d\tY: %d\tZ: %d\n",whoiam,Ax,Ay,Az);

      address++;
      counter++;

      SimpleDelay();

      //
      // Turn off the LED.
      //
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

      //UARTprintf("Hello world!\n");

      SimpleDelay();

    }

    return 0;
}
