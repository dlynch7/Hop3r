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
#include "i2c_master_no_int.h"
#include "LSM6DS33.h"

void LSM6DS33_init(void) { // initialize LSM6DS33 IMU
  InitI2C0(); // set up I2C0 at 100 kHz

  // initialize XL (accelerometer):
  I2CSend(IMU_ADDR,2,0x10,0x82);

  // initialize G (gyroscope):
  I2CSend(IMU_ADDR,2,0x11,0x88);

  // Configure for sequential reading:
  I2CSend(IMU_ADDR,2,0x12,0x04);
}

unsigned char WhoAmI(void) {
  return I2CReceive(IMU_ADDR, WHOAMI);
}

uint16_t ReadIMU(uint8_t reg)
{
    uint8_t accelDataL =  I2CReceive(IMU_ADDR, reg);
    uint8_t accelDataH =  I2CReceive(IMU_ADDR, reg+1);

    return ((accelDataH << 8) | accelDataL);
}

signed short getxXL(void) {
  return ReadIMU(XOUT8);
}

float convxXL(void) {
  return getxXL()*0.000061;
}

signed short getyXL(void) {
  return ReadIMU(YOUT8);
}

float convyXL(void) {
  return getyXL()*0.000061;
}

signed short getzXL(void) {
  return ReadIMU(ZOUT8);
}

float convzXL(void) {
  return getzXL()*0.000061;
}
