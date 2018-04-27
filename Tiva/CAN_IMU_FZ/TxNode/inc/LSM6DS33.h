#ifndef __LSM6DS33__H__
#define __LSM6DS33__H__
// Header file for LSM6DS33.c
// implements high-level IMU functions using I2C

#define IMU_ADDR 0x6B // I2C hardware address of LSM6DS33
#define WHOAMI 0x0F
#define XOUT8 0x28
#define YOUT8 0x2A
#define ZOUT8 0x2C
#define LSM_ARRAY_LEN 14 // data array has 14 values

void LSM6DS33_init(void); // initialize LSM6DS33 IMU
unsigned char WhoAmI(void); // use this to confirm communication with LSM6DS33 IMU
uint16_t ReadIMU(uint8_t reg);
signed short getxXL(void); // convert x-acceleration LSB and MSB to signed short
float convxXL(void); // convert x-acceleration signed short to float (g's)
signed short getyXL(void); // convert y-acceleration LSB and MSB to signed short
float convyXL(void); // convert y-acceleration signed short to float (g's)
signed short getzXL(void); // convert z-acceleration LSB and MSB to signed short
float convzXL(void); // convert z-acceleration signed short to float (g's)

#endif
