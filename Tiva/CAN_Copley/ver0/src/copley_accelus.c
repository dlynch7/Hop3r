// copley_accelus.c
// interfaces with Copley Accelus motor driver

// See CopleyNotes.txt (parent directory) for details

// Author: Dan Lynch
// Begun 4/12/18

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "copley_accelus.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#define COPLEY_NODE_NUMBER 0

//*****************************************************************************
//
// Private functions (used only in copley_accelus.c):
//
//*****************************************************************************
static uint8_t gen_checksum(uint8_t nbytes, uint8_t *tx_packet) { // generate checksum based on packet
  // from https://youtu.be/hGEtd86k3dU
  uint8_t i = 0;
  uint8_t checksum = 0;
  // XOR each byte in the packet with the previous byte:
  for (i = 0; i < nbytes; i++) {
    checksum ^= tx_packet[i];
  }
  // XOR the result with 0x5a:
  checksum ^= 0x5a;
  // return the result:
  return checksum;
}

static uint8_t copley_send() { // send command to Copley Accelus
  uint8_t checksum = 0;
  // assemble packet without checksum
  // generate checksum from packet
  // insert checksum into packet
  // send entire packet
}
static uint32_t copley_recv() { // receive response from Copley Accelus
  // receive packet
  // parse packet
  // check checksum and error code
}

//*****************************************************************************
//
// Public functions (available to other files via copley_accelus.h):
//
//*****************************************************************************
uint8_t set_copley_mode(uint8_t copley_mode) {

}

uint8_t get_copley_mode(void) {

}

uint8_t set_current_mA(int16_t cur_ref_mA) {

}
uint16_t get_current_mA(void) {

}
