// copley_accelus.c
// interfaces with Copley Accelus motor driver

// See CopleyNotes.txt (parent directory) for details

// Author: Dan Lynch
// Begun 4/12/18

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "copley_accelus.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
// Copley Accelus op-codes
// (see "Binary-Serial-Interface-Operation-Application-Notes.pdf")
//
//*****************************************************************************
#define OPCODE_NO_OP 0
#define OPCODE_RETRIEVE_MODE 7
#define OPCODE_GET_FLASH_CRC_VAL 10
#define OPCODE_SWAP_OP_MODES 11
#define OPCODE_GET_VAR_VAL 12
#define OPCODE_SET_VAR_VAL 13
#define OPCODE_COPY_VAR_VAL 14
#define OPCODE_TRACE_CMD 15
#define OPCODE_RESET 16
#define OPCODE_TRAJ_CMD 17
#define OPCODE_ERR_LOG 18
#define OPCODE_CVM_CMD 20
#define OPCODE_ENC_CMD 27
#define OPCODE_GET_CAN_OBJ_CMD 28
#define OPCODE_SET_CAN_OBJ_CMD 29
#define OPCODE_DYN_FILE_CMD 33

//*****************************************************************************
//
// Copley Accelus Parameters (see parameter_dictionary.pdf)
//
//*****************************************************************************
#define PARAM_CUR_CMD 0x02 // current loop programmed value
#define PARAM_STATE 0x24 // desired state

//*****************************************************************************
//
// Copley Accelus state options (see parameter_dictionary.pdf)
//
//*****************************************************************************
#define STATE_DISABLED 0
#define STATE_CURR_PRG 1
#define STATE_CURR_ANR 2
#define STATE_CURR_PWM 3
#define STATE_CURR_FNC 4
#define STATE_CURR_UVM 5
// 6-10 reserved for future use
#define STATE_VEL_PRG 11
#define STATE_VEL_ANR 12
#define STATE_VEL_PWM 13
#define STATE_VEL_FNC 14
// 15-20 reserved for future use
#define STATE_POS_PRG 21
#define STATE_POS_ANR 22
#define STATE_POS_DIG 23
#define STATE_POS_FNC 24
#define STATE_POS_CAM 25
// 26-29 reserved
#define STATE_CANOPEN_ETHERCAT 30
#define STATE_MICROSTEP_TRJ 31
// 32 reserved for future use
#define STATE_MICROSTEP_DIG 33
#define STATE_MICROSTEP_FNC 34
#define STATE_MICROSTEP_CAM 35
// 36-39 reserved for future use

//*****************************************************************************
//
// Other #defines
//
//*****************************************************************************
#define HEADER_LEN 4
#define COPLEY_NODE_NUMBER 0

//*****************************************************************************
//
// Private functions (used only in copley_accelus.c):
//
//*****************************************************************************
static uint8_t init_UART1(void) { // set up UART 1 to communicate with Copley Accelus
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)) {;} // WAIT FOR UART 1 module to be ready
  // Initialzie the UART. Set baud rate, number of data bits, stop bits, parity, "stick mode" (?):
  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  return 0;
}

static uint8_t init_SSI0(void) { // set up SSI 0 for analog current control via Copley Accelus
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // enable the SSI0 peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable Port A (may cause problems???)
  // configure pin muxing:
  GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  GPIOPinConfigure(GPIO_PA3_SSI0FSS);
  GPIOPinConfigure(GPIO_PA4_SSI0RX);
  GPIOPinConfigure(GPIO_PA5_SSI0TX);
  // Give control of A2~A5 to SSI0:
  GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
  // 1 MHz, 16-bit mode:
  SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 16);

  SSIEnable(SSI0_BASE);

  return 0;
}

static uint8_t gen_checksum(uint8_t *tx_packet, uint8_t nbytes) { // generate checksum based on packet
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

// static uint8_t copley_send() { // send command to Copley Accelus
//   uint8_t checksum = 0;
//   // assemble packet without checksum
//   // generate checksum from packet
//   // insert checksum into packet
//   // send entire packet
// }
// static uint32_t copley_recv() { // receive response from Copley Accelus
//   // receive packet
//   // parse packet
//   // check checksum and error code
// }

//*****************************************************************************
//
// Public functions (available to other files via copley_accelus.h):
//
//*****************************************************************************
uint8_t init_copley(void) {
  // initialize serial communication with Copley Accelus:
  if (init_UART1()) {
    UARTprintf("Failed to establish serial communication with Copley Accelus.\n");
  }
  else {
    UARTprintf("Initialized serial communication with Copley Accelus.\n");
  }

  if(init_SSI0()) {
    UARTprintf("Failed to set up SSI0.\n");
  }
  else {
    UARTprintf("Initialized SSI 0.\n");
  }

  // TODO: Initialize Copley Accelus in a safe operating mode
  // set_copley_mode(STATE_DISABLED);
  return 0;
}

uint8_t set_copley_mode(uint8_t copley_mode) {
  uint8_t checksum = 0;
  uint8_t tx_packet[HEADER_LEN + 4];
  uint8_t i = 0; // TO-DO: delete this later

  // header:
  tx_packet[0] = COPLEY_NODE_NUMBER;
  tx_packet[1] = checksum; // checksum is still 0 at this point, it will change later
  tx_packet[2] = 2; // 2 16-bit words will follow header
  tx_packet[3] = OPCODE_SET_VAR_VAL;

  // body:
  tx_packet[4] = 0;
  tx_packet[5] = PARAM_STATE; // register to write to: "desired state" (see Parameter Dictionary)
  tx_packet[6] = 0;
  tx_packet[7] = copley_mode;

  // calculate checksum:
  checksum = gen_checksum(tx_packet,sizeof(tx_packet)/sizeof(tx_packet[0]));
  tx_packet[1] = checksum;

  // TO-DO: send the command...
  UARTprintf("Send: 0x");
  for(i = 0; i < (sizeof(tx_packet)/sizeof(tx_packet[0])); i++)
  {
      UARTprintf("%02X ", tx_packet[i]);
      // UARTCharPut(UART1_BASE, tx_packet[i]);
      // UARTCharPut(UART1_BASE, '!');
  }
  UARTprintf("\n");
  // UARTCharPut(UART1_BASE, '\r');
  // UARTCharPut(UART1_BASE, '\n');

  return 0; // TO-DO: receive, process, and return response from Copley Accelus

}

uint8_t get_copley_mode(void) {
  uint8_t checksum = 0;
  uint8_t tx_packet[HEADER_LEN + 2];
  uint8_t i = 0;

  // header:
  tx_packet[0] = COPLEY_NODE_NUMBER;
  tx_packet[1] = checksum; // checksum is still 0 at this point, it will change later
  tx_packet[2] = 1; // 1 16-bit word will follow header
  tx_packet[3] = OPCODE_GET_VAR_VAL;

  // body:
  tx_packet[4] = 0;
  tx_packet[5] = PARAM_STATE; // register to read from: "desired state" (see Parameter Dictionary)

  // calculate checksum:
  checksum = gen_checksum(tx_packet, sizeof(tx_packet)/sizeof(tx_packet[0]));
  tx_packet[1] = checksum;

  // TO-DO: send the command...
  UARTprintf("Send: 0x");
  for(i = 0; i < (sizeof(tx_packet)/sizeof(tx_packet[0])); i++)
  {
      UARTprintf("%02X ", tx_packet[i]);
      UARTCharPut(UART1_BASE, tx_packet[i]);
  }
  UARTprintf("\n");

  return 0; // TO-DO: receive, process, and return response from Copley Accelus
}

uint8_t set_current_mA(uint8_t cur_ref_mA) {
  // uint8_t checksum = 0;
  // uint8_t tx_packet[HEADER_LEN + 4];
  uint8_t channel = 0;



  SSIDataPut(SSI0_BASE, (((channel << 15)|(cur_ref_mA << 4))|0x7000));
  // Wait until SSI0 is done transferring all the data in the transmit FIFO.
  while(SSIBusy(SSI0_BASE)) {;}

  // // header:
  // tx_packet[0] = COPLEY_NODE_NUMBER;
  // tx_packet[1] = checksum; // checksum is still 0 at this point, it will change later
  // tx_packet[2] = 2; // 2 16-bit words will follow header
  // tx_packet[3] = OPCODE_SET_VAR_VAL;
  //
  // // body:
  // tx_packet[4] = 0;
  // tx_packet[5] = PARAM_CUR_CMD; // register to write to: "desired state" (see Parameter Dictionary)
  // tx_packet[6] = cur_ref_mA >> 8;
  // tx_packet[7] = cur_ref_mA & 0x00ff;
  //
  // // calculate checksum:
  // checksum = gen_checksum(tx_packet,sizeof(tx_packet)/sizeof(tx_packet[0]));
  // tx_packet[1] = checksum;
  //
  // // TO-DO: send the command...
  // for (i = 0; i < (sizeof(tx_packet)/sizeof(tx_packet[0])); i++) {
  //     // UARTsend()
  // }
  // UARTprintf("Send: 0x");
  // for(i = 0; i < (sizeof(tx_packet)/sizeof(tx_packet[0])); i++)
  // {
  //     UARTprintf("%02X ", tx_packet[i]);
  // }
  // UARTprintf("\n");

  return 0; // TO-DO: receive, process, and return response from Copley Accelus
}
uint16_t get_current_mA(void) {
  return 0;
}
