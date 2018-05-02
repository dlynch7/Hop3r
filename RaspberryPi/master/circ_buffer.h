#ifndef __CIRC_BUFFER__H__
#define __CIRC_BUFFER__H__
// Header file for circ_buffer.c
// helps implement a circular buffer

// circ_buffer.c is based on a program of the same name
// in Chapter 11 (UART) of NU's mechatronics textbook.
// Here I adapt it into a library for the Raspberry Pi.

#include <stdio.h>
#include <stdint.h>

#define BUFLEN 1024
#define NSAMPLES 5000

uint16_t get_read_index(void); // return the value of the read index
uint16_t get_write_index(void);// return the value of the write index
uint8_t buffer_empty(void); // return true if the buffer is empty (read=write)
uint8_t buffer_full(void);  // return true if the buffer is full
void buffer_read(uint16_t *); // reads from current buffer index
void buffer_write(uint16_t,uint16_t,uint16_t);// change the buffer value indexed by "write"

#endif
