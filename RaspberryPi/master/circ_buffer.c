// circ_buffer.c is based on a program of the same name
// in Chapter 11 (UART) of NU's mechatronics textbook.
// Here I adapt it into a library for the Raspberry Pi.

#include <stdio.h>
#include <stdint.h>
#include "circ_buffer.h"

static volatile uint16_t read = 0, write = 0;	// circ buffer indices
static volatile uint16_t data_buf[BUFLEN];  // array that stores the data

uint16_t get_read_index(void) { // return the value of the read index
	return read;
}
uint16_t get_write_index(void) {// return the value of the write index
	return write;
}

uint8_t buffer_empty(void) { // return true if the buffer is empty (read=write)
	return read==write;
}

uint8_t buffer_full(void) { // return true if the buffer is full
	return ((write + 1) % BUFLEN) == read;
}

uint16_t buffer_read(void) { // reads from current buffer index
	// assumes buffer not empty
	uint16_t val = data_buf[read];
	++read;
	if(read >= BUFLEN) { // wraparound
		read = 0;
	}
	return val;
}

void buffer_write(uint16_t data) { // change the buffer value indexed by "write"
	if(!buffer_full()) { // if the buffer is full, the data is lost
		data_buf[write] = data;
		++write;
		if(write >= BUFLEN) { // wraparound
			write = 0;
		}
	}
}
