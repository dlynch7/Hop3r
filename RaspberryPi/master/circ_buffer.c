// circ_buffer.c is based on a program of the same name
// in Chapter 11 (UART) of NU's mechatronics textbook.
// Here I adapt it into a library for the Raspberry Pi.

#include <stdio.h>
#include <stdint.h>
#include "circ_buffer.h"

static volatile uint16_t read = 0, write = 0;	// circ buffer indices
static volatile uint16_t data_buf[BUFLEN][3];  // array that stores the data

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

void buffer_read(uint16_t *data_out) { // reads from current buffer index
	// assumes buffer not empty
	data_out[0] = data_buf[read][0];
	data_out[1] = data_buf[read][1];
	data_out[2] = data_buf[read][2];
	++read;
	if(read >= BUFLEN) { // wraparound
		read = 0;
	}
}

void buffer_write(uint16_t data1, uint16_t data2, uint16_t data3) { // change the buffer value indexed by "write"
	if(!buffer_full()) { // if the buffer is full, the data is lost
		data_buf[write][0] = data1;
		data_buf[write][1] = data2;
		data_buf[write][2] = data3;
		++write;
		if(write >= BUFLEN) { // wraparound
			write = 0;
		}
	}
}
