/*
 serial_interface.h
 Dan Lynch
 Begun May 2 2018

 Header file for serial_interface.c
 implements a serial interface on port '/dev/ttyS0'

 Reference: http://mirror.datenwolf.net/serial/

 */

#ifndef __SERIAL_INTERFACE__H__
#define __SERIAL_INTERFACE__H__

int open_port(void);
void config_port(int fd);

#endif
